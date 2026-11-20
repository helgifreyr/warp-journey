// CPU reference for the shader physics (double precision). `make test`.
// Mirrors metric_alcubierre.glsl + geodesic_rk45.glsl and checks them against
// things we know independently. Exit code 0 = all passed.
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "../src/physics/dual.h"

// journey = 0: constant-speed bubble at v.  journey = 1: the game's trip, with
// speed profile v(t) peaking at vmax = v, exactly as in metric_alcubierre.glsl.
typedef struct { double v, R, sigma; int journey; double tA, tB, tau; } Params;

static Dual dual_tanh(Dual a) { double t = tanh(a.f); return dual_create(t, a.s * (1.0 - t * t)); }
static Dual shape(Dual r, const Params* P) {
    double n = 2.0 * tanh(P->sigma * P->R);
    Dual a = dual_tanh(dual_mul_scalar(dual_add_scalar(r,  P->R), P->sigma));
    Dual b = dual_tanh(dual_mul_scalar(dual_add_scalar(r, -P->R), P->sigma));
    return dual_div_scalar(dual_sub(a, b), n);
}
static Dual ramp_S(Dual u) {
    Dual u3 = dual_mul(dual_mul(u, u), u);
    return dual_mul(u3, dual_add_scalar(dual_mul(u, dual_add_scalar(dual_mul_scalar(u, 6.0), -15.0)), 10.0));
}
static Dual ramp_I(Dual u) {
    Dual u2 = dual_mul(u, u);
    return dual_mul(dual_mul(u2, u2), dual_add_scalar(dual_mul(u, dual_add_scalar(u, -3.0)), 2.5));
}
// journey: at rest until tA, ramp over tau, cruise, ramp down ending at tB
static Dual traj_v(Dual t, const Params* P) {
    double tD = P->tB - P->tau;
    if (t.f <= P->tA || t.f >= P->tB) return dual_constant(0);
    if (t.f < P->tA + P->tau) return dual_mul_scalar(ramp_S(dual_div_scalar(dual_add_scalar(t, -P->tA), P->tau)), P->v);
    if (t.f <= tD) return dual_constant(P->v);
    return dual_mul_scalar(dual_scalar_sub(1.0, ramp_S(dual_div_scalar(dual_add_scalar(t, -tD), P->tau))), P->v);
}
static Dual traj_x(Dual t, const Params* P) {
    double tD = P->tB - P->tau, L = P->v * (P->tB - P->tA - P->tau);
    if (t.f <= P->tA) return dual_constant(0);
    if (t.f >= P->tB) return dual_constant(L);
    if (t.f < P->tA + P->tau) return dual_mul_scalar(ramp_I(dual_div_scalar(dual_add_scalar(t, -P->tA), P->tau)), P->v * P->tau);
    if (t.f <= tD) return dual_add_scalar(dual_mul_scalar(dual_add_scalar(t, -(P->tA + P->tau)), P->v), 0.5 * P->v * P->tau);
    Dual w = dual_div_scalar(dual_add_scalar(t, -tD), P->tau);
    return dual_add_scalar(dual_mul_scalar(dual_sub(w, ramp_I(w)), P->v * P->tau), P->v * (tD - P->tA - 0.5 * P->tau));
}
static void ginv_dual(const Dual q[4], const Params* P, Dual g[10]) {
    Dual X = P->journey ? traj_x(q[0], P) : dual_mul_scalar(q[0], P->v);
    Dual V = P->journey ? traj_v(q[0], P) : dual_constant(P->v);
    Dual xs = dual_sub(q[1], X);
    Dual r2 = dual_add(dual_add(dual_mul(xs, xs), dual_mul(q[2], q[2])), dual_mul(q[3], q[3]));
    Dual vf = dual_mul(shape(dual_sqrt(dual_add_scalar(r2, 1e-24)), P), V);
    Dual Z = dual_constant(0), O = dual_constant(1);
    g[0] = dual_constant(-1); g[1] = dual_neg(vf); g[2] = Z; g[3] = Z;
    g[4] = dual_sub(O, dual_mul(vf, vf)); g[5] = Z; g[6] = Z; g[7] = O; g[8] = Z; g[9] = O;
}
static double quad(const Dual g[10], const double p[4], int d) {
#define G(i) (d ? g[i].s : g[i].f)
    return G(0)*p[0]*p[0] + G(4)*p[1]*p[1] + G(7)*p[2]*p[2] + G(9)*p[3]*p[3]
         + 2*(G(1)*p[0]*p[1] + G(2)*p[0]*p[2] + G(3)*p[0]*p[3] + G(5)*p[1]*p[2] + G(6)*p[1]*p[3] + G(8)*p[2]*p[3]);
#undef G
}
static double H(const double q[4], const double p[4], const Params* P) {
    Dual d[4], g[10]; for (int i = 0; i < 4; i++) d[i] = dual_constant(q[i]);
    ginv_dual(d, P, g); return 0.5 * quad(g, p, 0);
}
static void rhs(const double y[8], const Params* P, double k[8]) {
    Dual d[4], g[10];
    for (int w = 0; w < 4; w++) {
        for (int i = 0; i < 4; i++) d[i] = (i == w) ? dual_variable(y[i]) : dual_constant(y[i]);
        ginv_dual(d, P, g);
        k[4 + w] = -0.5 * quad(g, y + 4, 1);
        if (w == 0) {
            const double* p = y + 4;
            k[0] = g[0].f*p[0] + g[1].f*p[1] + g[2].f*p[2] + g[3].f*p[3];
            k[1] = g[1].f*p[0] + g[4].f*p[1] + g[5].f*p[2] + g[6].f*p[3];
            k[2] = g[2].f*p[0] + g[5].f*p[1] + g[7].f*p[2] + g[8].f*p[3];
            k[3] = g[3].f*p[0] + g[6].f*p[1] + g[8].f*p[2] + g[9].f*p[3];
        }
    }
}
static int rk45(double y[8], double* h, double tol, const Params* P) {
    double k1[8],k2[8],k3[8],k4[8],k5[8],k6[8],k7[8],t[8],yn[8], d = *h;
    rhs(y,P,k1);
    for(int i=0;i<8;i++) t[i]=y[i]+d*(0.2*k1[i]); rhs(t,P,k2);
    for(int i=0;i<8;i++) t[i]=y[i]+d*(3./40*k1[i]+9./40*k2[i]); rhs(t,P,k3);
    for(int i=0;i<8;i++) t[i]=y[i]+d*(44./45*k1[i]-56./15*k2[i]+32./9*k3[i]); rhs(t,P,k4);
    for(int i=0;i<8;i++) t[i]=y[i]+d*(19372./6561*k1[i]-25360./2187*k2[i]+64448./6561*k3[i]-212./729*k4[i]); rhs(t,P,k5);
    for(int i=0;i<8;i++) t[i]=y[i]+d*(9017./3168*k1[i]-355./33*k2[i]+46732./5247*k3[i]+49./176*k4[i]-5103./18656*k5[i]); rhs(t,P,k6);
    for(int i=0;i<8;i++) yn[i]=y[i]+d*(35./384*k1[i]+500./1113*k3[i]+125./192*k4[i]-2187./6784*k5[i]+11./84*k6[i]); rhs(yn,P,k7);
    double err = 0;
    for (int i = 0; i < 8; i++) {
        double e = d*(71./57600*k1[i]-71./16695*k3[i]+71./1920*k4[i]-17253./339200*k5[i]+22./525*k6[i]-1./40*k7[i]);
        err = fmax(err, fabs(e) / (tol * (1 + fabs(y[i]))));
    }
    *h = d * fmin(4.0, fmax(0.2, 0.9 * pow(fmax(err, 1e-10), -0.2)));
    if (err <= 1) { memcpy(y, yn, sizeof yn); return 1; }
    return 0;
}
static double renorm(double p[4]) { double s = sqrt(p[1]*p[1]+p[2]*p[2]+p[3]*p[3]); for (int i=0;i<4;i++) p[i]/=s; return log(s); }

static int failures = 0;
static void check(const char* name, int ok, const char* detail) {
    printf("  [%s] %-46s %s\n", ok ? "PASS" : "FAIL", name, detail);
    if (!ok) failures++;
}

int main(void) {
    Params P = { 2.0, 1.0, 8.0, 0, 0, 0, 0 };
    char buf[160];
    printf("Alcubierre physics checks (CPU, double)\n");

    // 1. autodiff gradient vs central finite differences
    double worst = 0;
    double pts[][4] = { {0,0,0,0}, {0.3,1.0,0.2,0.1}, {-1.2,-2.1,0.4,0.3}, {0,0.95,0.3,0}, {2,3,-1,0.5} };
    double pp[4] = { 1.3, 0.4, -0.7, 0.2 };
    for (double v = 0.5; v <= 4.01; v *= 2) {
        Params Q = P; Q.v = v;
        for (int k = 0; k < 5; k++) {
            double y[8], dy[8]; memcpy(y, pts[k], 32); memcpy(y + 4, pp, 32); rhs(y, &Q, dy);
            for (int w = 0; w < 4; w++) {
                double a[4], b[4], h = 1e-5; memcpy(a, pts[k], 32); memcpy(b, pts[k], 32); a[w] += h; b[w] -= h;
                worst = fmax(worst, fabs(-dy[4 + w] - (H(a, pp, &Q) - H(b, pp, &Q)) / (2 * h)));
            }
        }
    }
    snprintf(buf, sizeof buf, "max |AD - FD| = %.1e", worst);
    check("autodiff dH/dq matches finite differences", worst < 1e-5, buf);

    // 2. Eulerian camera ray is null (formula used in eulerian_null_momentum)
    {
        double q[4] = { 0, 0.2, 0.1, 0 }, d[3] = { 0.6, 0.64, 0.48 };
        double r = sqrt(q[1]*q[1] + q[2]*q[2]);
        double beta = -P.v * shape(dual_constant(r), &P).f;
        double p[4] = { 1 + beta * d[0], d[0], d[1], d[2] };
        snprintf(buf, sizeof buf, "H = %.1e", H(q, p, &P));
        check("camera ray satisfies H = 0", fabs(H(q, p, &P)) < 1e-12, buf);
    }

    // 3. v = 0 is flat: straight lines
    {
        Params Q = P; Q.v = 0;
        double y[8] = { 0,0,0,0, 1, 0.48, 0.6, 0.64 }, h = 0.1;
        while (-y[0] < 10) rk45(y, &h, 1e-9, &Q);
        double s = -y[0], e = fabs(y[1] - 0.48*s) + fabs(y[2] - 0.6*s) + fabs(y[3] - 0.64*s);
        snprintf(buf, sizeof buf, "deviation %.1e", e);
        check("flat limit gives straight rays", e < 1e-9, buf);
    }

    // 4. analytic front-wall horizon: forward photon stalls where f(r) = 1 - 1/v
    for (double v = 1.5; v <= 4.01; v += 1.25) {
        Params Q = P; Q.v = v;
        double lo = 0, hi = 3;
        for (int i = 0; i < 80; i++) { double m = (lo+hi)/2; if (shape(dual_constant(m), &Q).f > 1 - 1/v) lo = m; else hi = m; }
        double y[8] = { 0,0,0,0, -1 - v, 1, 0, 0 }, h = 0.01, L = 0;
        while (y[0] < 40) if (rk45(y, &h, 1e-7, &Q)) L += renorm(y + 4);
        double r = y[1] - v * y[0];
        snprintf(buf, sizeof buf, "r=%.5f analytic %.5f (blueshift e^%.0f)", r, lo, L);
        char name[64]; snprintf(name, sizeof name, "horizon position, v = %.2f", v);
        check(name, fabs(r - lo) < 1e-4, buf);
    }

    // 5. superluminal ship: looking straight back hits the rear horizon,
    //    looking forward escapes
    {
        Params Q = P; Q.v = 3.0;
        int results[2];
        for (int k = 0; k < 2; k++) {
            double dx = k == 0 ? -1 : 1;
            double y[8] = { 0, 0.05, 0, 0, 1 - 3.0 * dx, dx, 0, 0 }, h = 0.05, L = 0;
            int n = 0, out = 0;
            while (n++ < 5000) {
                if (rk45(y, &h, 1e-6, &Q)) L += renorm(y + 4);
                if (fabs(L) > 12) break;
                double xs = y[1] - 3.0 * y[0];
                if (sqrt(xs*xs + y[2]*y[2] + y[3]*y[3]) > 6) { out = 1; break; }
            }
            results[k] = out;
        }
        check("v=3: rear view is a horizon, front view escapes", !results[0] && results[1],
              results[0] ? "rear ray escaped" : (results[1] ? "ok" : "front ray trapped"));
    }

    // 6. journey trajectory: exactly at rest before departure, X' = v, X 0 -> L
    {
        Params J = { 4.0, 2.2, 6.0, 1, 0.0, 44.0, 4.0 };
        double worstd = 0;
        for (double t = -10; t <= 60; t += 0.7) {
            Dual X = traj_x(dual_variable(t), &J);
            worstd = fmax(worstd, fabs(X.s - traj_v(dual_constant(t), &J).f));
        }
        double x0 = traj_x(dual_constant(-0.001), &J).f, x1 = traj_x(dual_constant(44.001), &J).f;
        double vm = traj_v(dual_constant(20), &J).f;
        double xm = traj_x(dual_constant(44.0 - 1e-6), &J).f, vb = traj_v(dual_constant(-0.5), &J).f;
        worstd = fmax(worstd, fabs(xm - 160) + fabs(vb));   // continuous into the stop, zero before start
        snprintf(buf, sizeof buf, "|X'-v| %.1e  X %.4f -> %.4f  v_mid %.4f", worstd, x0, x1, vm);
        check("journey: at rest before/after, X' = v, X 0 -> 160",
              worstd < 1e-9 && fabs(x0) < 1e-6 && fabs(x1 - 160) < 1e-6 && fabs(vm - 4) < 1e-6, buf);
    }

    // 7. journey metric: autodiff gradient, including d/dt through the
    //    acceleration, matches finite differences
    {
        Params J = { 4.0, 2.2, 6.0, 1, 0.0, 44.0, 4.0 };
        double worstj = 0;
        double ev[][4] = { {1.3, 0.9, 0.5, 0.2}, {2.7, 7.0, 1.8, 0.4}, {42.5, 158.0, 1.2, -0.9}, {20, 72.5, 2.0, 0.3} };
        double pj[4] = { 1.1, 0.6, -0.4, 0.3 };
        for (int k = 0; k < 4; k++) {
            double y[8], dy[8];
            memcpy(y, ev[k], 32); memcpy(y + 4, pj, 32); rhs(y, &J, dy);
            for (int w = 0; w < 4; w++) {
                double a[4], b[4], h = 1e-5;
                memcpy(a, ev[k], 32); memcpy(b, ev[k], 32); a[w] += h; b[w] -= h;
                worstj = fmax(worstj, fabs(-dy[4 + w] - (H(a, pj, &J) - H(b, pj, &J)) / (2 * h)));
            }
        }
        snprintf(buf, sizeof buf, "max |AD - FD| = %.1e", worstj);
        check("journey metric: autodiff incl. d/dt matches FD", worstj < 1e-5, buf);
    }

    printf(failures ? "%d check(s) FAILED\n" : "all checks passed\n", failures);
    return failures ? 1 : 0;
}
