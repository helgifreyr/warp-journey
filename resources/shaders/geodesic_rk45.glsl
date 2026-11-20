#ifndef GEODESIC_RK45_GLSL
#define GEODESIC_RK45_GLSL
// ---------------------------------------------------------------------------
// Metric-agnostic null-geodesic machinery. Needs: dual.glsl and a metric file
// providing  void metric_ginv(Dual q[4], out Dual g[10]).
//
//   H = 1/2 g^{mu nu}(q) p_mu p_nu,   dq/dl = dH/dp,   dp/dl = -dH/dq
//
// dH/dq comes from forward-mode autodiff: one Dual evaluation of the inverse
// metric per coordinate (t, x, y, z) -- time included, since a moving warp
// bubble makes the metric time dependent.
//
// Why RK45 and not the black-hole project's symplectic (Tao) scheme: the
// feasibility test showed Tao's doubled phase space decouples once the shift
// is superluminal. Dormand-Prince stays accurate and needs ~20-40 steps a ray.
// ---------------------------------------------------------------------------

float ginv_quad(Dual g[10], vec4 p, bool deriv) {
    #define G(i) (deriv ? g[i].s : g[i].f)
    return G(0)*p.x*p.x + G(4)*p.y*p.y + G(7)*p.z*p.z + G(9)*p.w*p.w
         + 2.0 * (G(1)*p.x*p.y + G(2)*p.x*p.z + G(3)*p.x*p.w
                + G(5)*p.y*p.z + G(6)*p.y*p.w + G(8)*p.z*p.w);
    #undef G
}

vec4 ginv_raise(Dual g[10], vec4 p) {
    return vec4(g[0].f*p.x + g[1].f*p.y + g[2].f*p.z + g[3].f*p.w,
                g[1].f*p.x + g[4].f*p.y + g[5].f*p.z + g[6].f*p.w,
                g[2].f*p.x + g[5].f*p.y + g[7].f*p.z + g[8].f*p.w,
                g[3].f*p.x + g[6].f*p.y + g[8].f*p.z + g[9].f*p.w);
}

void metric_ginv_at(vec4 q, out Dual g[10]) {
    Dual d[4];
    for (int i = 0; i < 4; i++) d[i] = dual_const(q[i]);
    metric_ginv(d, g);
}

// contravariant direction of travel k^mu = g^{mu nu} p_nu
vec4 geodesic_velocity(vec4 q, vec4 p) {
    Dual g[10]; metric_ginv_at(q, g);
    return ginv_raise(g, p);
}

void hamilton_rhs(vec4 q, vec4 p, out vec4 dq, out vec4 dp) {
    Dual d[4]; Dual g[10];
    for (int w = 0; w < 4; w++) {
        for (int i = 0; i < 4; i++) d[i] = Dual(q[i], i == w ? 1.0 : 0.0);
        metric_ginv(d, g);
        dp[w] = -0.5 * ginv_quad(g, p, true);
        if (w == 0) dq = ginv_raise(g, p);
    }
}

// Past-directed null momentum seen by the Eulerian (normal) observer at q looking
// along spatial coordinate direction dir. Works for any metric via 3+1 split:
//   alpha^2 = -1/g^00,  beta^i = alpha^2 g^0i,  gamma^ij = g^ij + beta^i beta^j / alpha^2
//   k = -n + e  =>  p_mu = (alpha + beta_i e^i, gamma_ij e^j),  e normalised by gamma.
// At the observer, -p.u = 1, so later Doppler factors are relative to this.
vec4 eulerian_null_momentum(vec4 q, vec3 dir) {
    Dual g[10]; metric_ginv_at(q, g);
    float a2 = -1.0 / g[0].f;
    float alpha = sqrt(a2);
    vec3 beta = a2 * vec3(g[1].f, g[2].f, g[3].f);
    mat3 gup = mat3(g[4].f, g[5].f, g[6].f,
                    g[5].f, g[7].f, g[8].f,
                    g[6].f, g[8].f, g[9].f) + outerProduct(beta, beta) / a2;
    mat3 gdn = inverse(gup);
    vec3 e = dir / sqrt(max(dot(dir, gdn * dir), 1e-12));
    vec3 e_low = gdn * e;
    return vec4(alpha + dot(beta, e_low), e_low);
}

// One Dormand-Prince 5(4) trial step. Returns the error ratio (<=1: accept).
float rk45_try(vec4 q, vec4 p, float h, float tol, out vec4 nq, out vec4 np) {
    vec4 k1q,k1p,k2q,k2p,k3q,k3p,k4q,k4p,k5q,k5p,k6q,k6p,k7q,k7p;
    hamilton_rhs(q, p, k1q, k1p);
    hamilton_rhs(q + h*(0.2*k1q), p + h*(0.2*k1p), k2q, k2p);
    hamilton_rhs(q + h*(3.0/40.0*k1q + 9.0/40.0*k2q),
                 p + h*(3.0/40.0*k1p + 9.0/40.0*k2p), k3q, k3p);
    hamilton_rhs(q + h*(44.0/45.0*k1q - 56.0/15.0*k2q + 32.0/9.0*k3q),
                 p + h*(44.0/45.0*k1p - 56.0/15.0*k2p + 32.0/9.0*k3p), k4q, k4p);
    hamilton_rhs(q + h*(19372.0/6561.0*k1q - 25360.0/2187.0*k2q + 64448.0/6561.0*k3q - 212.0/729.0*k4q),
                 p + h*(19372.0/6561.0*k1p - 25360.0/2187.0*k2p + 64448.0/6561.0*k3p - 212.0/729.0*k4p), k5q, k5p);
    hamilton_rhs(q + h*(9017.0/3168.0*k1q - 355.0/33.0*k2q + 46732.0/5247.0*k3q + 49.0/176.0*k4q - 5103.0/18656.0*k5q),
                 p + h*(9017.0/3168.0*k1p - 355.0/33.0*k2p + 46732.0/5247.0*k3p + 49.0/176.0*k4p - 5103.0/18656.0*k5p), k6q, k6p);
    nq = q + h*(35.0/384.0*k1q + 500.0/1113.0*k3q + 125.0/192.0*k4q - 2187.0/6784.0*k5q + 11.0/84.0*k6q);
    np = p + h*(35.0/384.0*k1p + 500.0/1113.0*k3p + 125.0/192.0*k4p - 2187.0/6784.0*k5p + 11.0/84.0*k6p);
    hamilton_rhs(nq, np, k7q, k7p);
    vec4 eq = h*(71.0/57600.0*k1q - 71.0/16695.0*k3q + 71.0/1920.0*k4q - 17253.0/339200.0*k5q + 22.0/525.0*k6q - 1.0/40.0*k7q);
    vec4 ep = h*(71.0/57600.0*k1p - 71.0/16695.0*k3p + 71.0/1920.0*k4p - 17253.0/339200.0*k5p + 22.0/525.0*k6p - 1.0/40.0*k7p);
    vec4 r1 = abs(eq) / (tol * (1.0 + abs(q)));
    vec4 r2 = abs(ep) / (tol * (1.0 + abs(p)));
    return max(max(max(r1.x, r1.y), max(r1.z, r1.w)), max(max(r2.x, r2.y), max(r2.z, r2.w)));
}

float rk45_next_h(float h, float err) {
    return h * clamp(0.9 * pow(max(err, 1e-10), -0.2), 0.2, 4.0);
}

// Null geodesics are unchanged by p -> c p. Light near a warp horizon blueshifts
// by hundreds of e-folds, so rescale every step and keep the log of the scale.
void renormalise(inout vec4 p, inout float log_scale) {
    float s = length(p.yzw);
    p /= s;
    log_scale += log(s);
}

#endif
