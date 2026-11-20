#ifndef METRIC_ALCUBIERRE_GLSL
#define METRIC_ALCUBIERRE_GLSL
// ---------------------------------------------------------------------------
// Alcubierre warp-drive spacetime with a full journey. THIS IS THE ONLY
// METRIC-SPECIFIC FILE.
//
//   ds^2 = -dt^2 + (dx - v(t) f(r_s) dt)^2 + dy^2 + dz^2
//   r_s  = |(x - X(t), y, z)|,    X'(t) = v(t)
//   f(r) = [tanh(s(r+R)) - tanh(s(r-R))] / (2 tanh(sR))
//
// The bubble travels along +x from X = 0 to X = pathLen. It is exactly at rest
// until tA, ramps up to vMax over tauA with the smootherstep
//   S(u) = 6u^5 - 15u^4 + 10u^3   (zero slope AND curvature at both ends),
// cruises, ramps down over tauA ending at tB, and is exactly at rest after.
// X(t) is the exact integral (I(u) = u^6 - 3u^5 + 5/2 u^4), so X' = v.
// pathLen = vMax (tB - tA - tauA). Both are Dual functions, so dH/dt -- which
// sees the acceleration -- comes out of autodiff exactly.
//
// ADM form: lapse 1, shift beta^x = -v f, flat slices. Inverse metric:
//   g^00 = -1,  g^0x = -v f,  g^xx = 1 - v^2 f^2,  g^yy = g^zz = 1.
//
// Optional Van Den Broeck pocket (C. Van Den Broeck, Class. Quantum Grav. 16
// (1999) 3973, eq. 4):
//   ds^2 = -dt^2 + B^2(r_s) [ (dx - v f(r_s) dt)^2 + dy^2 + dz^2 ]
// with B = 1 + alpha inside r_s < Rt, B = 1 beyond Rt + Dt, and in between his
// eq. 12-13:   B = alpha (-(n-1) w^n + n w^(n-1)) + 1,   w = (Rt + Dt - r_s) / Dt.
// The pocket sits where f = 1 (Rt + Dt < R). Space inside is flat but expanded by
// 1 + alpha: a region of coordinate radius Rt has proper radius (1 + alpha) Rt.
// B does not depend on v, so the pocket exists (and needs energy) at rest.
// Lapse 1, shift beta^x = -v f, spatial metric B^2 delta. Inverse metric:
//   g^00 = -1,  g^0x = -v f,  g^xx = 1/B^2 - v^2 f^2,  g^yy = g^zz = 1/B^2.
//
// Expects uniforms: float Rb, sigma, vMax, tA (start), tB (stop), tauA (ramp time), pathLen,
//                   int pocket, float pAlpha, pR (Rt), pD (Dt), pN (n).
// ---------------------------------------------------------------------------

Dual dual_tanh(Dual a) {
    float t = tanh(clamp(a.f, -15.0, 15.0));
    return Dual(t, a.s * (1.0 - t * t));
}
// smootherstep and its integral, as Dual polynomials (u in [0, 1])
Dual ramp_S(Dual u) {   // 6u^5 - 15u^4 + 10u^3
    Dual u3 = dual_mul(dual_mul(u, u), u);
    return dual_mul(u3, dual_add_scalar(dual_mul(u, dual_add_scalar(dual_mul_scalar(u, 6.0), -15.0)), 10.0));
}
Dual ramp_I(Dual u) {   // u^6 - 3u^5 + 2.5u^4
    Dual u2 = dual_mul(u, u);
    return dual_mul(dual_mul(u2, u2), dual_add_scalar(dual_mul(u, dual_add_scalar(u, -3.0)), 2.5));
}

// ---- the journey -------------------------------------------------------------
Dual traj_v(Dual t) {
    float tD = tB - tauA;
    if (t.f <= tA || t.f >= tB) return dual_const(0.0);
    if (t.f < tA + tauA) return dual_mul_scalar(ramp_S(dual_div_scalar(dual_add_scalar(t, -tA), tauA)), vMax);
    if (t.f <= tD) return dual_const(vMax);
    return dual_mul_scalar(dual_scalar_sub(1.0, ramp_S(dual_div_scalar(dual_add_scalar(t, -tD), tauA))), vMax);
}
Dual traj_x(Dual t) {
    float tD = tB - tauA;
    if (t.f <= tA) return dual_const(0.0);
    if (t.f >= tB) return dual_const(pathLen);
    if (t.f < tA + tauA) return dual_mul_scalar(ramp_I(dual_div_scalar(dual_add_scalar(t, -tA), tauA)), vMax * tauA);
    if (t.f <= tD) return dual_add_scalar(dual_mul_scalar(dual_add_scalar(t, -(tA + tauA)), vMax), 0.5 * vMax * tauA);
    Dual w = dual_div_scalar(dual_add_scalar(t, -tD), tauA);
    return dual_add_scalar(dual_mul_scalar(dual_sub(w, ramp_I(w)), vMax * tauA), vMax * (tD - tA - 0.5 * tauA));
}
float traj_v_at(float t) { return traj_v(dual_const(t)).f; }
float traj_x_at(float t) { return traj_x(dual_const(t)).f; }

// ---- the bubble ------------------------------------------------------------------
Dual alc_shape(Dual r) {
    float n = 2.0 * tanh(sigma * Rb);
    Dual a = dual_tanh(dual_mul_scalar(dual_add_scalar(r,  Rb), sigma));
    Dual b = dual_tanh(dual_mul_scalar(dual_add_scalar(r, -Rb), sigma));
    return dual_div_scalar(dual_sub(a, b), n);
}

// ---- Van Den Broeck pocket ---------------------------------------------------------------
float pocket_outer() { return pocket != 0 ? pR + pD : 0.0; }
float pocket_B_in()  { return pocket != 0 ? 1.0 + pAlpha : 1.0; }

Dual vdb_B(Dual r) {
    if (pocket == 0 || r.f >= pR + pD) return dual_const(1.0);
    if (r.f <= pR) return dual_const(1.0 + pAlpha);
    Dual w = dual_div_scalar(dual_scalar_sub(pR + pD, r), pD);           // eq. 13
    // eq. 12 written as alpha w^(n-1) (n - (n-1) w) + 1
    Dual wn1 = dual_pow(w, pN - 1.0);
    return dual_add_scalar(dual_mul_scalar(dual_mul(wn1, dual_scalar_sub(pN, dual_mul_scalar(w, pN - 1.0))), pAlpha), 1.0);
}

// Van Den Broeck eq. 11: energy density seen by Eulerian observers where B varies
// (and f = 1), times 8 pi:  B'^2/B^4 - 2 B''/B^3 - 4 B'/(r B^3).  Signed: negative
// over most of the transition, positive in a thin layer at its inner edge.
float metric_pocket_density(float r) {
    if (pocket == 0 || r <= pR || r >= pR + pD) return 0.0;
    float n = pN, w = (pR + pD - r) / pD;
    float B   = pAlpha * pow(w, n - 1.0) * (n - (n - 1.0) * w) + 1.0;
    float dBw = pAlpha * n * (n - 1.0) * pow(w, n - 2.0) * (1.0 - w);
    float d2B = pAlpha * n * (n - 1.0) * ((n - 2.0) * pow(w, n - 3.0) * (1.0 - w) - pow(w, n - 2.0));
    float Bp = -dBw / pD, Bpp = d2B / (pD * pD);
    float B3 = B * B * B;
    return Bp * Bp / (B3 * B) - 2.0 * Bpp / B3 - 4.0 * Bp / (r * B3);
}

// position relative to the bubble centre at the event q = (t, x, y, z)
vec3 alc_comoving(vec4 q) { return vec3(q.y - traj_x_at(q.x), q.z, q.w); }

// Inverse metric, packed symmetric:
//   0:g00 1:g01 2:g02 3:g03 4:g11 5:g12 6:g13 7:g22 8:g23 9:g33
void metric_ginv(Dual q[4], out Dual g[10]) {
    Dual xs = dual_sub(q[1], traj_x(q[0]));
    Dual r2 = dual_add(dual_add(dual_mul(xs, xs), dual_mul(q[2], q[2])), dual_mul(q[3], q[3]));
    Dual r  = dual_sqrt(dual_add_scalar(r2, 1e-12));
    Dual vf = dual_mul(alc_shape(r), traj_v(q[0]));
    Dual B = vdb_B(r);
    Dual iB2 = dual_scalar_div(1.0, dual_mul(B, B));                     // 1 when the pocket is off
    Dual Z = dual_const(0.0);
    g[0] = dual_const(-1.0); g[1] = dual_neg(vf); g[2] = Z; g[3] = Z;
    g[4] = dual_sub(iB2, dual_mul(vf, vf)); g[5] = Z; g[6] = Z;
    g[7] = iB2; g[8] = Z; g[9] = iB2;
}

// Beyond this comoving radius the bubble's metric is flat to ~1e-5.
float metric_influence_radius() { return Rb + 6.0 / sigma; }

// Every event the bubble ever touches lies in a capsule around the path segment
// [0, pathLen] on the x axis. Outside it spacetime is flat, and since the capsule
// is convex, a straight ray outside it moving away never comes back.
bool metric_ray_is_free(vec3 x, vec3 dir) {
    vec3 c = vec3(clamp(x.x, 0.0, pathLen), 0.0, 0.0);
    vec3 d = x - c;
    float dl = length(d);
    return dl > metric_influence_radius() && dot(d, dir) > 0.0;
}

// Eulerian energy density (up to -1/8pi) divided by v^2, faded in with speed:
//   rho / v^2 = (y^2 + z^2) f'(r)^2 / (4 r^2)        (it is NEGATIVE energy)
float metric_exotic_density(vec4 q) {
    vec3 c = alc_comoving(q);
    float r = length(c);
    Dual f = alc_shape(dual_var(r));
    float v = traj_v_at(q.x);
    return (c.y * c.y + c.z * c.z) * f.s * f.s / (4.0 * max(r * r, 1e-6)) * min(v * v, 1.0);
}

#endif
