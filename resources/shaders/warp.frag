// (no #version line: the loader prepends '#version 330' or '#version 300 es')
// ---------------------------------------------------------------------------
// Alcubierre warp-bubble raytracer. A fragment shader drawn over the whole
// render target, one invocation per pixel:
//   1. build a past-directed null ray for the Eulerian observer at the camera
//      event (tNow, camPos)
//   2. integrate it (RK45 + autodiff Hamiltonian) while it is anywhere the
//      bubble has been, testing each accepted step as a straight segment
//   3. once it has left the journey's region for good, finish as a straight line
//   4. colour = what it hit, Doppler-shifted by the accumulated frequency ratio
// ---------------------------------------------------------------------------
out vec4 finalColor;
uniform vec2  resolution;                        // render target size in pixels

uniform vec3  camPos, camRight, camUp, camFwd;   // journey frame (x = direction of travel)
uniform float tNow;                              // coordinate time of the camera
uniform float tanHalfFov, aspect;
uniform vec3  bubX, bubY, bubZ;                  // journey-frame axes in world space
uniform float Rb, sigma;
uniform float vMax, tA, tB, tauA, pathLen;       // the journey
uniform int   pocket;                            // Van Den Broeck pocket on/off
uniform float pAlpha, pR, pD, pN;                // its alpha, R~, Delta~, n (eq. 5, 12)
uniform float pRcrit, pBcrit;                    // min of B(r) r across the wall, and where
uniform float tol;
uniform int   maxSteps;
uniform int   showShip, doppler;
uniform float glowGain, exposure, time;

float vNow;   // bubble speed at tNow (used by ship shading)

#include "dual.glsl"
#include "metric_alcubierre.glsl"
#include "geodesic_rk45.glsl"
#include "scene.glsl"

const float HORIZON_LOG = 12.0;
const float POCKET_GLOW = 0.004;  // pocket shell integrates to ~50 across vs ~0.5 for the Alcubierre wall   // e^12 ~ 1.6e5 frequency ratio: call it a horizon

vec3 doppler_grade(vec3 c, float D) {
    if (doppler == 0) return c;
    float s = clamp(log2(D), -4.0, 4.0);
    vec3 tint = s > 0.0 ? mix(vec3(1.0), vec3(0.45, 0.7, 1.8), min(s / 2.5, 1.0))
                        : mix(vec3(1.0), vec3(1.9, 0.55, 0.18), min(-s / 2.5, 1.0));
    // gentle intensity curve: true D^2..D^4 beaming saturates to white instantly
    return c * tint * clamp(pow(D, 0.8), 0.0, 2.5);
}

vec3 aces(vec3 x) {
    return clamp((x * (2.51 * x + 0.03)) / (x * (2.43 * x + 0.59) + 0.14), 0.0, 1.0);
}

void main() {
    // Texel row 0 is the top of the image (same layout the compute version wrote).
    vec2 pix = floor(gl_FragCoord.xy);
    vec2 sz = resolution;
    vNow = traj_v_at(tNow);

    float u = (2.0 * (pix.x + 0.5) / sz.x - 1.0) * aspect * tanHalfFov;
    float v = (1.0 - 2.0 * (pix.y + 0.5) / sz.y) * tanHalfFov;
    vec3 dir = normalize(camFwd + u * camRight + v * camUp);

    vec4 q = vec4(tNow, camPos);
    vec4 p = eulerian_null_momentum(q, dir);
    float logs = 0.0, glow = 0.0, glowPos = 0.0, h = 0.05;
    bool trapped = false;
    float infl = metric_influence_radius();
    float pOut = pocket_outer();

    vec3 col = vec3(0.0);
    bool done = false;
    for (int it = 0; it < maxSteps; it++) {
        vec4 k = geodesic_velocity(q, p);
        if (metric_ray_is_free(q.yzw, k.yzw)) break;          // flat from here on: finish straight
        float rr = length(alc_comoving(q));
        float hmax = rr > infl ? max(0.05, (rr - infl) / (1.0 + vMax)) : 1.0;  // stride through flat parts
        if (pocket != 0 && rr < pOut + 1.0) {
            // B jumps from 1 to 1 + alpha in a thin layer just outside pR (eq. 12 with n = 80):
            // walk up to it, never across it. Light covers 1/B^2 coordinate distance per unit h.
            float Bh = vdb_B(dual_const(rr)).f;
            float dLayer = max(max(rr - (pR + 0.15 * pD), pR - rr), 0.0);
            float hp = Bh * Bh * max(0.5 * dLayer, 0.004);
            hmax = rr < pR ? hp : min(hmax, hp);
        }
        h = min(h, hmax);
        vec4 nq, np;
        float err = rk45_try(q, p, h, tol, nq, np);
        if (err <= 1.0) {
            // treat the accepted step as a straight segment for hit tests
            Hit hit; hit.t = 1e30; hit.id = -2;
            vec3 a = q.yzw, s = nq.yzw - q.yzw;
            scene_hit(a, s, 1.0, hit);
            if (showShip != 0) {
                vec3 ac = alc_comoving(q);
                ship_hit(ac, alc_comoving(nq) - ac, 1.0, hit);
            }
            if (hit.id != -2) {
                vec3 vd = normalize(s);    // eye -> surface (the light itself travelled along -s)
                if (hit.id == -1) {
                    float tEmit = mix(q.x, nq.x, hit.t);
                    float D = exp(-logs) / max(p.x + p.y * traj_v_at(tEmit), 1e-6);  // ship rides the bubble
                    vec3 sunDir = normalize(sun_pos() - (a + s * hit.t));
                    col = doppler_grade(shade_ship(hit, vd, sunDir), D);
                } else {
                    float D = exp(-logs) / max(p.x, 1e-6);                           // static emitter
                    col = doppler_grade(shade_body(hit, vd), D);
                }
                done = true;
                break;
            }
            glow += metric_exotic_density(0.5 * (q + nq)) * length(s);
            if (pocket != 0) {
                // the pocket's energy sits in a thin shell: sample the step several times
                vec3 c0 = alc_comoving(q), c1 = alc_comoving(nq);
                if (min(length(c0), length(c1)) < pOut + length(c1 - c0)) {
                    for (int j = 0; j < 8; j++) {
                        float rho = metric_pocket_density(length(mix(c0, c1, (float(j) + 0.5) / 8.0)));
                        float dl = length(s) / 8.0;
                        if (rho < 0.0) glow -= rho * dl * POCKET_GLOW; else glowPos += rho * dl * POCKET_GLOW;
                    }
                }
            }
            q = nq; p = np;
            renormalise(p, logs);
            if (pocket != 0) {
                // Trapped light (impact parameter above the wall's minimum B r) can never get
                // past pRcrit; if float drift carries it there, it is leaking: stop it.
                vec3 c = alc_comoving(q);
                float rc = length(c);
                if (rc < pRcrit) {
                    float b = vdb_B(dual_const(rc)).f * length(cross(c, normalize(p.yzw)));
                    trapped = b > pBcrit * 1.002;
                } else if (trapped) { col = vec3(0.0); done = true; break; }
            }
            if (logs >  HORIZON_LOG) { col = vec3(0.0); done = true; break; }              // redshifted away
            if (logs < -HORIZON_LOG) { col = vec3(6.0, 7.0, 9.0); done = true; break; }    // blueshift flash
        }
        h = rk45_next_h(h, err);
    }

    // Out of steps while still inside the pocket: light bouncing off its wall forever
    // (total internal reflection). Nothing at the end of it: black.
    if (!done && pocket != 0 && length(alc_comoving(q)) < pOut) { col = vec3(0.0); done = true; }

    if (!done) {
        vec4 k = geodesic_velocity(q, p);
        vec3 d = normalize(k.yzw);
        Hit hit; hit.t = 1e30; hit.id = -2;
        float D = exp(-logs) / max(p.x, 1e-6);
        if (scene_hit(q.yzw, d, 1e7, hit)) col = shade_body(hit, d);
        else col = sky(to_world(d)) + sun_glare(q.yzw, d);
        col = doppler_grade(col, D);
    }

    col += vec3(0.75, 0.25, 1.0) * glow * glowGain;          // negative energy
    col += vec3(1.0, 0.62, 0.2) * glowPos * glowGain;        // positive energy (pocket's inner layer)
    col = aces(col * exposure);
    finalColor = vec4(pow(col, vec3(1.0 / 2.2)), 1.0);
}
