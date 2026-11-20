#ifndef SCENE_GLSL
#define SCENE_GLSL
// ---------------------------------------------------------------------------
// Scene content: procedural sky, star + planets (+ rings), and the warp ship.
// Pure art -- no relativity in here. Everything is in the BUBBLE FRAME (x =
// direction of travel, bubble centre at the origin at t = 0); bubX/Y/Z map
// directions back to world space so textures don't swim when the ship turns.
//
// Body layout (4 x vec4 per body in the bodyData uniform array):
//   posRad : xyz centre, w radius
//   col1   : rgb primary colour,   w type (0 star, 1 rocky, 2 gas giant, 3 ice)
//   col2   : rgb secondary colour, w ring inner radius (0 = no ring)
//   axis   : xyz spin/ring axis,   w ring outer radius
// Body 0 must be the star (it lights everything).
// ---------------------------------------------------------------------------

struct Body { vec4 posRad; vec4 col1; vec4 col2; vec4 axis; };
// Plain uniforms rather than a storage buffer, so this also runs on OpenGL 3.3,
// macOS and WebGL2. 32 bodies = 128 vec4s, within WebGL2's guaranteed 224.
#define MAX_BODIES 32
uniform int  numBodies;
uniform vec4 bodyData[4 * MAX_BODIES];
Body body_at(int i) { return Body(bodyData[4 * i], bodyData[4 * i + 1], bodyData[4 * i + 2], bodyData[4 * i + 3]); }

vec3 to_world(vec3 d) { return bubX * d.x + bubY * d.y + bubZ * d.z; }

// ---- noise -----------------------------------------------------------------
uint hash_u(uvec3 v) {
    v = v * 1664525u + 1013904223u;
    v.x += v.y * v.z; v.y += v.z * v.x; v.z += v.x * v.y;
    v ^= v >> 16u;
    v.x += v.y * v.z; v.y += v.z * v.x; v.z += v.x * v.y;
    return v.x ^ v.y ^ v.z;
}
float hash13(vec3 p) { return float(hash_u(uvec3(ivec3(floor(p)) + 32768))) / 4294967295.0; }
vec3 hash33(vec3 p) {
    uint h = hash_u(uvec3(ivec3(floor(p)) + 32768));
    return vec3(h & 1023u, (h >> 10) & 1023u, (h >> 20) & 1023u) / 1023.0;
}
float vnoise(vec3 p) {
    vec3 i = floor(p), f = fract(p);
    f = f * f * (3.0 - 2.0 * f);
    return mix(mix(mix(hash13(i), hash13(i + vec3(1,0,0)), f.x),
                   mix(hash13(i + vec3(0,1,0)), hash13(i + vec3(1,1,0)), f.x), f.y),
               mix(mix(hash13(i + vec3(0,0,1)), hash13(i + vec3(1,0,1)), f.x),
                   mix(hash13(i + vec3(0,1,1)), hash13(i + vec3(1,1,1)), f.x), f.y), f.z);
}
float fbm(vec3 p) {
    float a = 0.5, s = 0.0;
    for (int i = 0; i < 5; i++) { s += a * vnoise(p); p = p * 2.03 + 17.1; a *= 0.5; }
    return s;
}

// ---- sky ---------------------------------------------------------------------
vec3 star_layer(vec3 d, float K, float density, float bright) {
    vec3 cell = floor(d * K);
    vec3 h = hash33(cell);
    if (h.x > density) return vec3(0.0);
    vec3 c = normalize((cell + 0.25 + 0.5 * hash33(cell + 91.0)) / K);
    float dist = length(d - c) * K;
    float b = exp(-dist * dist * 90.0) * bright * (0.2 + 1.6 * h.y * h.y * h.y);
    vec3 tint = mix(vec3(1.0, 0.75, 0.55), vec3(0.65, 0.8, 1.0), h.z);
    return tint * b;
}

vec3 sky(vec3 dW) {
    vec3 galN = normalize(vec3(0.2, 1.0, 0.35));
    float band = exp(-pow(dot(dW, galN) * 2.6, 2.0));
    float n1 = fbm(dW * 3.0), n2 = fbm(dW * 6.0 + 11.0);
    vec3 neb = mix(vec3(0.35, 0.12, 0.45), vec3(0.08, 0.30, 0.45), n2);
    vec3 col = neb * band * pow(n1, 2.5) * 0.9 + vec3(0.004, 0.005, 0.010);
    col += vec3(0.8, 0.55, 0.35) * band * pow(fbm(dW * 14.0), 4.0) * 0.25;
    col += star_layer(dW, 60.0, 0.025, 2.0);
    col += star_layer(dW, 150.0, 0.035, 0.8);
    col += star_layer(dW, 360.0, 0.05, 0.35) * (0.3 + band);
    return col;
}

// ---- intersections -------------------------------------------------------------
struct Hit { float t; int id; int kind; vec3 pos; vec3 n; };  // kind: 0 sphere, 1 ring, 2 hull, 3 warp ring

float ring_density(int id, float r) {
    Body b = body_at(id);
    float ri = b.col2.w, ro = b.axis.w;
    float u = (r - ri) / (ro - ri);
    float bands = 0.55 + 0.45 * sin(u * 61.0 + 4.0 * vnoise(vec3(u * 40.0, float(id), 0.0)));
    float gaps = smoothstep(0.02, 0.06, abs(u - 0.62)) * smoothstep(0.01, 0.03, abs(u - 0.31));
    float edge = smoothstep(0.0, 0.05, u) * smoothstep(1.0, 0.9, u);
    return bands * gaps * edge;
}

// Nearest scene hit on the segment a -> a + s*t, t in [0, tmax].
bool scene_hit(vec3 a, vec3 s, float tmax, inout Hit hit) {
    bool found = false;
    for (int i = 0; i < numBodies; i++) {
        Body b = body_at(i);
        vec3 oc = a - b.posRad.xyz;
        float A = dot(s, s), B = dot(oc, s), C = dot(oc, oc) - b.posRad.w * b.posRad.w;
        float disc = B * B - A * C;
        if (disc >= 0.0) {
            float t = (-B - sqrt(disc)) / A;
            if (t >= 0.0 && t <= tmax && t < hit.t) {
                hit.t = t; hit.id = i; hit.kind = 0; hit.pos = a + s * t;
                hit.n = normalize(hit.pos - b.posRad.xyz); found = true;
            }
        }
        if (b.col2.w > 0.0) {
            vec3 ax = b.axis.xyz;
            float den = dot(s, ax);
            if (abs(den) > 1e-12) {
                float t = -dot(oc, ax) / den;
                if (t >= 0.0 && t <= tmax && t < hit.t) {
                    vec3 pp = oc + s * t;
                    float r = length(pp);
                    if (r > b.col2.w && r < b.axis.w && ring_density(i, r) > 0.35) {
                        hit.t = t; hit.id = i; hit.kind = 1; hit.pos = a + s * t;
                        hit.n = ax * -sign(den); found = true;
                    }
                }
            }
        }
    }
    return found;
}

// Ship, in COMOVING coordinates (it rides at the bubble centre). Hull is an
// ellipsoid; the "warp ring" is a flat annulus around it, facing forward.
float ship_scale() { return min(1.0, Rb / 1.6); }
bool ship_hit(vec3 a, vec3 s, float tmax, inout Hit hit) {
    float k = ship_scale();
    bool found = false;
    vec3 rad = vec3(0.62, 0.13, 0.19) * k;
    vec3 ao = a / rad, so = s / rad;
    float A = dot(so, so), B = dot(ao, so), C = dot(ao, ao) - 1.0;
    float disc = B * B - A * C;
    if (disc >= 0.0) {
        float t = (-B - sqrt(disc)) / A;
        if (t >= 0.0 && t <= tmax && t < hit.t) {
            hit.t = t; hit.id = -1; hit.kind = 2; hit.pos = a + s * t;
            hit.n = normalize(hit.pos / (rad * rad)); found = true;
        }
    }
    float rx = -0.12 * k;
    if (abs(s.x) > 1e-12) {
        float t = (rx - a.x) / s.x;
        if (t >= 0.0 && t <= tmax && t < hit.t) {
            vec3 pp = a + s * t;
            float r = length(pp.yz);
            if (r > 0.86 * k && r < 0.98 * k) {
                hit.t = t; hit.id = -1; hit.kind = 3; hit.pos = pp;
                hit.n = vec3(-sign(s.x), 0.0, 0.0); found = true;
            }
        }
    }
    return found;
}

// ---- shading (linear HDR) -----------------------------------------------------------
vec3 sun_pos() { return body_at(0).posRad.xyz; }
const float SUN_POWER = 2.2;

bool sphere_blocks(vec3 o, vec3 d, int id) {
    vec3 oc = o - body_at(id).posRad.xyz;
    float B = dot(oc, d), C = dot(oc, oc) - body_at(id).posRad.w * body_at(id).posRad.w;
    return B < 0.0 && B * B - C > 0.0;
}

vec3 shade_body(Hit h, vec3 viewDir) {
    Body b = body_at(h.id);
    int type = int(b.col1.w + 0.5);
    vec3 L = normalize(sun_pos() - h.pos);
    if (h.kind == 1) {
        float r = length(h.pos - b.posRad.xyz);
        float u = (r - b.col2.w) / (b.axis.w - b.col2.w);
        vec3 alb = mix(b.col2.rgb, b.col1.rgb, vnoise(vec3(u * 90.0, 3.0, float(h.id))));
        float lit = 0.25 + 0.75 * abs(dot(b.axis.xyz, L));
        if (sphere_blocks(h.pos, L, h.id)) lit *= 0.06;
        return alb * lit * SUN_POWER * 0.8;
    }
    if (type == 0) {
        float mu = max(dot(h.n, -viewDir), 0.0);
        float gran = fbm(to_world(h.n) * 18.0);
        return b.col1.rgb * (0.35 + 0.65 * pow(mu, 0.45)) * (0.8 + 0.4 * gran) * 9.0;
    }
    vec3 nw = to_world(h.n);
    float ndl = max(dot(h.n, L), 0.0);
    float rim = pow(1.0 - max(dot(h.n, -viewDir), 0.0), 4.0);
    vec3 alb, atm = vec3(0.0);
    if (type == 1) {
        float f = fbm(nw * 3.5 + float(h.id) * 7.0);
        float land = smoothstep(0.50, 0.53, f);
        alb = mix(b.col2.rgb, b.col1.rgb * (0.7 + 0.6 * fbm(nw * 12.0)), land);
        float cl = smoothstep(0.55, 0.75, fbm(nw * 5.0 + 40.0));
        alb = mix(alb, vec3(0.9), cl);
        atm = vec3(0.3, 0.55, 1.0) * rim * (0.15 + ndl) * 1.5;
    } else if (type == 2) {
        float lat = dot(h.n, b.axis.xyz);
        float turb = fbm(nw * 4.0 + float(h.id));
        float bands = 0.5 + 0.5 * sin(lat * 22.0 + turb * 5.0);
        alb = mix(b.col1.rgb, b.col2.rgb, bands) * (0.85 + 0.3 * fbm(vec3(lat * 40.0, turb, 1.0)));
        atm = b.col1.rgb * rim * (0.1 + ndl) * 0.6;
    } else {
        float f = fbm(nw * 6.0 + float(h.id));
        alb = mix(b.col1.rgb, b.col2.rgb, f);
        vec3 H = normalize(L - viewDir);
        atm = vec3(0.6, 0.8, 1.0) * pow(max(dot(h.n, H), 0.0), 60.0) * ndl * 2.0;
    }
    return alb * ndl * SUN_POWER + alb * 0.004 + atm * SUN_POWER;
}

vec3 shade_ship(Hit h, vec3 viewDir, vec3 sunDir) {
    float k = ship_scale();
    if (h.kind == 3) {
        float r = length(h.pos.yz) / k;
        float a = atan(h.pos.z, h.pos.y);
        float pulse = 0.6 + 0.4 * sin(a * 12.0 - time * 3.0);
        vec3 glowc = mix(vec3(0.25, 0.7, 1.0), vec3(0.9, 0.4, 1.0), clamp(vNow / 5.0, 0.0, 1.0));
        float edge = smoothstep(0.86, 0.9, r) * smoothstep(0.98, 0.94, r);
        return vec3(0.08) + glowc * edge * pulse * (0.15 + min(vNow, 4.0));
    }
    vec3 n = h.n;
    vec3 lp = h.pos / k;
    float panels = step(0.92, fract(lp.x * 9.0)) + step(0.95, fract(atan(lp.z, lp.y) * 3.0));
    vec3 alb = vec3(0.55, 0.57, 0.6) * (1.0 - 0.35 * min(panels, 1.0));
    if (lp.x > 0.45 && n.y > 0.2) alb = vec3(0.05, 0.1, 0.15);         // canopy
    float ndl = max(dot(n, sunDir), 0.0);
    vec3 H = normalize(sunDir - viewDir);
    float spec = pow(max(dot(n, H), 0.0), 40.0);
    vec3 col = alb * (ndl * SUN_POWER + 0.03) + vec3(spec) * SUN_POWER * 0.6;
    if (lp.x < -0.55) col += vec3(0.3, 0.6, 1.0) * 3.0;                  // engine
    return col;
}

// glare around the star for rays that escape to the sky
vec3 sun_glare(vec3 o, vec3 d) {
    vec3 to = sun_pos() - o;
    float dist = length(to);
    float ang = acos(clamp(dot(d, to / dist), -1.0, 1.0));
    float a = body_at(0).posRad.w / dist;
    return body_at(0).col1.rgb * (0.6 * a * a / (ang * ang + a * a * 0.5) + 0.006 * a / (ang + 0.02));
}

#endif
