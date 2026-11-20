// Warp journey viewer: one scripted Alcubierre trip (rest -> accelerate ->
// cruise -> decelerate -> rest) on a timeline you can play and scrub, seen
// from a camera riding inside the bubble or from stationary observers.
// All relativity lives in the fragment shader; this file is the timeline,
// cameras, scene list, HUD and screenshot/frame export.
#include "game.h"
#include "../gpu/warp_renderer.h"
#include "raylib.h"
#include "raymath.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#if defined(PLATFORM_WEB)
#include <emscripten/emscripten.h>
#endif

// Units: c = 1. Distance and time share a unit ("light-units").
#define PATH_LEN   160.0f   // departure (x = 0) to arrival (x = PATH_LEN)
#define T_DEPART   0.0f     // the bubble starts moving (exactly at rest before)
#define T_ACCEL    4.0f     // duration of the speed-up, and of the slow-down
#define T_BEFORE   8.0f     // timeline starts this long before departure
#define T_AFTER    70.0f    // ... and ends this long after arrival

enum { CAM_SHIP = 0, CAM_DEPART = 1, CAM_ARRIVE = 2, CAM_CHASE = 3 };
static const char* CAM_NAMES[] = { "ship (inside the bubble)", "observer at departure",
                                   "observer at arrival, looking back along the route",
                                   "chase (outside the bubble)" };
// Observer presets. The arrival observer sits past the arrival point looking back
// toward the departure point, so the arrival and the whole route are in view.
static const Vector3 DEPART_OBS = { -7.0f, 2.5f, 10.0f };
static const Vector3 ARRIVE_OBS = { 160.0f + 12.0f, 3.0f, 7.0f };

typedef struct {
    const char* name;
    Vector3 pos;
    float radius;
    int type;              // 0 star, 1 rocky, 2 gas giant, 3 ice
    Vector3 c1, c2;
    float ringIn, ringOut;
    Vector3 axis;
} WorldBody;

static const float RENDER_SCALES[] = { 1.0f, 0.75f, 0.5f, 0.35f };

struct Game {
    WarpRenderer* rr;
    WorldBody bodies[WARP_MAX_BODIES];
    int numBodies;

    // journey
    float vMax, R, sigma;
    float t, playRate;
    bool playing;

    // cameras
    int cam;                          // CAM_*
    float shipYaw, shipPitch, shipDist;
    float chaseYaw, chasePitch, chaseDist;
    Vector3 obsPos;
    float obsYaw, obsPitch;
    float fov;

    // look
    bool doppler, glow, hud, help, mouseCaptured;
    float exposure;
    int scaleIdx;

    float clock, helpTimer, msgTimer;
    char msg[320];
    bool shotPending;
};

// ---- journey (must match metric_alcubierre.glsl) ---------------------------------
// arrival = the moment it stops. Route length = vMax (t_arrive - T_DEPART - T_ACCEL).
static float t_arrive(const Game* g) { return T_DEPART + PATH_LEN / g->vMax + T_ACCEL; }
static float t_min(void) { return T_DEPART - T_BEFORE; }
// Long enough for light from the departure to cross the route to the arrival observer.
static float t_max(const Game* g) {
    return fmaxf(t_arrive(g) + T_AFTER, T_DEPART + Vector3Length(ARRIVE_OBS) + 15.0f);
}

// smootherstep ramp and its integral: exactly as metric_alcubierre.glsl
static double ramp_S(double u) { return u * u * u * (10.0 + u * (-15.0 + 6.0 * u)); }
static double ramp_I(double u) { return u * u * u * u * (2.5 + u * (-3.0 + u)); }
static float traj_v(const Game* g, float t) {
    double tb = t_arrive(g), td = tb - T_ACCEL;
    if (t <= T_DEPART || t >= tb) return 0.0f;
    if (t < T_DEPART + T_ACCEL) return (float)(g->vMax * ramp_S((t - T_DEPART) / T_ACCEL));
    if (t <= td) return g->vMax;
    return (float)(g->vMax * (1.0 - ramp_S((t - td) / T_ACCEL)));
}
static float traj_x(const Game* g, float t) {
    double tb = t_arrive(g), td = tb - T_ACCEL, v = g->vMax;
    if (t <= T_DEPART) return 0.0f;
    if (t >= tb) return PATH_LEN;
    if (t < T_DEPART + T_ACCEL) return (float)(v * T_ACCEL * ramp_I((t - T_DEPART) / T_ACCEL));
    if (t <= td) return (float)(v * (t - T_DEPART - T_ACCEL) + 0.5 * v * T_ACCEL);
    double w = (t - td) / T_ACCEL;
    return (float)(v * T_ACCEL * (w - ramp_I(w)) + v * (td - T_DEPART - 0.5 * T_ACCEL));
}
static const char* phase_name(const Game* g, float t) {
    float tb = t_arrive(g);
    if (t <= T_DEPART) return "waiting at departure";
    if (t < T_DEPART + T_ACCEL) return "accelerating";
    if (t <= tb - T_ACCEL) return "cruising";
    if (t < tb) return "decelerating";
    return "arrived";
}

// ---- frames -----------------------------------------------------------------------------
static Vector3 dir_from_angles(float yaw, float pitch) {
    return (Vector3){ cosf(pitch) * cosf(yaw), sinf(pitch), cosf(pitch) * sinf(yaw) };
}
static void angles_from_dir(Vector3 d, float* yaw, float* pitch) {
    *pitch = asinf(Clamp(d.y, -1.0f, 1.0f));
    *yaw = atan2f(d.z, d.x);
}

static float ship_cam_max_dist(const Game* g) {   // stay inside the flat interior
    return fmaxf(0.9f, g->R - 2.5f / g->sigma - 0.05f);
}

// Chase camera: must stay outside the bubble wall (the metric there is not flat).
static float chase_min_dist(const Game* g) { return g->R + 3.0f / g->sigma + 0.6f; }

static void camera_frame(const Game* g, Vector3* pos, Vector3* fwd, Vector3* right, Vector3* up) {
    if (g->cam == CAM_CHASE) {
        // Orbits the ship's TRUE position. A stationary observer at that spot sees
        // whatever light has reached it, so above 1c the ship only shows from behind.
        Vector3 d = dir_from_angles(g->chaseYaw, g->chasePitch);
        Vector3 ship = { traj_x(g, g->t), 0, 0 };
        *pos = Vector3Add(ship, Vector3Scale(d, fmaxf(g->chaseDist, chase_min_dist(g))));
        *fwd = Vector3Negate(d);
    } else if (g->cam == CAM_SHIP) {
        Vector3 d = dir_from_angles(g->shipYaw, g->shipPitch);
        Vector3 ship = { traj_x(g, g->t), 0, 0 };
        *pos = Vector3Add(ship, Vector3Scale(d, fminf(g->shipDist, ship_cam_max_dist(g))));
        *fwd = Vector3Negate(d);
    } else {
        *pos = g->obsPos;
        *fwd = dir_from_angles(g->obsYaw, g->obsPitch);
    }
    *right = Vector3Normalize(Vector3CrossProduct(*fwd, (Vector3){ 0, 1, 0 }));
    *up = Vector3CrossProduct(*right, *fwd);
}

static void observer_preset(Game* g, int which) {
    Vector3 target;
    switch (which) {
    case CAM_DEPART: g->obsPos = DEPART_OBS; target = (Vector3){ 2.0f, 0, 0 }; break;
    default:         g->obsPos = ARRIVE_OBS; target = (Vector3){ PATH_LEN * 0.55f, 0, 0 }; break;
    }
    angles_from_dir(Vector3Normalize(Vector3Subtract(target, g->obsPos)), &g->obsYaw, &g->obsPitch);
    g->fov = 70.0f;
}

static void select_camera(Game* g, int which) {
    g->cam = which;
    if (which == CAM_SHIP) g->fov = 80.0f;
    else if (which == CAM_CHASE) g->fov = 70.0f;
    else observer_preset(g, which);
}

// ---- scene (journey frame: x = direction of travel) ---------------------------------
static void add_body(Game* g, WorldBody b) {
    if (g->numBodies < WARP_MAX_BODIES) {
        b.axis = Vector3Normalize(b.axis);
        g->bodies[g->numBodies++] = b;
    }
}

static void build_scene(Game* g) {
    g->numBodies = 0;
    add_body(g, (WorldBody){ "Sun", {120, -60, -900}, 60, 0, {1.0f, 0.82f, 0.55f}, {0, 0, 0}, 0, 0, {0, 1, 0} });
    add_body(g, (WorldBody){ "Thalassa", {-34, -30, -28}, 18, 1, {0.35f, 0.42f, 0.22f}, {0.03f, 0.10f, 0.25f}, 0, 0, {0.1f, 1, 0} });
    add_body(g, (WorldBody){ "Tamsin", {PATH_LEN + 70, -30, -75}, 45, 2, {0.80f, 0.62f, 0.42f}, {0.45f, 0.30f, 0.20f}, 62, 105, {0.25f, 1, 0.15f} });
    add_body(g, (WorldBody){ "Tamsin b", {PATH_LEN + 20, 25, -60}, 6, 3, {0.55f, 0.55f, 0.58f}, {0.30f, 0.29f, 0.30f}, 0, 0, {0, 1, 0} });
    add_body(g, (WorldBody){ "Rime", {-420, 220, 640}, 30, 3, {0.80f, 0.90f, 0.95f}, {0.45f, 0.65f, 0.80f}, 0, 0, {0, 1, 0.3f} });
    add_body(g, (WorldBody){ "Cinder", {60, 40, 260}, 9, 1, {0.65f, 0.30f, 0.15f}, {0.35f, 0.15f, 0.08f}, 0, 0, {0, 1, 0} });
    add_body(g, (WorldBody){ "Nereid", {950, -220, 850}, 70, 2, {0.30f, 0.50f, 0.80f}, {0.15f, 0.25f, 0.55f}, 95, 120, {-0.3f, 1, 0.2f} });
}

static void upload_scene(Game* g) {
    GpuBody gb[WARP_MAX_BODIES];
    for (int i = 0; i < g->numBodies; i++) {
        const WorldBody* b = &g->bodies[i];
        gb[i] = (GpuBody){
            { b->pos.x, b->pos.y, b->pos.z, b->radius },
            { b->c1.x, b->c1.y, b->c1.z, (float)b->type },
            { b->c2.x, b->c2.y, b->c2.z, b->ringIn },
            { b->axis.x, b->axis.y, b->axis.z, b->ringOut } };
    }
    warp_renderer_set_bodies(g->rr, gb, g->numBodies);
}

// ---- lifecycle ---------------------------------------------------------------------------
Game* game_create(const char* shader_dir) {
    Game* g = (Game*)calloc(1, sizeof *g);
    if (!g) return NULL;
    g->rr = warp_renderer_create(GetScreenWidth(), GetScreenHeight(), shader_dir);
    if (!g->rr) { free(g); return NULL; }
    build_scene(g);
    upload_scene(g);
    g->vMax = 4.0f;
    g->R = 2.2f;
    g->sigma = 6.0f;
    g->t = t_min();
    g->playRate = 4.0f;
    g->playing = true;
    g->shipYaw = -2.45f; g->shipPitch = 0.30f; g->shipDist = 1.6f;   // sunlit side
    g->chaseYaw = PI + 0.12f; g->chasePitch = 0.12f; g->chaseDist = 7.0f;   // just behind it
    select_camera(g, CAM_SHIP);
    g->doppler = true;
    g->hud = true;
    g->exposure = 1.0f;
    g->helpTimer = 12.0f;
    g->mouseCaptured = true;
#if defined(PLATFORM_WEB)
    g->scaleIdx = 2;   // 50%: browsers and laptop GPUs are slower, F2 to change
#endif
    DisableCursor();
    return g;
}

void game_destroy(Game* g) {
    if (!g) return;
    warp_renderer_destroy(g->rr);
    free(g);
}

static void toast(Game* g, const char* text) {
    snprintf(g->msg, sizeof g->msg, "%s", text);
    g->msgTimer = 3.0f;
}

// ---- timeline geometry (shared by HUD drawing and mouse seeking) -------------------------
static Rectangle timeline_rect(void) {
    int sw = GetScreenWidth(), sh = GetScreenHeight();
    return (Rectangle){ 30.0f, (float)sh - 46.0f, (float)sw - 60.0f, 14.0f };
}
static float timeline_x(const Game* g, float t) {
    Rectangle r = timeline_rect();
    return r.x + r.width * (t - t_min()) / (t_max(g) - t_min());
}

// ---- update ------------------------------------------------------------------------------------
void game_update(Game* g, float dt) {
    g->clock += dt;
    if (g->helpTimer > 0) g->helpTimer -= dt;
    if (g->msgTimer > 0) g->msgTimer -= dt;

    if (IsKeyPressed(KEY_TAB)) {
        g->mouseCaptured = !g->mouseCaptured;
        if (g->mouseCaptured) DisableCursor(); else EnableCursor();
    }

    // ---- timeline
    if (IsKeyPressed(KEY_SPACE)) g->playing = !g->playing;
    if (IsKeyPressed(KEY_UP))   g->playRate = fminf(40.0f, g->playRate * 1.5f);
    if (IsKeyPressed(KEY_DOWN)) g->playRate = fmaxf(0.1f, g->playRate / 1.5f);
    float scrub = (IsKeyDown(KEY_LEFT_SHIFT) ? 4.0f : 1.0f) * fmaxf(g->playRate, 2.0f) * 2.0f * dt;
    if (IsKeyDown(KEY_RIGHT)) g->t += scrub;
    if (IsKeyDown(KEY_LEFT))  g->t -= scrub;
    if (IsKeyPressed(KEY_HOME)) g->t = t_min();
    if (IsKeyPressed(KEY_END))  g->t = t_arrive(g) - T_ACCEL - 3.0f;
    if (g->playing) g->t += g->playRate * dt;
    Rectangle tr = timeline_rect();
    bool seeking = false;
    if (!g->mouseCaptured && IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
        Vector2 m = GetMousePosition();
        if (m.y > tr.y - 40 && m.y < tr.y + tr.height + 14) {
            g->t = t_min() + (m.x - tr.x) / tr.width * (t_max(g) - t_min());
            seeking = true;
        }
    }
    if (g->t > t_max(g)) { g->t = t_max(g); g->playing = false; }
    if (g->t < t_min()) g->t = t_min();

    // ---- cameras
    if (IsKeyPressed(KEY_ONE))   select_camera(g, CAM_SHIP);
    if (IsKeyPressed(KEY_TWO))   select_camera(g, CAM_DEPART);
    if (IsKeyPressed(KEY_THREE)) select_camera(g, CAM_ARRIVE);
    if (IsKeyPressed(KEY_FOUR))  select_camera(g, CAM_CHASE);

    Vector2 md = g->mouseCaptured ? GetMouseDelta() : (Vector2){ 0, 0 };
    if (!g->mouseCaptured && !seeking && IsMouseButtonDown(MOUSE_BUTTON_LEFT)) md = GetMouseDelta();
    const float sens = 0.0025f;
    float wheel = GetMouseWheelMove();
    if (g->cam == CAM_SHIP) {
        g->shipYaw += md.x * sens;
        g->shipPitch = Clamp(g->shipPitch + md.y * sens, -1.5f, 1.5f);
        g->shipDist = Clamp(g->shipDist * powf(0.9f, wheel), 0.9f, ship_cam_max_dist(g));
    } else if (g->cam == CAM_CHASE) {
        g->chaseYaw += md.x * sens;
        g->chasePitch = Clamp(g->chasePitch + md.y * sens, -1.5f, 1.5f);
        g->chaseDist = Clamp(g->chaseDist * powf(0.9f, wheel), chase_min_dist(g), 60.0f);
    } else {
        g->obsYaw += md.x * sens;
        g->obsPitch = Clamp(g->obsPitch - md.y * sens, -1.5f, 1.5f);
        g->fov = Clamp(g->fov - wheel * 5.0f, 20.0f, 130.0f);
        Vector3 f = dir_from_angles(g->obsYaw, g->obsPitch);
        Vector3 r = Vector3Normalize(Vector3CrossProduct(f, (Vector3){ 0, 1, 0 }));
        float sp = (IsKeyDown(KEY_LEFT_SHIFT) ? 40.0f : 8.0f) * dt;
        if (IsKeyDown(KEY_W)) g->obsPos = Vector3Add(g->obsPos, Vector3Scale(f, sp));
        if (IsKeyDown(KEY_S)) g->obsPos = Vector3Subtract(g->obsPos, Vector3Scale(f, sp));
        if (IsKeyDown(KEY_D)) g->obsPos = Vector3Add(g->obsPos, Vector3Scale(r, sp));
        if (IsKeyDown(KEY_A)) g->obsPos = Vector3Subtract(g->obsPos, Vector3Scale(r, sp));
        if (IsKeyDown(KEY_E)) g->obsPos.y += sp;
        if (IsKeyDown(KEY_Q)) g->obsPos.y -= sp;
    }

    // ---- journey + look
    if (IsKeyPressed(KEY_PAGE_UP) || IsKeyPressed(KEY_PAGE_DOWN)) {
        g->vMax = Clamp(g->vMax * (IsKeyPressed(KEY_PAGE_UP) ? 1.25f : 0.8f), 0.3f, 25.0f);
        char tt[96];
        snprintf(tt, sizeof tt, "Cruise speed %.2f c  (trip takes %.1f)", g->vMax, PATH_LEN / g->vMax);
        toast(g, tt);
    }
    if (IsKeyDown(KEY_RIGHT_BRACKET)) g->R = fminf(6.0f, g->R * (1.0f + dt));
    if (IsKeyDown(KEY_LEFT_BRACKET))  g->R = fmaxf(1.2f, g->R * (1.0f - dt));
    if (IsKeyDown(KEY_EQUAL)) g->sigma = fminf(30.0f, g->sigma * (1.0f + dt));
    if (IsKeyDown(KEY_MINUS)) g->sigma = fmaxf(1.5f, g->sigma * (1.0f - dt));
    if (IsKeyDown(KEY_PERIOD)) g->exposure = fminf(8.0f, g->exposure * (1.0f + dt));
    if (IsKeyDown(KEY_COMMA))  g->exposure = fmaxf(0.1f, g->exposure * (1.0f - dt));

    if (IsKeyPressed(KEY_C)) select_camera(g, g->cam == CAM_SHIP ? CAM_DEPART : CAM_SHIP);
    if (IsKeyPressed(KEY_B)) g->doppler = !g->doppler;
    if (IsKeyPressed(KEY_G)) g->glow = !g->glow;
    if (IsKeyPressed(KEY_H)) g->hud = !g->hud;
    // F1/F2 also have letter keys: Mac keyboards send F-keys as media keys
    if (IsKeyPressed(KEY_F1) || IsKeyPressed(KEY_SLASH)) { g->help = !g->help; g->helpTimer = 0; }
    if (IsKeyPressed(KEY_F2) || IsKeyPressed(KEY_R)) {
        g->scaleIdx = (g->scaleIdx + 1) % 4;
        char tt[64];
        snprintf(tt, sizeof tt, "Render scale %d%%", (int)(RENDER_SCALES[g->scaleIdx] * 100));
        toast(g, tt);
    }
#if defined(PLATFORM_WEB)
    // F5/F11/F12 belong to the browser (reload, fullscreen, dev tools)
    if (IsKeyPressed(KEY_P)) g->shotPending = true;
#else
    if (IsKeyPressed(KEY_F5)) toast(g, warp_renderer_reload(g->rr) ? "Shaders reloaded" : "Shader reload FAILED - see console, keeping old");
    if (IsKeyPressed(KEY_F11)) ToggleBorderlessWindowed();
    if (IsKeyPressed(KEY_F12) || IsKeyPressed(KEY_P)) g->shotPending = true;
#endif
}

// ---- render ---------------------------------------------------------------------------------------
static void render_view(Game* g, int w, int h) {
    warp_renderer_resize(g->rr, w, h);
    WarpFrame f = { 0 };
    camera_frame(g, &f.camPos, &f.camFwd, &f.camRight, &f.camUp);
    f.tNow = g->t;
    f.tanHalfFov = tanf(g->fov * 0.5f * DEG2RAD);
    f.aspect = (float)w / (float)h;
    f.bubX = (Vector3){ 1, 0, 0 };
    f.bubY = (Vector3){ 0, 1, 0 };
    f.bubZ = (Vector3){ 0, 0, 1 };
    f.R = g->R;
    f.sigma = g->sigma;
    f.vMax = g->vMax;
    f.tA = T_DEPART;
    f.tB = t_arrive(g);
    f.tauA = T_ACCEL;
    f.pathLen = PATH_LEN;
    f.tol = 1e-4f;
    f.maxSteps = 600;
    f.showShip = 1;
    f.doppler = g->doppler;
    f.glowGain = g->glow ? 0.6f : 0.0f;
    f.exposure = g->exposure;
    f.time = g->clock;
    warp_renderer_render(g->rr, &f);
}

static void save_screenshot(Game* g) {
    Image im = LoadImageFromTexture(warp_renderer_texture(g->rr));
    char name[256];
    time_t now = time(NULL);
#if defined(PLATFORM_WEB)
    // write into the in-memory file system, then hand it to the browser as a download
    strftime(name, sizeof name, "warp_%Y%m%d_%H%M%S.png", localtime(&now));
    bool ok = ExportImage(im, name);
    if (ok) EM_ASM({
        var n = UTF8ToString($0);
        var data = FS.readFile(n);
        var a = document.createElement('a');
        a.href = URL.createObjectURL(new Blob([data], { type: 'image/png' }));
        a.download = n;
        document.body.appendChild(a); a.click(); a.remove();
        FS.unlink(n);
    }, name);
#else
    strftime(name, sizeof name, "screenshots/warp_%Y%m%d_%H%M%S.png", localtime(&now));
    if (!DirectoryExists("screenshots")) MakeDirectory("screenshots");
    bool ok = ExportImage(im, name);
#endif
    UnloadImage(im);
    char m[300];
    snprintf(m, sizeof m, ok ? "Saved %s (no HUD)" : "Could not save %s", name);
    toast(g, m);
}

typedef struct { float t; const char* label; Color c; } TimelineEvent;

static void draw_timeline(Game* g) {
    Rectangle r = timeline_rect();
    float tb = t_arrive(g);
    DrawRectangle((int)r.x - 10, (int)r.y - 52, (int)r.width + 20, 80, Fade(BLACK, 0.55f));
    // speed profile above the bar, with the light-speed line
    float vTop = fmaxf(g->vMax, 1.0f);
    float plotH = 34.0f, base = r.y - 6.0f;
    float y1c = base - plotH / vTop;
    DrawLine((int)r.x, (int)y1c, (int)(r.x + r.width), (int)y1c, Fade((Color){ 255, 170, 120, 255 }, 0.5f));
    DrawText("c", (int)(r.x + r.width) - 10, (int)y1c - 11, 10, (Color){ 255, 170, 120, 220 });
    Vector2 prev = { 0, 0 };
    for (int i = 0; i <= 300; i++) {
        float t = t_min() + (t_max(g) - t_min()) * (float)i / 300.0f;
        float v = traj_v(g, t);
        Vector2 p = { timeline_x(g, t), base - plotH * v / vTop };
        if (i) DrawLineEx(prev, p, 2.0f, v > 1.0f ? (Color){ 255, 140, 230, 255 } : (Color){ 140, 210, 255, 255 });
        prev = p;
    }
    DrawRectangleRec(r, Fade(DARKGRAY, 0.8f));
    DrawRectangle((int)r.x, (int)r.y, (int)(timeline_x(g, g->t) - r.x), (int)r.height, Fade(SKYBLUE, 0.5f));

    TimelineEvent ev[4];
    int n = 0;
    ev[n++] = (TimelineEvent){ T_DEPART, "departs", WHITE };
    ev[n++] = (TimelineEvent){ tb, "arrives", WHITE };
    if (g->cam == CAM_DEPART || g->cam == CAM_ARRIVE) {   // when light from those events reaches this observer
        float dDep = Vector3Length(g->obsPos);
        float dArr = Vector3Distance(g->obsPos, (Vector3){ PATH_LEN, 0, 0 });
        ev[n++] = (TimelineEvent){ T_DEPART + dDep, "you see it leave", YELLOW };
        ev[n++] = (TimelineEvent){ tb + dArr, "you see it arrive", YELLOW };
    }
    for (int i = 0; i < n; i++) {
        int x = (int)timeline_x(g, ev[i].t);
        DrawLine(x, (int)r.y - 2, x, (int)(r.y + r.height + 2), ev[i].c);
        DrawText(ev[i].label, x + 3, (int)(r.y + r.height + 3), 10, ev[i].c);
    }
    int cx = (int)timeline_x(g, g->t);
    DrawRectangle(cx - 2, (int)r.y - 4, 4, (int)r.height + 8, RAYWHITE);
}

static void draw_hud(Game* g) {
    int sw = GetScreenWidth();
    float v = traj_v(g, g->t);
    DrawRectangle(10, 10, 520, 132, Fade(BLACK, 0.55f));
    Color vc = v >= 1.0f ? (Color){ 255, 140, 230, 255 } : (Color){ 140, 210, 255, 255 };
    DrawText(TextFormat("t = %6.1f    v = %.2f c", g->t, v), 22, 18, 28, vc);
    DrawText(TextFormat("%s%s   playback x%.1f%s", phase_name(g, g->t), v >= 1.0f ? " (faster than light)" : "",
             g->playRate, g->playing ? "" : "   [PAUSED]"), 22, 52, 14, RAYWHITE);
    DrawText(TextFormat("camera: %s", CAM_NAMES[g->cam]), 22, 72, 14, LIGHTGRAY);
    if (g->cam == CAM_CHASE) {
        Vector3 d = dir_from_angles(g->chaseYaw, g->chasePitch);
        float cone = v > 1.0f ? asinf(1.0f / v) * RAD2DEG : 180.0f;
        float fromTail = acosf(Clamp(-d.x, -1.0f, 1.0f)) * RAD2DEG;
        if (v > 1.0f && fromTail > cone)
            DrawText(TextFormat("faster than light: the ship can't be seen from here. Orbit to within %.0f deg of its tail", cone),
                     22, 90, 14, (Color){ 255, 170, 120, 255 });
        else
            DrawText("a stationary observer at this spot, moved along with the ship", 22, 90, 14, LIGHTGRAY);
    } else if (g->cam != CAM_SHIP) {
        float d = Vector3Distance(g->obsPos, (Vector3){ traj_x(g, g->t), 0, 0 });
        float tFirst = t_arrive(g) + Vector3Distance(g->obsPos, (Vector3){ PATH_LEN, 0, 0 });
        if (g->cam == CAM_ARRIVE && g->t < tFirst)
            DrawText(TextFormat("nothing to see yet: no light from the trip reaches you until t = %.0f", tFirst),
                     22, 90, 14, (Color){ 255, 220, 120, 255 });
        else
            DrawText(TextFormat("ship is really %.1f away: its light takes %.1f to get here", d, d), 22, 90, 14, LIGHTGRAY);
    } else {
        DrawText("inside the bubble: flat, free-falling, no g-forces", 22, 90, 14, LIGHTGRAY);
    }
    DrawText(TextFormat("cruise %.2f c   bubble R %.1f   wall sharpness %.1f   %d fps",
             g->vMax, g->R, g->sigma, GetFPS()), 22, 108, 14, GRAY);
    DrawText("F1 or ? help", 22, 126, 12, DARKGRAY);

    if (g->help || g->helpTimer > 0) {
        static const char* lines[] = {
            "SPACE        play / pause",
            "LEFT RIGHT   scrub (SHIFT faster)   UP DOWN  playback speed",
            "HOME / END   start / just before arrival",
            "TAB          free the mouse: click the timeline to seek",
            "1            ship camera (orbits inside the bubble)",
            "2 3          observers: at departure / at arrival looking back",
            "4            chase camera outside the bubble",
            "mouse        orbit the ship (1, 4) / look around (2, 3)",
            "WASD QE      fly the observer (SHIFT faster)",
            "wheel        orbit distance (1, 4) / observer zoom (2, 3)",
            "PGUP PGDN    cruise speed      [ ]  bubble size",
            "- =          wall sharpness    , .  exposure",
            "B / G        Doppler colour / exotic-matter glow",
#if defined(PLATFORM_WEB)
            "P            screenshot (no HUD), downloaded as a PNG",
            "H hide HUD   F2 / R render scale   F11 fullscreen",
#else
            "P / F12      screenshot (no HUD) -> screenshots/",
            "H hide HUD   F2 / R render scale   F5 shaders   F11 fullscreen",
#endif
        };
        int n = (int)(sizeof lines / sizeof lines[0]);
        int bw = 520, bh = n * 20 + 20, bx = sw - bw - 10, by = 10;
        DrawRectangle(bx, by, bw, bh, Fade(BLACK, 0.6f));
        for (int i = 0; i < n; i++) DrawText(lines[i], bx + 12, by + 10 + i * 20, 15, RAYWHITE);
    }
    draw_timeline(g);
    if (g->msgTimer > 0) {
        int w = MeasureText(g->msg, 18);
        DrawRectangle(sw / 2 - w / 2 - 10, 150, w + 20, 30, Fade(BLACK, 0.6f));
        DrawText(g->msg, sw / 2 - w / 2, 156, 18, RAYWHITE);
    }
}

void game_draw(Game* g) {
    int sw = GetScreenWidth(), sh = GetScreenHeight();
    float s = RENDER_SCALES[g->scaleIdx];
    render_view(g, (int)(sw * s), (int)(sh * s));
    if (g->shotPending) { save_screenshot(g); g->shotPending = false; }

    BeginDrawing();
    ClearBackground(BLACK);
    warp_renderer_draw(g->rr, (Rectangle){ 0, 0, (float)sw, (float)sh });
    if (g->hud) draw_hud(g);
    EndDrawing();
}

// ---- offline output ------------------------------------------------------------------------------
static bool export_view(Game* g, const char* path) {
    render_view(g, 1280, 720);
    Image im = LoadImageFromTexture(warp_renderer_texture(g->rr));
    bool ok = ExportImage(im, path);
    UnloadImage(im);
    BeginDrawing(); EndDrawing();   // keep the (hidden) window serviced
    return ok;
}

typedef struct {
    const char* name;
    int cam;
    float t;                 // offset from the anchor event
    int anchor;              // 0 departure, 1 arrival, 2/3 = when this observer SEES departure/arrival
    float yaw, pitch, dist;  // ship and chase cameras only
    bool glow;
} Shot;

int game_render_reference_shots(Game* g, const char* out_dir) {
    static const Shot shots[] = {
        // riding along: camera inside the bubble, behind / ahead / beside the ship
        { "01_ship_waiting",              CAM_SHIP,   -4.0f, 0, -2.45f, 0.30f, 1.6f, false },
        { "02_ship_accelerating",         CAM_SHIP,    2.0f, 0, -2.45f, 0.30f, 1.6f, true  },
        { "03_ship_cruise",               CAM_SHIP,   15.0f, 0, -2.45f, 0.30f, 1.6f, false },
        { "04_ship_cruise_looking_back",  CAM_SHIP,   15.0f, 0,  0.50f, 0.20f, 1.6f, false },
        { "05_ship_cruise_side",          CAM_SHIP,   15.0f, 0, -1.57f, 0.10f, 1.6f, false },
        { "06_ship_cruise_glow",          CAM_SHIP,   15.0f, 0, -2.45f, 0.30f, 1.6f, true  },
        { "07_ship_decelerating",         CAM_SHIP,   -2.0f, 1, -2.45f, 0.30f, 1.6f, true  },
        { "08_ship_arrived",              CAM_SHIP,    6.0f, 1, -2.45f, 0.30f, 1.6f, false },
        // departure observer, timed from when the departure's light reaches them
        { "10_depart_obs_waiting",        CAM_DEPART, -3.0f, 2, 0, 0, 0, false },
        { "11_depart_obs_bubble_forms",   CAM_DEPART,  2.5f, 2, 0, 0, 0, true  },
        { "12_depart_obs_leaving",        CAM_DEPART,  6.0f, 2, 0, 0, 0, false },
        { "13_depart_obs_receding",       CAM_DEPART, 12.0f, 2, 0, 0, 0, false },
        { "14_depart_obs_receding_glow",  CAM_DEPART, 12.0f, 2, 0, 0, 0, true  },
        // arrival observer, timed from when the arrival's light reaches them
        { "20_arrive_obs_already_here",   CAM_ARRIVE, -6.0f, 3, 0, 0, 0, false },
        { "21_arrive_obs_first_glimpse", CAM_ARRIVE, -0.6f, 3, 0, 0, 0, false },
        { "22_arrive_obs_appears",        CAM_ARRIVE,  0.6f, 3, 0, 0, 0, false },
        { "23_arrive_obs_trip_in_reverse",CAM_ARRIVE,  3.0f, 3, 0, 0, 0, false },
        { "24_arrive_obs_ghost_receding", CAM_ARRIVE, 10.0f, 3, 0, 0, 0, false },
        // beside the route: it has already passed by the time you see anything
        // chase camera outside the bubble (orbiting the true position)
        { "30_chase_waiting",             CAM_CHASE,  -4.0f, 0, PI + 0.9f, 0.25f, 7.0f, false },
        { "31_chase_cruise_behind",       CAM_CHASE,  15.0f, 0, PI + 0.12f, 0.12f, 7.0f, false },
        { "32_chase_cruise_side_invisible",        CAM_CHASE,  15.0f, 0, PI * 0.5f, 0.15f, 7.0f, false },
        { "33_chase_decelerating",        CAM_CHASE,  -2.0f, 1, PI + 0.12f, 0.12f, 7.0f, false },
    };
    int n = (int)(sizeof shots / sizeof shots[0]), written = 0;
    if (!DirectoryExists(out_dir)) MakeDirectory(out_dir);
    for (int i = 0; i < n; i++) {
        const Shot* s = &shots[i];
        select_camera(g, s->cam);
        if (s->cam == CAM_SHIP) { g->shipYaw = s->yaw; g->shipPitch = s->pitch; g->shipDist = s->dist; }
        if (s->cam == CAM_CHASE) { g->chaseYaw = s->yaw; g->chasePitch = s->pitch; g->chaseDist = s->dist; }
        float anchorT = T_DEPART;
        if (s->anchor == 1) anchorT = t_arrive(g);
        if (s->anchor == 2) anchorT = T_DEPART + Vector3Length(g->obsPos);
        if (s->anchor == 3) anchorT = t_arrive(g) + Vector3Distance(g->obsPos, (Vector3){ PATH_LEN, 0, 0 });
        g->t = anchorT + s->t;
        g->glow = s->glow;
        char path[512];
        snprintf(path, sizeof path, "%s/%s.png", out_dir, s->name);
        if (export_view(g, path)) written++;
    }
    return written;
}

int game_render_frames(Game* g, int cam, float dt, const char* out_dir) {
    select_camera(g, cam);
    if (!DirectoryExists(out_dir)) MakeDirectory(out_dir);
    int written = 0;
    for (float t = t_min(); t <= t_max(g); t += dt) {
        g->t = t;
        char path[512];
        snprintf(path, sizeof path, "%s/frame_%04d.png", out_dir, written);
        if (!export_view(g, path)) break;
        written++;
    }
    return written;
}
