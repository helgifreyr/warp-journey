// GPU harness: per-pixel raytracer as a fragment shader drawn over a render
// texture, through plain raylib. Derived from raylib_raytrace_blackhole's
// compute harness (same #include resolver). It was a compute shader until the
// web/macOS port: macOS stops at OpenGL 4.1 and WebGL2 has no compute shaders,
// while a fragment shader runs on OpenGL 3.3+, macOS and WebGL2 alike.
#include "warp_renderer.h"
#include "rlgl.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "shaders_embedded.h"

#if defined(PLATFORM_WEB) || defined(GRAPHICS_API_OPENGL_ES3)
static const char* GLSL_HEADER = "#version 300 es\nprecision highp float;\nprecision highp int;\n";
#else
static const char* GLSL_HEADER = "#version 330\n";
#endif

struct WarpRenderer {
    char shaderDir[256];
    Shader shader;
    RenderTexture2D target;
    GpuBody bodies[WARP_MAX_BODIES];
    int numBodies;
    int width, height;
};

// ---- #include resolver (from the black-hole harness) -------------------------
static char* read_file(const char* path, size_t* len) {
    FILE* f = fopen(path, "rb");
    if (!f) return NULL;
    fseek(f, 0, SEEK_END);
    long n = ftell(f);
    fseek(f, 0, SEEK_SET);
    char* s = (char*)malloc((size_t)n + 1);
    if (!s) { fclose(f); return NULL; }
    size_t got = fread(s, 1, (size_t)n, f);
    s[got] = '\0';
    fclose(f);
    if (len) *len = got;
    return s;
}

// Disk first (so F5 hot reload works while editing), then the copy compiled into the exe.
// The metric-agnostic geodesic code (dual.glsl, geodesic.glsl, rk45.glsl) comes from the
// geodesic-gl library in external/geodesic-gl/glsl/.
#define GEODESIC_GL_DIR "external/geodesic-gl/glsl/"
static bool g_usedEmbedded = false;
static char* read_shader(const char* dir, const char* name) {
    char path[512];
    snprintf(path, sizeof path, "%s%s", dir, name);
    char* s = read_file(path, NULL);
    if (s) return s;
    snprintf(path, sizeof path, "%s%s", GEODESIC_GL_DIR, name);
    s = read_file(path, NULL);
    if (s) return s;
    for (int i = 0; i < EMBEDDED_SHADER_COUNT; i++) {
        if (strcmp(EMBEDDED_SHADERS[i].name, name) == 0) {
            size_t n = strlen(EMBEDDED_SHADERS[i].src);
            char* c = (char*)malloc(n + 1);
            if (c) memcpy(c, EMBEDDED_SHADERS[i].src, n + 1);
            g_usedEmbedded = true;
            return c;
        }
    }
    TraceLog(LOG_ERROR, "WARP: shader %s not found on disk or embedded", name);
    return NULL;
}

typedef struct { char* buf; size_t len, cap; } StrBuf;
static void sb_put(StrBuf* b, const char* s, size_t n) {
    if (b->len + n + 1 > b->cap) {
        while (b->len + n + 1 > b->cap) b->cap = b->cap ? b->cap * 2 : 16384;
        b->buf = (char*)realloc(b->buf, b->cap);
    }
    memcpy(b->buf + b->len, s, n);
    b->len += n;
    b->buf[b->len] = '\0';
}

static bool resolve_includes(const char* dir, const char* src, StrBuf* out, int depth) {
    if (depth > 8) return false;
    const char* p = src;
    while (*p) {
        const char* eol = strchr(p, '\n');
        size_t n = eol ? (size_t)(eol - p + 1) : strlen(p);
        if (strncmp(p, "#include \"", 10) == 0) {
            const char* start = p + 10;
            const char* end = memchr(start, '"', n - 10);
            if (!end) return false;
            char name[256];
            snprintf(name, sizeof name, "%.*s", (int)(end - start), start);
            char* inc = read_shader(dir, name);
            if (!inc) return false;
            bool ok = resolve_includes(dir, inc, out, depth + 1);
            free(inc);
            if (!ok) return false;
            sb_put(out, "\n", 1);
        } else {
            sb_put(out, p, n);
        }
        p += n;
    }
    return true;
}

static bool build_shader(const char* dir, Shader* out) {
    g_usedEmbedded = false;
    char* raw = read_shader(dir, "warp.frag");
    if (!raw) return false;
    StrBuf sb = {0};
    sb_put(&sb, GLSL_HEADER, strlen(GLSL_HEADER));
    bool ok = resolve_includes(dir, raw, &sb, 0);
    free(raw);
    if (!ok) { free(sb.buf); return false; }
    if (g_usedEmbedded) TraceLog(LOG_INFO, "WARP: using shaders embedded in the executable");
    Shader sh = LoadShaderFromMemory(NULL, sb.buf);   // raylib's default vertex shader
    free(sb.buf);
    if (!IsShaderValid(sh) || sh.id == rlGetShaderIdDefault()) return false;
    *out = sh;
    return true;
}

static void make_target(WarpRenderer* r, int w, int h) {
    if (r->target.id) UnloadRenderTexture(r->target);
    r->target = LoadRenderTexture(w, h);
    SetTextureFilter(r->target.texture, TEXTURE_FILTER_BILINEAR);
    r->width = w;
    r->height = h;
}

WarpRenderer* warp_renderer_create(int width, int height, const char* shader_dir) {
    WarpRenderer* r = (WarpRenderer*)calloc(1, sizeof *r);
    if (!r) return NULL;
    snprintf(r->shaderDir, sizeof r->shaderDir, "%s", shader_dir ? shader_dir : "resources/shaders/");
    if (!build_shader(r->shaderDir, &r->shader)) { free(r); return NULL; }
    make_target(r, width, height);
    if (!r->target.id) { UnloadShader(r->shader); free(r); return NULL; }
    return r;
}

void warp_renderer_destroy(WarpRenderer* r) {
    if (!r) return;
    if (r->target.id) UnloadRenderTexture(r->target);
    UnloadShader(r->shader);
    free(r);
}

bool warp_renderer_reload(WarpRenderer* r) {
    Shader s;
    if (!build_shader(r->shaderDir, &s)) return false;
    UnloadShader(r->shader);
    r->shader = s;
    return true;
}

void warp_renderer_resize(WarpRenderer* r, int w, int h) {
    if (w < 16) w = 16;
    if (h < 16) h = 16;
    if (w != r->width || h != r->height) make_target(r, w, h);
}

void warp_renderer_set_bodies(WarpRenderer* r, const GpuBody* bodies, int count) {
    if (count > WARP_MAX_BODIES) count = WARP_MAX_BODIES;
    r->numBodies = count;
    memcpy(r->bodies, bodies, sizeof(GpuBody) * (size_t)count);
}

static void set_f(Shader s, const char* name, float v) { SetShaderValue(s, GetShaderLocation(s, name), &v, SHADER_UNIFORM_FLOAT); }
static void set_i(Shader s, const char* name, int v)   { SetShaderValue(s, GetShaderLocation(s, name), &v, SHADER_UNIFORM_INT); }
static void set_v3(Shader s, const char* name, Vector3 v) { SetShaderValue(s, GetShaderLocation(s, name), &v, SHADER_UNIFORM_VEC3); }

void warp_renderer_render(WarpRenderer* r, const WarpFrame* f) {
    Shader p = r->shader;
    float res[2] = { (float)r->width, (float)r->height };
    SetShaderValue(p, GetShaderLocation(p, "resolution"), res, SHADER_UNIFORM_VEC2);
    set_v3(p, "camPos", f->camPos);
    set_v3(p, "camFwd", f->camFwd);
    set_v3(p, "camRight", f->camRight);
    set_v3(p, "camUp", f->camUp);
    set_f(p, "tanHalfFov", f->tanHalfFov);
    set_f(p, "aspect", f->aspect);
    set_v3(p, "bubX", f->bubX);
    set_v3(p, "bubY", f->bubY);
    set_v3(p, "bubZ", f->bubZ);
    set_f(p, "tNow", f->tNow);
    set_f(p, "vMax", f->vMax);
    set_f(p, "tA", f->tA);
    set_f(p, "tB", f->tB);
    set_f(p, "tauA", f->tauA);
    set_f(p, "pathLen", f->pathLen);
    set_f(p, "Rb", f->R);
    set_i(p, "pocket", f->pocket);
    set_f(p, "pAlpha", f->pAlpha);
    set_f(p, "pR", f->pR);
    set_f(p, "pD", f->pD);
    set_f(p, "pN", f->pN);
    set_f(p, "pRcrit", f->pRcrit);
    set_f(p, "pBcrit", f->pBcrit);
    set_f(p, "sigma", f->sigma);
    set_f(p, "tol", f->tol);
    set_i(p, "maxSteps", f->maxSteps);
    set_i(p, "showShip", f->showShip);
    set_i(p, "doppler", f->doppler);
    set_f(p, "glowGain", f->glowGain);
    set_f(p, "exposure", f->exposure);
    set_f(p, "time", f->time);
    set_i(p, "numBodies", r->numBodies);
    if (r->numBodies > 0)
        SetShaderValueV(p, GetShaderLocation(p, "bodyData"), r->bodies, SHADER_UNIFORM_VEC4, 4 * r->numBodies);

    // One quad over the whole target; every pixel runs the raytracer once.
    BeginTextureMode(r->target);
    BeginShaderMode(p);
    DrawRectangle(0, 0, r->width, r->height, WHITE);
    EndShaderMode();   // output alpha is 1, so the default blend leaves it untouched
    EndTextureMode();
}

void warp_renderer_draw(WarpRenderer* r, Rectangle dst) {
    DrawTexturePro(r->target.texture, (Rectangle){ 0, 0, (float)r->width, (float)r->height },
                   dst, (Vector2){ 0, 0 }, 0.0f, WHITE);
}

Texture2D warp_renderer_texture(WarpRenderer* r) { return r->target.texture; }
