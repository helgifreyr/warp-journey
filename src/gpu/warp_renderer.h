#ifndef WARP_RENDERER_H
#define WARP_RENDERER_H

#include "raylib.h"
#include <stdbool.h>

#define WARP_MAX_BODIES 32

// Mirrors `struct Body` in resources/shaders/scene.glsl (4 x vec4 in the bodyData uniform array).
typedef struct {
    float posRad[4];   // bubble-frame centre, radius
    float col1[4];     // rgb, type (0 star, 1 rocky, 2 gas giant, 3 ice)
    float col2[4];     // rgb, ring inner radius (0 = none)
    float axis[4];     // bubble-frame axis, ring outer radius
} GpuBody;

typedef struct {
    Vector3 camPos, camFwd, camRight, camUp;  // journey frame (x = direction of travel)
    float tNow;                               // coordinate time of the camera event
    float tanHalfFov, aspect;
    Vector3 bubX, bubY, bubZ;                 // journey axes in world space
    float R, sigma;
    float vMax, tA, tB, tauA, pathLen;        // journey: see metric_alcubierre.glsl
    int pocket;                               // Van Den Broeck pocket on/off
    float pAlpha, pR, pD, pN;                 // its alpha, R~, Delta~, n
    float pRcrit, pBcrit;                     // where B r is smallest across its wall, and that value
    float tol;
    int maxSteps;
    int showShip, doppler;
    float glowGain, exposure, time;
} WarpFrame;

typedef struct WarpRenderer WarpRenderer;

WarpRenderer* warp_renderer_create(int width, int height, const char* shader_dir);
void          warp_renderer_destroy(WarpRenderer* r);
bool          warp_renderer_reload(WarpRenderer* r);          // hot-reload shaders; keeps old on failure
void          warp_renderer_resize(WarpRenderer* r, int width, int height);
void          warp_renderer_set_bodies(WarpRenderer* r, const GpuBody* bodies, int count);
void          warp_renderer_render(WarpRenderer* r, const WarpFrame* f);
void          warp_renderer_draw(WarpRenderer* r, Rectangle dst);
Texture2D     warp_renderer_texture(WarpRenderer* r);

#endif
