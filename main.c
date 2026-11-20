/*
 * Warp Bubble - fly a ship inside an Alcubierre warp bubble.
 *
 * Every pixel is a null geodesic traced through the Alcubierre metric on the
^ * GPU (RK45 + dual-number autodiff of the inverse metric), so the distortion,
 * horizons and Doppler colours are physically computed, not faked.
 *
 *   warp.exe                    play the journey
 *   warp.exe --shots            render reference images to screenshots/reference
 *   warp.exe --frames CAM [DT]  render the whole timeline as a PNG sequence from
 *                               camera CAM (1 ship, 2 departure, 3 arrival,
 *                               4 chase), one frame every DT time units (0.25)
 */
#include "raylib.h"
#include "src/game/game.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#if defined(PLATFORM_WEB)
#include <emscripten/emscripten.h>

static Game* g_game;
static void web_frame(void) {
    game_update(g_game, GetFrameTime());
    game_draw(g_game);
}
#endif

int main(int argc, char** argv) {
    bool shots = argc > 1 && strcmp(argv[1], "--shots") == 0;
    int frameCam = 0;
    float frameDt = 0.25f;
    if (argc > 2 && strcmp(argv[1], "--frames") == 0) {
        frameCam = atoi(argv[2]);
        if (argc > 3) frameDt = (float)atof(argv[3]);
        if (frameCam < 1 || frameCam > 4 || frameDt <= 0.0f) {
            printf("usage: warp --frames CAM [DT]   CAM 1 ship, 2 departure, 3 arrival, 4 chase\n");
            return 1;
        }
        shots = true;   // same hidden-window setup
    }

    SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_VSYNC_HINT | (shots ? FLAG_WINDOW_HIDDEN : 0));
    InitWindow(1280, 720, "Warp Journey - Alcubierre drive raytracer");
    SetExitKey(KEY_NULL);
#if !defined(PLATFORM_WEB)
    if (!shots) {   // halfway between 1280x720 and 85% of the monitor, centred
        int m = GetCurrentMonitor();
        int mw = GetMonitorWidth(m), mh = GetMonitorHeight(m);
        if (mw > 0 && mh > 0) {
            int w = (1280 + (int)(mw * 0.85f)) / 2, h = (720 + (int)(mh * 0.85f)) / 2;
            SetWindowSize(w, h);
            Vector2 mp = GetMonitorPosition(m);
            SetWindowPosition((int)mp.x + (mw - w) / 2, (int)mp.y + (mh - h) / 2);
        }
    }
#endif

    Game* game = game_create("resources/shaders/");
    if (!game) {
        TraceLog(LOG_ERROR, "Could not start the GPU raytracer (needs OpenGL 3.3 or WebGL2).");
#if defined(PLATFORM_WEB)
        EM_ASM({ fail('Could not start the GPU raytracer (shader error).<br>See the browser console for details.'); });
        return 1;
#endif
        while (!shots && !WindowShouldClose()) {
            BeginDrawing();
            ClearBackground(BLACK);
            DrawText("Could not start the GPU raytracer.", 40, 40, 24, RED);
            DrawText("Needs OpenGL 3.3 or newer.", 40, 80, 18, RAYWHITE);
            DrawText("See the console log for the shader error.", 40, 110, 18, RAYWHITE);
            EndDrawing();
        }
        CloseWindow();
        return 1;
    }

    if (frameCam) {
        char dir[128];
        snprintf(dir, sizeof dir, "screenshots/frames_cam%d", frameCam);
        int n = game_render_frames(game, frameCam - 1, frameDt, dir);
        printf("wrote %d frames to %s\n", n, dir);
        game_destroy(game);
        CloseWindow();
        return n > 0 ? 0 : 1;
    }
    if (shots) {
        int n = game_render_reference_shots(game, "screenshots/reference");
        printf("wrote %d reference images to screenshots/reference\n", n);
        game_destroy(game);
        CloseWindow();
        return n > 0 ? 0 : 1;
    }

#if defined(PLATFORM_WEB)
    // the browser owns the loop; the canvas follows the browser window size
    g_game = game;
    emscripten_set_main_loop(web_frame, 0, 1);
#else
    while (!WindowShouldClose()) {
        game_update(game, GetFrameTime());
        game_draw(game);
    }
#endif

    game_destroy(game);
    CloseWindow();
    return 0;
}
