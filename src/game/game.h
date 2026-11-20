#ifndef GAME_H
#define GAME_H

#include <stdbool.h>

typedef struct Game Game;

Game* game_create(const char* shader_dir);
void  game_destroy(Game* g);
void  game_update(Game* g, float dt);
void  game_draw(Game* g);

// Renders a fixed set of reference views (no HUD) into out_dir and returns
// the number written. Used by `--shots`.
int   game_render_reference_shots(Game* g, const char* out_dir);

// Renders the whole timeline from one camera (0 ship, 1 departure, 2 arrival,
// 3 midway) as a PNG sequence, every dt time units. Used by `--frames`.
int   game_render_frames(Game* g, int cam, float dt, const char* out_dir);

#endif
