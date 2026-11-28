# Warp Journey: an Alcubierre drive raytracer

One faster-than-light trip by Alcubierre warp drive, played on a timeline. The
ship waits, its bubble accelerates past light speed, cruises, slows down and
stops at the destination. You watch it from a camera riding inside the bubble,
or from stationary observers at the departure point, at the arrival point and
beside the route.

Every pixel is a light ray traced through the warp-drive spacetime on the GPU,
including the speed-up and slow-down. The distortion, the black regions, the
colour shifts and the delays are computed, not painted on. It's meant as visual
reference for a game that uses the Alcubierre drive for FTL.

**Play it in the browser: https://helgifreyr.github.io/warp-journey/** (Windows, macOS
or Linux, in a current Chrome, Edge, Firefox or Safari with WebGL2). Click the view
to capture the mouse. It starts at 50% render scale; F2 or R changes it. On a Mac keyboard, use ? for help
and R for render scale, since the F-keys control brightness and volume there.

```
git submodule update --init  # once after cloning: fetches the geodesic-gl library
make                         # build warp.exe
make run                     # play
make shots                   # render the labelled reference set to screenshots/reference/
./warp.exe --frames 3 0.25   # PNG sequence of the whole trip from camera 3 (for video)
make test                    # CPU check of the physics (should say "all checks passed")
make web                     # browser build into docs/ (needs emsdk; `make web-deps` once first)
make serve                   # try the browser build at http://localhost:8000
```

**Sharing:** the easiest way is the browser link above, which serves `docs/` through
GitHub Pages. After changing the code, run `make web` and commit `docs/`.
`warp.exe` is a single self-contained file for Windows (shaders and raylib built in,
only standard Windows 10/11 DLLs). If a `resources/shaders/` folder sits next to it,
that's used instead, so editing a shader and pressing F5 works while developing.

## Controls

| Key | |
|---|---|
| SPACE | play / pause |
| LEFT / RIGHT | scrub (SHIFT for faster) |
| UP / DOWN | playback speed |
| HOME / END | start / just before arrival |
| TAB | free the mouse, then click or drag on the timeline to seek |
| 1 | ship camera: mouse orbits the ship inside the bubble, wheel sets distance |
| 2 / 3 | observer at departure / observer past the arrival point, looking back along the route |
| 4 | chase camera outside the bubble: mouse orbits the ship, wheel sets distance, V toggles keep-it-framed |
| mouse, WASD, QE | observers (2, 3): look, fly, up/down (SHIFT faster), wheel zooms |
| PGUP / PGDN | cruise speed (the route length stays fixed, so the trip gets shorter) |
| [ ] and - = | bubble size and wall sharpness |
| B / G | Doppler colour / exotic-matter glow |
| K | Van Den Broeck pocket on / off (see below) |
| 9 / 0 | pocket smaller / bigger inside |
| , . | exposure |
| P or F12 | screenshot without the HUD, saved to `screenshots/` (browser: P, downloaded) |
| H | hide HUD |
| I | info box: explains the current camera and what you're seeing at this moment |
| F1 or ? | show / hide the controls (shown by default) |
| F2 or R | render scale |
| F5, F11 | reload shaders (not in the browser), fullscreen |

The timeline shows the speed profile, with the light-speed line marked. For an
observer, it also marks the yellow moments when the light from the departure and
arrival actually reaches them.

## What you'll see

**Riding along (camera 1).** The camera floats inside the bubble, which is the
only place a camera can travel along above light speed. In there, space is flat
and the ship is in free fall: no g-forces, even while "accelerating".
- As speed rises, the sky ahead bunches toward the nose and turns blue.
- Looking back during cruise, the view is black. Light from behind can't catch
  the bubble. Around the edge of the dark region, stars are smeared red.
- Looking sideways, the sky splits sharply into a blue half and a black half.

**Watching it leave (camera 2).** Nothing happens until the departure's light
reaches the observer (the yellow marker). Then the bubble visibly forms, stretches
and shrinks away into the distance. This observer is behind it, so the departing
bubble stays visible as a receding image long after it has really gone.

**Watching it arrive (camera 3).** The observer is just past the arrival point,
looking back down the route toward the departure point. This is the strange one:
- The ship arrives long before you see anything. It's already sitting there while
  the view is empty. The HUD says when the first light will get there.
- When the light does arrive, there's a brief blueshifted spark: light that piled up
  on the bubble's front wall during the trip is released when it stops.
- The ship appears in place, and at the same time a dark, redshifted "ghost" bubble
  peels away from it and drifts back along the route, leaving the ship behind. You are watching the trip in
  reverse, because light from the faster-than-light leg arrives after the ship
  and in reverse order.
- The ghost shrinks away toward the departure point, 160 units off. That reversed
  trip is how this observer sees it leave. The departure itself is
  only a few pixels at that distance (wheel zooms in), and its light takes 172
  time units to get here, so the timeline runs long enough to include it.

**Chasing it (camera 4).** The camera orbits the ship from outside the bubble. No
observer can really keep up outside the bubble, so each frame shows what a
stationary observer at that spot would see at that moment. That light left the
ship a little earlier, so the ship's image lags behind its true position.
- By default (V toggles it) the camera aims at the image rather than the true
  position. It backs off so the image stays at the wheel distance. Above light
  speed it stays inside the cone where the ship can be seen at all. Without it,
  the ship seems to slide backwards out of view as it speeds up.
- At rest and below light speed you see the ship and bubble from any side.
- Above light speed the ship is visible only from within a cone behind it
  (14° at 4c), because light can't get out ahead of the bubble. With framing off,
  the HUD warns when you've orbited outside the cone.
- The ship looks bigger in cruise than at rest, even at the same image distance,
  because the bubble wall acts as a magnifying lens.
- From behind it looks much closer and bigger than it really is, and reddened.
  You're seeing light that left it a moment ago, when it was nearer.

**The glow (G) is not light.** It shows where the exotic (negative) energy sits:
a band around the bubble's middle, perpendicular to travel. Its brightness is
normalised so it reads at any speed.

**Wall sharpness matters for the look.** Sharp walls (= key) give crisp lensing
rings. Soft walls (- key) give a heat-haze smear.

**Van Den Broeck pocket (K).** In Alcubierre's metric the bubble *is* the motion:
at rest there is nothing there. Van Den Broeck (Class. Quantum Grav. 16 (1999)
3973) added a factor B(r) to the space part of the metric:

    ds² = −dt² + B²(r) [ (dx − v f(r) dt)² + dy² + dz² ]

B = 1 + α inside radius R̃, falls to 1 across a shell of thickness Δ̃ (his eq. 12,
B = α(−(n−1)wⁿ + n wⁿ⁻¹) + 1 with w = (R̃ + Δ̃ − r)/Δ̃ and n = 80), and is 1
outside. The pocket sits in the flat middle of the Alcubierre bubble. Inside, space
is flat but 1 + α times bigger than it looks from outside, so a big ship fits in a
small bubble. That is his point: the bubble wall can be tiny and the negative
energy it needs drops enormously. B doesn't depend on speed, so the pocket exists,
and needs energy, while parked. Implementation notes:
- The metric, B(r) and the energy density (his eq. 11) are exactly his. The
  numbers are scaled to be seeable: he uses α = 10¹⁷ and R̃ = Δ̃ = 10⁻¹⁵ m. Here
  R̃ = Δ̃ = half the flat interior and α defaults to 4 (9 / 0 change it). α is
  raised automatically if the ship wouldn't fit.
- The ship is the same size as before, so in coordinates it is 1/(1 + α) as big.
  The ship camera's distance (wheel) is a measured distance.
- From outside, the pocket is a small bead that lenses like a very dense glass
  ball, with a tiny-looking ship inside. It is there before departure.
- From inside, light behaves as if inside glass with refractive index 1 + α.
  Only light within a cone gets in or out. Beyond that it
  is totally internally reflected: you see black, or the ship mirrored in the wall.
- With G on, the pocket wall glows purple (negative energy) with a thin orange
  layer on its inside (positive energy). That's the sign pattern he finds.

`screenshots/reference/` has 28 labelled stills covering all of this (40–45 are the
pocket). The observer
shots are timed relative to when each event's light reaches that observer.

### Simplifications

- How a bubble could be created or steered is an open physics problem. Here
  the bubble is exactly at rest until departure, ramps smoothly to cruise speed
  over 4 time units (no jerk at either end), and ramps down the same way into
  the arrival. "Departs" and "arrives" on the timeline are the moments it starts
  and stops moving.
- Observers are stationary. Flying one around with WASD teleports it between
  stationary observers rather than moving it at a real speed.
- Light from the sun to the planets ignores the bubble.
- The Doppler brightness change is toned down, because the true values turn the
  screen white or black.

## How it works (for programmers)

The light-ray physics that doesn't depend on the spacetime (autodiff, Hamiltonian,
camera rays, RK45) is the [geodesic-gl](https://github.com/helgifreyr/geodesic-gl)
library, a git submodule in `external/geodesic-gl`. Its method (Hamiltonian
geodesics with the metric differentiated by dual-number autodiff) comes from
FANTASY: P. Christian and C.-k. Chan, ApJ 909, 67 (2021),
[arXiv:2010.02237](https://arxiv.org/abs/2010.02237). This project adds the metric,
the scene and the app around it. The app harness (Makefile layout, vendored raylib,
`#include` resolver for GLSL) comes from an earlier black-hole raytracer.

| File | Role |
|---|---|
| `resources/shaders/metric_alcubierre.glsl` | **the only metric-specific code**: inverse metric g^μν, with the trajectory X(t), v(t) written as Dual functions, so ∂H/∂t through the acceleration is exact |
| `external/geodesic-gl/glsl/` | the library: `dual.glsl` autodiff; `geodesic.glsl` Hamiltonian H = ½g^μν p_μ p_ν, autodiff ∂H/∂q, Eulerian-observer camera rays from a general 3+1 split; `rk45.glsl` Dormand–Prince RK45. The renderer looks there for any `#include` not in `resources/shaders/` |
| `resources/shaders/scene.glsl` | procedural sky, planets, rings, ship (art only) |
| `resources/shaders/warp.frag` | per-pixel driver: integrate while inside the capsule the bubble ever occupies, test hits per step, Doppler, tonemap |
| `src/gpu/warp_renderer.c` | draws one fullscreen quad with that fragment shader into a render texture; shaders loaded from disk or embedded; `#version 330` on desktop, `#version 300 es` in the browser |
| `web/shell.html` | the browser page around the WebGL2 canvas |
| `src/game/game.c` | timeline, cameras, scene, HUD, `--shots`, `--frames` |
| `tests/physics_test.c` | double-precision CPU reference (using the library's `c/dual.h`) with 14 pass/fail checks (5 for the pocket: B's shape, eq. 11 against the curvature, autodiff, light delay through the pocket = extra proper diameter, total internal reflection) |

Differences from the earlier black-hole raytracer, all found in feasibility testing:

1. **RK45 instead of the Tao symplectic integrator.** Tao's doubled phase space
   breaks down once the shift is superluminal.
2. **The momentum is rescaled every step.** Light at a warp horizon blueshifts by
   hundreds of e-folds. The accumulated log-scale gives the Doppler factor, and
   |log| > 12 is treated as a horizon.
3. **A fragment shader, not a compute shader.** It started as a GL 4.3 compute
   shader like the black-hole raytracer. macOS stops at OpenGL 4.1 and WebGL2
   has no compute shaders, so it now draws one fullscreen quad instead. The
   planets moved from a storage buffer to a uniform array. Output matches the
   compute version to within float rounding (12 of the 19 reference shots are
   bit-identical; the rest differ in under 0.3% of pixels).

Performance: a few ms per 1280×720 frame on an RTX 5070 Ti. On slower GPUs,
F2 lowers the render scale.

**Web build:** Emscripten, raylib 5.5 compiled for WebGL2 (`GRAPHICS_API_OPENGL_ES3`),
fetched and built by `make web-deps`. The same C and GLSL sources serve both builds.
A native macOS build should also work (`make` with a raylib built for OpenGL 3.3),
but hasn't been tried.

## License

MIT, see [LICENSE](LICENSE). The geodesic-gl submodule is MIT as well.
