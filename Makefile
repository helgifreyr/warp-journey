# Makefile for the Alcubierre warp-bubble raytracer (raylib; OpenGL 3.3 / WebGL2 fragment shader)
# Same layout as raylib_raytrace_blackhole: vendored raylib in include/ + lib/.

CC = gcc
CFLAGS = -std=c99 -Wall -Wextra -O2
INCLUDES = -Iinclude -I.
LDFLAGS = -Llib

ifeq ($(OS),Windows_NT)
    LIBS = -lraylib -lopengl32 -lgdi32 -lwinmm
    EXE = .exe
else
    UNAME_S := $(shell uname -s)
    ifeq ($(UNAME_S),Linux)
        LIBS = -lraylib -lGL -lm -lpthread -ldl -lrt -lX11
    endif
    ifeq ($(UNAME_S),Darwin)
        LIBS = -lraylib -framework OpenGL -framework Cocoa -framework IOKit -framework CoreVideo
    endif
    EXE =
endif

TARGET = warp$(EXE)
TEST   = tests/physics_test$(EXE)

SOURCES = main.c src/game/game.c src/gpu/warp_renderer.c
OBJECTS = $(SOURCES:.c=.o)
HEADERS = src/game/game.h src/gpu/warp_renderer.h
# the metric-agnostic geodesic code is the geodesic-gl library (git submodule)
GEODESIC_GL = external/geodesic-gl
SHADERS = $(wildcard resources/shaders/*.glsl) resources/shaders/warp.frag \
          $(GEODESIC_GL)/glsl/dual.glsl $(GEODESIC_GL)/glsl/geodesic.glsl $(GEODESIC_GL)/glsl/rk45.glsl
EMBED   = tools/embed_shaders$(EXE)

all: $(TARGET)

$(TARGET): $(OBJECTS)
	$(CC) $(OBJECTS) -o $@ $(LDFLAGS) $(LIBS)

%.o: %.c $(HEADERS)
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

# shaders are compiled into the exe, so warp.exe ships as a single file
$(EMBED): tools/embed_shaders.c
	$(CC) -O2 $< -o $@

src/gpu/shaders_embedded.h: $(SHADERS) $(EMBED)
	./$(EMBED) $@ $(SHADERS)

src/gpu/warp_renderer.o: src/gpu/shaders_embedded.h

# CPU double-precision check of the physics the shader implements
$(TEST): tests/physics_test.c $(GEODESIC_GL)/c/dual.c $(GEODESIC_GL)/c/dual.h
	$(CC) $(CFLAGS) -Wno-misleading-indentation tests/physics_test.c $(GEODESIC_GL)/c/dual.c -o $@ -lm

test: $(TEST)
	./$(TEST)

run: $(TARGET)
	./$(TARGET)

# render the reference image set to screenshots/reference/
shots: $(TARGET)
	./$(TARGET) --shots

# ---- web build (WebGL2, via Emscripten) -> docs/, served by GitHub Pages ---------
# Needs emcc on PATH (emsdk) and raylib 5.5 built for the web: `make web-deps` once.
WEB_RAYLIB = web_deps/raylib/src
WEB_OUT    = docs/index.html
WEB_FLAGS  = -std=gnu99 -O2 -Wall -DPLATFORM_WEB -DGRAPHICS_API_OPENGL_ES3 -I$(WEB_RAYLIB) -I.              -sUSE_GLFW=3 -sMIN_WEBGL_VERSION=2 -sMAX_WEBGL_VERSION=2 -sALLOW_MEMORY_GROWTH=1              --shell-file web/shell.html

web: $(WEB_OUT)

$(WEB_OUT): $(SOURCES) $(HEADERS) src/gpu/shaders_embedded.h web/shell.html $(WEB_RAYLIB)/libraylib.a
	@mkdir -p docs
	emcc $(WEB_FLAGS) $(SOURCES) $(WEB_RAYLIB)/libraylib.a -o $@

web-deps: $(WEB_RAYLIB)/libraylib.a

$(WEB_RAYLIB)/libraylib.a:
	git clone --depth 1 --branch 5.5 https://github.com/raysan5/raylib.git web_deps/raylib
	$(MAKE) -C $(WEB_RAYLIB) PLATFORM=PLATFORM_WEB GRAPHICS=GRAPHICS_API_OPENGL_ES3

# local test server (browsers won't load .wasm from file://)
serve: $(WEB_OUT)
	python -m http.server 8000 --directory docs

clean:
	rm -f $(OBJECTS) $(TARGET) $(TEST) $(EMBED) src/gpu/shaders_embedded.h

.PHONY: all test run shots clean web web-deps serve
