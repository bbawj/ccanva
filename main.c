#include "geometry.h"
#include "log.h"
#include <SDL2/SDL_keycode.h>
#include <stdint.h>
#define PERSPECTIVE_PROJ_MATRIX

#include "camera.h"
#include "cg.h"
#include "cow.h"
#include "geometry.h"
#include "light.h"
#include <SDL2/SDL.h>
#include <SDL2/SDL_events.h>
#include <SDL2/SDL_render.h>
#include <SDL2/SDL_surface.h>
#include <SDL2/SDL_timer.h>
#include <SDL2/SDL_video.h>
#include <assert.h>

#define WINDOW_HEIGHT 900
#define WINDOW_WIDTH 900

const uint32_t numTris = 3156;

int main(void) {
  Light LIGHT = {(Vec3f){0, 1, 0}, (Vec3f){1, 1, 1}, 1};

  Camera c;
  Vec3f camera_start = {0, 20, 80.400412};
  Vec3f lookat_point = {0, 0, 0};

  init_camera(&c, camera_start, lookat_point);

  float aperture_width = 25, aperture_height = 25, focal_len = 100;
  uint32_t image_width = WINDOW_WIDTH, image_height = WINDOW_HEIGHT;

  Matrix44f pers_proj = {0};
  setPerspectiveProj(&pers_proj, 25, 0.1, 100);

  uint32_t image_buffer[image_width * image_height];
  float z_buffer[image_width * image_height];

  if (SDL_Init(SDL_INIT_VIDEO) < 0)
    return 1;
  SDL_Window *window =
      SDL_CreateWindow("cg", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
                       WINDOW_HEIGHT, WINDOW_WIDTH, 0);
  assert(window);

  SDL_Renderer *renderer =
      SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
  assert(renderer);

  SDL_Texture *buf =
      SDL_CreateTexture(renderer, SDL_PIXELFORMAT_ABGR32,
                        SDL_TEXTUREACCESS_STREAMING, image_width, image_height);

  bool quit = false;
  float alpha = 0.f, beta = 0.f, gamma = 0.f;
  float vel = 0.5;
  Uint64 start = SDL_GetTicks64();

  while (!quit) {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
      Uint64 delta = SDL_GetTicks64() - start;
      switch (event.type) {
      case SDL_MOUSEBUTTONUP: {
        c.moveType = NONE;
      } break;
      case SDL_MOUSEBUTTONDOWN: {
        if (event.key.keysym.mod & KMOD_CTRL) {
          c.moveType = event.button.button == SDL_BUTTON_LEFT     ? TUMBLE
                       : event.button.button == SDL_BUTTON_MIDDLE ? TRACK
                       : event.button.button == SDL_BUTTON_RIGHT  ? DOLLY
                                                                  : NONE;
        }
      } break;
      case SDL_MOUSEMOTION: {
        camera_move(&c, delta, event.motion.xrel, event.motion.yrel);
      } break;
      case SDL_MOUSEWHEEL: {
        camera_move(&c, delta, event.wheel.x, event.wheel.preciseY);
      } break;
      case SDL_KEYDOWN: {
        switch (event.key.keysym.sym) {
        default:
          break;
        }
        break;
      }
      case SDL_QUIT:
        quit = true;
        break;
      default:
        break;
      }
    }

    memset(image_buffer, 0, sizeof(uint32_t) * image_width * image_height);
    for (int i = 0; i < image_height; ++i) {
      for (int j = 0; j < image_width; ++j) {
        z_buffer[i * image_width + j] = FLT_MAX;
      }
    }

    // Matrix44f rot = rotationMatrix(alpha, beta, gamma);
    Matrix44f worldToCamera = inverse(c.cam_to_world);
    Vec3f viewDir = normalize(vec_minus(c.pos, lookat_point));
    for (uint32_t i = 0; i < numTris; ++i) {
      const Vec3f v0World = vertices[nvertices[i * 3]];
      const Vec3f v1World = vertices[nvertices[i * 3 + 1]];
      const Vec3f v2World = vertices[nvertices[i * 3 + 2]];
      ccanva_render(image_buffer, z_buffer, viewDir, v0World, v1World, v2World,
                    worldToCamera, pers_proj, aperture_width, aperture_height,
                    focal_len, image_width, image_height, &LIGHT);
    }

    void *pixels;
    int pitch;
    int ret = SDL_LockTexture(buf, NULL, &pixels, &pitch);
    assert(ret == 0);

    for (int i = 0; i < image_height; ++i) {
      memcpy(pixels + i * pitch, image_buffer + i * image_width, pitch);
    }
    SDL_UnlockTexture(buf);
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0);
    SDL_RenderClear(renderer);
    SDL_RenderCopy(renderer, buf, NULL,
                   &(SDL_Rect){0, 0, image_width, image_height});

    SDL_RenderPresent(renderer);
    Uint64 end = SDL_GetTicks64();
    if (end - start < 33) {
      SDL_Delay(33 - (end - start));
    }
    start = end;
  }
  // SDL_FreeSurface(texture);
  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();

  return 0;
}
