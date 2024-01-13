#define OPT_BACKFACE_CULLING
#define PERSPECTIVE_PROJ_MATRIX

#include "cg.h"
#include <SDL2/SDL.h>
#include <SDL2/SDL_events.h>
#include <SDL2/SDL_render.h>
#include <SDL2/SDL_surface.h>
#include <SDL2/SDL_timer.h>
#include <SDL2/SDL_video.h>
#include <assert.h>

const uint32_t numTris = 3156;

int main(void) {
  if (SDL_Init(SDL_INIT_VIDEO) < 0)
    return 1;
  SDL_Window *window = SDL_CreateWindow("cg", SDL_WINDOWPOS_UNDEFINED,
                                        SDL_WINDOWPOS_UNDEFINED, 1024, 512, 0);
  assert(window);

  SDL_Renderer *renderer =
      SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
  assert(renderer);

  Matrix44f worldToCamera = {.mat = {{0.707107, -0.331295, 0},
                                     {0, 0.883452, 0.468521, 0},
                                     {-0.707107, -0.331295, 0.624695, 0},
                                     {-1.63871, -5.747777, -40.400412, 1}}};
  // Matrix44f worldToCamera = inverse(cameraToWorld);

  float aperture_width = 25, aperture_height = 25, focal_len = 100;
  uint32_t image_width = 512, image_height = 512;

  Matrix44f pers_proj = {0};
  setPerspectiveProj(&pers_proj, 15, 0.1, 100);

  uint32_t image_buffer[image_width * image_height];
  float z_buffer[image_width * image_height];
  memset(image_buffer, 0, image_width * image_height);
  for (int i = 0; i < image_height; ++i) {
    for (int j = 0; j < image_width; ++j) {
      z_buffer[i * image_width + j] = FLT_MAX;
    }
  }
  uint32_t image_buffer2[image_width * image_height];
  float z_buffer2[image_width * image_height];
  memset(image_buffer2, 0, image_width * image_height);
  for (int i = 0; i < image_height; ++i) {
    for (int j = 0; j < image_width; ++j) {
      z_buffer2[i * image_width + j] = FLT_MAX;
    }
  }

  Uint64 start = SDL_GetTicks64();
  for (uint32_t i = 0; i < numTris; ++i) {
    const Vec3f v0World = vertices[nvertices[i * 3]];
    const Vec3f v1World = vertices[nvertices[i * 3 + 1]];
    const Vec3f v2World = vertices[nvertices[i * 3 + 2]];
    ccanva_render(image_buffer, z_buffer, v0World, v1World, v2World,
                  worldToCamera, pers_proj, aperture_width, aperture_height,
                  focal_len, image_width, image_height, false);
  }
  Uint64 end = SDL_GetTicks64();
  printf("Render time taken: %ld\n", end - start);

  SDL_Texture *buf = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_ABGR32,
                                       SDL_TEXTUREACCESS_STREAMING, 512, 512);
  void *pixels;
  int pitch;
  SDL_LockTexture(buf, NULL, &pixels, &pitch);
  for (int i = 0; i < image_height; ++i) {
    memcpy(pixels + i * pitch, image_buffer + i * image_width,
           image_width * sizeof(uint32_t));
  }
  SDL_UnlockTexture(buf);
  SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0);
  SDL_RenderClear(renderer);
  SDL_RenderCopy(renderer, buf, NULL, &(SDL_Rect){0, 0, 512, 512});

  SDL_Texture *buf2 = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_ABGR32,
                                        SDL_TEXTUREACCESS_STREAMING, 512, 512);
  void *pixels2;
  int pitch2;
  for (uint32_t i = 0; i < numTris; ++i) {
    const Vec3f v0World = vertices[nvertices[i * 3]];
    const Vec3f v1World = vertices[nvertices[i * 3 + 1]];
    const Vec3f v2World = vertices[nvertices[i * 3 + 2]];
    ccanva_render(image_buffer2, z_buffer2, v0World, v1World, v2World,
                  worldToCamera, pers_proj, aperture_width, aperture_height,
                  focal_len, image_width, image_height, true);
  }
  SDL_LockTexture(buf2, NULL, &pixels2, &pitch2);
  for (int i = 0; i < image_height; ++i) {
    memcpy(pixels2 + i * pitch2, image_buffer2 + i * image_width,
           image_width * sizeof(uint32_t));
  }
  SDL_UnlockTexture(buf2);
  SDL_RenderCopy(renderer, buf2, NULL, &(SDL_Rect){512, 0, 512, 512});
  SDL_RenderPresent(renderer);

  bool quit = false;
  start = SDL_GetTicks();
  while (!quit) {
    SDL_Event event;
    SDL_PollEvent(&event);
    switch (event.type) {
    case SDL_QUIT:
      quit = true;
      break;
    default:
      break;
    }
    Uint32 end = SDL_GetTicks();
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
