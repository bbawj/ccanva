#include "cg.h"
#include <SDL2/SDL.h>
#include <SDL2/SDL_events.h>
#include <SDL2/SDL_render.h>
#include <SDL2/SDL_surface.h>
#include <SDL2/SDL_timer.h>
#include <SDL2/SDL_video.h>
#include <assert.h>

const Vec3f verts[146] = {
    {0, 39.034, 0},          {0.76212, 36.843, 0},   {3, 36.604, 0},
    {1, 35.604, 0},          {2.0162, 33.382, 0},    {0, 34.541, 0},
    {-2.0162, 33.382, 0},    {-1, 35.604, 0},        {-3, 36.604, 0},
    {-0.76212, 36.843, 0},   {-0.040181, 34.31, 0},  {3.2778, 30.464, 0},
    {-0.040181, 30.464, 0},  {-0.028749, 30.464, 0}, {3.2778, 30.464, 0},
    {1.2722, 29.197, 0},     {1.2722, 29.197, 0},    {-0.028703, 29.197, 0},
    {1.2722, 29.197, 0},     {5.2778, 25.398, 0},    {-0.02865, 25.398, 0},
    {1.2722, 29.197, 0},     {5.2778, 25.398, 0},    {3.3322, 24.099, 0},
    {-0.028683, 24.099, 0},  {7.1957, 20.299, 0},    {-0.02861, 20.299, 0},
    {5.2778, 19.065, 0},     {-0.028663, 18.984, 0}, {9.2778, 15.265, 0},
    {-0.028571, 15.185, 0},  {9.2778, 15.265, 0},    {7.3772, 13.999, 0},
    {-0.028625, 13.901, 0},  {9.2778, 15.265, 0},    {12.278, 8.9323, 0},
    {-0.028771, 8.9742, 0},  {12.278, 8.9323, 0},    {10.278, 7.6657, 0},
    {-0.028592, 7.6552, 0},  {15.278, 2.5994, 0},    {-0.028775, 2.6077, 0},
    {15.278, 2.5994, 0},     {13.278, 1.3329, 0},    {-0.028727, 1.2617, 0},
    {18.278, -3.7334, 0},    {18.278, -3.7334, 0},   {2.2722, -1.2003, 0},
    {-0.028727, -1.3098, 0}, {4.2722, -5, 0},        {4.2722, -5, 0},
    {-0.028727, -5, 0},      {-3.3582, 30.464, 0},   {-3.3582, 30.464, 0},
    {-1.3526, 29.197, 0},    {-1.3526, 29.197, 0},   {-1.3526, 29.197, 0},
    {-5.3582, 25.398, 0},    {-1.3526, 29.197, 0},   {-5.3582, 25.398, 0},
    {-3.4126, 24.099, 0},    {-7.276, 20.299, 0},    {-5.3582, 19.065, 0},
    {-9.3582, 15.265, 0},    {-9.3582, 15.265, 0},   {-7.4575, 13.999, 0},
    {-9.3582, 15.265, 0},    {-12.358, 8.9323, 0},   {-12.358, 8.9323, 0},
    {-10.358, 7.6657, 0},    {-15.358, 2.5994, 0},   {-15.358, 2.5994, 0},
    {-13.358, 1.3329, 0},    {-18.358, -3.7334, 0},  {-18.358, -3.7334, 0},
    {-2.3526, -1.2003, 0},   {-4.3526, -5, 0},       {-4.3526, -5, 0},
    {0, 34.31, 0.040181},    {0, 30.464, -3.2778},   {0, 30.464, 0.040181},
    {0, 30.464, 0.028749},   {0, 30.464, -3.2778},   {0, 29.197, -1.2722},
    {0, 29.197, -1.2722},    {0, 29.197, 0.028703},  {0, 29.197, -1.2722},
    {0, 25.398, -5.2778},    {0, 25.398, 0.02865},   {0, 29.197, -1.2722},
    {0, 25.398, -5.2778},    {0, 24.099, -3.3322},   {0, 24.099, 0.028683},
    {0, 20.299, -7.1957},    {0, 20.299, 0.02861},   {0, 19.065, -5.2778},
    {0, 18.984, 0.028663},   {0, 15.265, -9.2778},   {0, 15.185, 0.028571},
    {0, 15.265, -9.2778},    {0, 13.999, -7.3772},   {0, 13.901, 0.028625},
    {0, 15.265, -9.2778},    {0, 8.9323, -12.278},   {0, 8.9742, 0.028771},
    {0, 8.9323, -12.278},    {0, 7.6657, -10.278},   {0, 7.6552, 0.028592},
    {0, 2.5994, -15.278},    {0, 2.6077, 0.028775},  {0, 2.5994, -15.278},
    {0, 1.3329, -13.278},    {0, 1.2617, 0.028727},  {0, -3.7334, -18.278},
    {0, -3.7334, -18.278},   {0, -1.2003, -2.2722},  {0, -1.3098, 0.028727},
    {0, -5, -4.2722},        {0, -5, -4.2722},       {0, -5, 0.028727},
    {0, 30.464, 3.3582},     {0, 30.464, 3.3582},    {0, 29.197, 1.3526},
    {0, 29.197, 1.3526},     {0, 29.197, 1.3526},    {0, 25.398, 5.3582},
    {0, 29.197, 1.3526},     {0, 25.398, 5.3582},    {0, 24.099, 3.4126},
    {0, 20.299, 7.276},      {0, 19.065, 5.3582},    {0, 15.265, 9.3582},
    {0, 15.265, 9.3582},     {0, 13.999, 7.4575},    {0, 15.265, 9.3582},
    {0, 8.9323, 12.358},     {0, 8.9323, 12.358},    {0, 7.6657, 10.358},
    {0, 2.5994, 15.358},     {0, 2.5994, 15.358},    {0, 1.3329, 13.358},
    {0, -3.7334, 18.358},    {0, -3.7334, 18.358},   {0, -1.2003, 2.3526},
    {0, -5, 4.3526},         {0, -5, 4.3526}};

const uint32_t numTris = 3156;

//[comment]
// Triangle index array. A triangle has 3 vertices. Each successive group of 3
// integers in this array represent the positions of the vertices in the vertex
// array making up one triangle of that object. For example, the first 3
// integers from this array, 8/7/9 represent the positions of the vertices
// making up the the first triangle. You can access these vertices with the
// following code:
//
//     verts[8]; /* first vertex  */
//
//     verts[7]; /* second vertex */
//
//     verts[9]; /* third vertex  */
//
// 6/5/5 are the positions of the vertices in the vertex array making up the
// second triangle, and so on. To find the indices of the n-th triangle, use the
// following code:
//
//     tris[n * 3];     /* index of the first vertex in the verts array */
//
//     tris[n * 3 + 1]; /* index of the second vertexin the verts array */
//
//     tris[n * 3 + 2]; /* index of the third vertex in the verts array */
//[/comment]
const uint32_t tris[128 * 3] = {
    8,   7,   9,   6,   5,   7,   4,   3,   5,   2,   1,   3,   0,   9,   1,
    5,   3,   7,   7,   3,   9,   9,   3,   1,   10,  12,  11,  13,  15,  14,
    15,  13,  16,  13,  17,  16,  18,  20,  19,  17,  20,  21,  20,  23,  22,
    20,  24,  23,  23,  26,  25,  24,  26,  23,  26,  27,  25,  26,  28,  27,
    27,  30,  29,  28,  30,  27,  30,  32,  31,  30,  33,  32,  27,  30,  34,
    32,  36,  35,  33,  36,  32,  36,  38,  37,  36,  39,  38,  38,  41,  40,
    39,  41,  38,  41,  43,  42,  41,  44,  43,  44,  45,  43,  44,  47,  46,
    44,  48,  47,  48,  49,  47,  48,  51,  50,  10,  52,  12,  13,  53,  54,
    55,  17,  54,  13,  54,  17,  56,  57,  20,  17,  58,  20,  20,  59,  60,
    20,  60,  24,  60,  61,  26,  24,  60,  26,  26,  61,  62,  26,  62,  28,
    62,  63,  30,  28,  62,  30,  30,  64,  65,  30,  65,  33,  62,  66,  30,
    65,  67,  36,  33,  65,  36,  36,  68,  69,  36,  69,  39,  69,  70,  41,
    39,  69,  41,  41,  71,  72,  41,  72,  44,  44,  72,  73,  44,  74,  75,
    44,  75,  48,  48,  75,  76,  48,  77,  51,  78,  80,  79,  81,  83,  82,
    83,  81,  84,  81,  85,  84,  86,  88,  87,  85,  88,  89,  88,  91,  90,
    88,  92,  91,  91,  94,  93,  92,  94,  91,  94,  95,  93,  94,  96,  95,
    95,  98,  97,  96,  98,  95,  98,  100, 99,  98,  101, 100, 95,  98,  102,
    100, 104, 103, 101, 104, 100, 104, 106, 105, 104, 107, 106, 106, 109, 108,
    107, 109, 106, 109, 111, 110, 109, 112, 111, 112, 113, 111, 112, 115, 114,
    112, 116, 115, 116, 117, 115, 116, 119, 118, 78,  120, 80,  81,  121, 122,
    123, 85,  122, 81,  122, 85,  124, 125, 88,  85,  126, 88,  88,  127, 128,
    88,  128, 92,  128, 129, 94,  92,  128, 94,  94,  129, 130, 94,  130, 96,
    130, 131, 98,  96,  130, 98,  98,  132, 133, 98,  133, 101, 130, 134, 98,
    133, 135, 104, 101, 133, 104, 104, 136, 137, 104, 137, 107, 137, 138, 109,
    107, 137, 109, 109, 139, 140, 109, 140, 112, 112, 140, 141, 112, 142, 143,
    112, 143, 116, 116, 143, 144, 116, 145, 119};

int main(void) {
  Matrix44f worldToCamera = {.mat = {{0.707107, -0.331295, 0},
                                     {0, 0.883452, 0.468521, 0},
                                     {-0.707107, -0.331295, 0.624695, 0},
                                     {-1.63871, -5.747777, -40.400412, 1}}};
  // Matrix44f worldToCamera = inverse(cameraToWorld);

  float aperture_width = 25, aperture_height = 25, focal_len = 100;
  uint32_t image_width = 512, image_height = 512;

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

  for (uint32_t i = 0; i < numTris; ++i) {
    const Vec3f v0World = vertices[nvertices[i * 3]];
    const Vec3f v1World = vertices[nvertices[i * 3 + 1]];
    const Vec3f v2World = vertices[nvertices[i * 3 + 2]];
    ccanva_render(image_buffer, z_buffer, v0World, v1World, v2World,
                  worldToCamera, aperture_width, aperture_height, focal_len,
                  image_width, image_height, false);
  }

  if (SDL_Init(SDL_INIT_VIDEO) < 0)
    return 1;
  SDL_Window *window = SDL_CreateWindow("cg", SDL_WINDOWPOS_UNDEFINED,
                                        SDL_WINDOWPOS_UNDEFINED, 1024, 512, 0);
  assert(window);

  SDL_Renderer *renderer =
      SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
  assert(renderer);

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
                  worldToCamera, aperture_width, aperture_height, focal_len,
                  image_width, image_height, true);
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
  Uint32 start = SDL_GetTicks();
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
