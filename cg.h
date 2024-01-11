#ifndef CG_H_
#define CG_H_

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define AA 2

#ifdef OPT_BACKFACE_CULLING
#ifndef BACKFACE_CULLING_THRESHOLD
#define BACKFACE_CULLING_THRESHOLD M_PI
#endif // BACKFACE_CULLING_THRESHOLD
#endif // OPT_BACKFACE_CULLING

typedef struct {
  int x, y;
} Vec2i;

typedef struct {
  float x, y;
} Vec2f;

typedef struct {
  float x, y, z;
} Vec3f;

#include "cow.h"

typedef struct {
  float mat[4][4];
} Matrix44f;

Matrix44f identity();
Vec3f multVecMatrix(Vec3f v, Matrix44f m);
Matrix44f inverse(Matrix44f m);
bool worldToRaster(Vec3f *raster, const Vec3f pWorld,
                   const Matrix44f worldToCamera, const float aperture_width,
                   const float aperture_height, const float focal_len,
                   const float near_clipping_plane, const uint32_t image_width,
                   const uint32_t image_height);

Matrix44f identity() {
  Matrix44f identity = {.mat = {
                            {1, 0, 0, 0},
                            {0, 1, 0, 0},
                            {0, 0, 1, 0},
                            {0, 0, 0, 1},
                        }};
  return identity;
}

Vec3f multVecMatrix(Vec3f v, Matrix44f m) {
  Vec3f res;
  res.x =
      v.x * m.mat[0][0] + v.y * m.mat[1][0] + v.z * m.mat[2][0] + m.mat[3][0];
  res.y =
      v.x * m.mat[0][1] + v.y * m.mat[1][1] + v.z * m.mat[2][1] + m.mat[3][1];
  res.z =
      v.x * m.mat[0][2] + v.y * m.mat[1][2] + v.z * m.mat[2][2] + m.mat[3][2];
  return res;
}

float dot(Vec3f a, Vec3f b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

Vec3f crossProduct(Vec3f a, Vec3f b) {
  return (Vec3f){
      .x = a.y * b.z - a.z * b.y,
      .y = a.x * b.z - a.z * b.x,
      .z = a.x * b.y - b.y * a.x,
  };
}

Vec3f minus(Vec3f a, Vec3f b) {
  return (Vec3f){
      .x = a.x - b.x,
      .y = a.y - b.y,
      .z = a.z - b.z,
  };
}

Vec3f normalize(Vec3f a) {
  float magnitude = sqrtf(dot(a, a));
  float inv_mag = 1 / magnitude;
  return (Vec3f){a.x * inv_mag, a.y * inv_mag, a.z * inv_mag};
}

// Signed area of the parallelogram defined by vectors (c-a) and (b-a)
// which can be defined as the cross product of them
float edgeFunction(const Vec3f a, const Vec3f b, const Vec3f c) {
  return (c.x - a.x) * (b.y - a.y) - (c.y - a.y) * (b.x - a.x);
}

Vec3f barycentric(Vec3f v0, Vec3f v1, Vec3f v2, Vec3f p) {
  float area = edgeFunction(v0, v1, v2);
  float a0 = edgeFunction(v1, v2, p);
  float a1 = edgeFunction(v2, v0, p);
  float a2 = edgeFunction(v0, v1, p);

  Vec3f res = (Vec3f){0, 0, 0};

  // p is within the triangle
  if (a0 >= 0 && a1 >= 0 && a2 >= 0) {
    res.x = a0 / area;
    res.y = a1 / area;
    res.z = a2 / area;
  }
  return res;
}

#define RGBA(r, g, b, a)                                                       \
  (((r)&0xFF) << 24) | (((g)&0xFF) << 16) | (((b)&0xFF) << 8) |                \
      (((a)&0xFF) << 0)

// Gauss-Jordan inverse
Matrix44f inverse(Matrix44f m) {
  Matrix44f inv = identity();

  for (int col = 0; col < 3; ++col) {
    // Step 1: Choose a pivot > 0
    int pivot = col;
    float pivot_size = fabs(m.mat[pivot][pivot]);

    for (int row = col + 1; row < 4; ++row) {
      float temp = fabs(m.mat[row][col]);
      if (temp > pivot_size) {
        pivot = row;
        pivot_size = temp;
      }
    }

    if (pivot_size == 0) {
      printf("Singular matrix\n");
      return identity();
    }
    // if pivot changed, swap the rows
    if (pivot != col) {
      for (int i = 0; i < 4; ++i) {
        float temp = m.mat[col][i];
        m.mat[col][i] = m.mat[pivot][i];
        m.mat[pivot][i] = temp;

        temp = inv.mat[col][i];
        inv.mat[col][i] = inv.mat[pivot][i];
        inv.mat[pivot][i] = temp;
      }
    }
    // Step 2: remove bottom half of diagonal
    for (int row = col + 1; row < 4; ++row) {
      float constant = -m.mat[row][col] / m.mat[col][col];
      for (int i = 0; i < 4; ++i) {
        m.mat[row][i] += constant * m.mat[col][i];
        inv.mat[row][i] += constant * inv.mat[col][i];
      }
      m.mat[row][col] = 0.f;
    }
  }

  // Step 3: make pivots == 1
  for (int row = 0; row < 4; ++row) {
    float pivot = m.mat[row][row];
    for (int col = 0; col < 4; ++col) {
      m.mat[row][col] /= pivot;
      inv.mat[row][col] /= pivot;
    }
    // set the diagonal to 1.0 exactly to avoid
    // possible round-off error
    m.mat[row][row] = 1.f;
  }

  // Step 4: remove top half of diagonal
  for (int row = 0; row < 4; ++row) {
    for (int col = row + 1; col < 4; ++col) {
      float cur = m.mat[row][col];
      for (int i = 0; i < 4; ++i) {
        m.mat[row][i] -= cur * m.mat[col][i];
        inv.mat[row][i] -= cur * inv.mat[col][i];
      }
      m.mat[row][col] = 0.f;
    }
  }

  return inv;
}

bool worldToRaster(Vec3f *raster, const Vec3f pWorld,
                   const Matrix44f worldToCamera, const float aperture_width,
                   const float aperture_height, const float focal_len,
                   const float near_clipping_plane, const uint32_t image_width,
                   const uint32_t image_height) {
  Vec3f camera = multVecMatrix(pWorld, worldToCamera);
  Vec2f screen;
  screen.x = camera.x / -camera.z * near_clipping_plane;
  screen.y = camera.y / -camera.z * near_clipping_plane;
  Vec2f ndc;
  float top = ((aperture_height / 2) / focal_len) * near_clipping_plane;
  float bottom = -top;
  float right = top * (aperture_width / aperture_height);
  float left = -right;

  ndc.x = 2 * screen.x / (right - left) - (right + left) / (right - left);
  ndc.y = 2 * screen.y / (top - bottom) - (top + bottom) / (top - bottom);

  raster->x = (ndc.x + 1) / 2 * image_width;
  raster->y = (1 - ndc.y) / 2 * image_height;
  raster->z = -camera.z;

  if (raster->y > top || raster->y < bottom || raster->x > right ||
      raster->x < left)
    return false;
  return true;
}

void ccanva_render(uint32_t *image_buffer, float *z_buffer, const Vec3f v0World,
                   const Vec3f v1World, const Vec3f v2World,
                   const Matrix44f worldToCamera, const float aperture_width,
                   const float aperture_height, const float focal_len,
                   const uint32_t image_width, const uint32_t image_height,
                   bool perspective_correct) {
  Vec3f v0Raster, v1Raster, v2Raster;
  float near = 1;
  worldToRaster(&v0Raster, v0World, worldToCamera, aperture_width,
                aperture_height, focal_len, near, image_width, image_height);
  worldToRaster(&v1Raster, v1World, worldToCamera, aperture_width,
                aperture_height, focal_len, near, image_width, image_height);
  worldToRaster(&v2Raster, v2World, worldToCamera, aperture_width,
                aperture_height, focal_len, near, image_width, image_height);
  Vec3f normal = normalize(
      crossProduct(minus(v1Raster, v0Raster), minus(v2Raster, v1Raster)));
  // Calculate the bounding box of the triangle
  float max_x =
      fmin(fmax(fmax(v0Raster.x, v1Raster.x), v2Raster.x), image_width);
  float max_y =
      fmin(fmax(fmax(v0Raster.y, v1Raster.y), v2Raster.y), image_height);
  float min_x = fmax(fmin(fmin(v0Raster.x, v1Raster.x), v2Raster.x), 0);
  float min_y = fmax(fmin(fmin(v0Raster.y, v1Raster.y), v2Raster.y), 0);

  if (perspective_correct) {
    v0Raster.z = 1 / v0Raster.z;
    v1Raster.z = 1 / v1Raster.z;
    v2Raster.z = 1 / v2Raster.z;
  }

  // Pre calculate the step value as we iterate through the image buffer
  Vec2f w0_step = {.x = (v2Raster.y - v1Raster.y),
                   .y = -(v2Raster.x - v1Raster.x)};
  Vec2f w1_step = {.x = (v0Raster.y - v2Raster.y),
                   .y = -(v0Raster.x - v2Raster.x)};
  Vec2f w2_step = {.x = (v1Raster.y - v0Raster.y),
                   .y = -(v1Raster.x - v0Raster.x)};
  Vec2f w0_step_aa = {w0_step.x / AA, w0_step.y / AA};
  Vec2f w1_step_aa = {w1_step.x / AA, w1_step.y / AA};
  Vec2f w2_step_aa = {w2_step.x / AA, w2_step.y / AA};

  // Pre calculate the initial barycentric weights at the start of the bounding
  // box
  float w0_old = edgeFunction(v1Raster, v2Raster, (Vec3f){min_x, min_y, 0});
  float w1_old = edgeFunction(v2Raster, v0Raster, (Vec3f){min_x, min_y, 0});
  float w2_old = edgeFunction(v0Raster, v1Raster, (Vec3f){min_x, min_y, 0});
  float area = edgeFunction(v0Raster, v1Raster, v2Raster);
  for (uint32_t j = min_y; j < max_y;
       ++j, w0_old += w0_step.y, w1_old += w1_step.y, w2_old += w2_step.y) {
    float w0_temp = w0_old;
    float w1_temp = w1_old;
    float w2_temp = w2_old;

    for (uint32_t k = min_x; k < max_x; ++k, w0_temp += w0_step.x,
                  w1_temp += w1_step.x, w2_temp += w2_step.x) {
      int visible_subpixels = 0;
      float red = 0.f;
      float green = 0.f;
      float blue = 0.f;
      float w0_aa = w0_temp;
      float w1_aa = w1_temp;
      float w2_aa = w2_temp;

#ifdef OPT_BACKFACE_CULLING
      Vec3f view_direction = {-k, -j, 0};
      if (acosf(dot(normalize(view_direction), normal)) >
          BACKFACE_CULLING_THRESHOLD) {
        continue;
      }
#endif
      for (int q = 0; q < AA; ++q, w0_aa += w0_step_aa.y, w1_aa += w1_step_aa.y,
               w2_aa += w2_step_aa.y) {
        float w0_aa_temp = w0_aa;
        float w1_aa_temp = w1_aa;
        float w2_aa_temp = w2_aa;

        for (int r = 0; r < AA; ++r, w0_aa_temp += w0_step_aa.x,
                 w1_aa_temp += w1_step_aa.x, w2_aa_temp += w2_step_aa.x) {
          // Point is outside the triangle
          if (w0_aa_temp < 0 || w1_aa_temp < 0 || w2_aa_temp < 0) {
            continue;
          }

          float w0 = w0_aa_temp / area;
          float w1 = w1_aa_temp / area;
          float w2 = w2_aa_temp / area;
          float depth;
          if (perspective_correct) {
            depth = 1 / (w0 * v0Raster.z + w1 * v1Raster.z + w2 * v2Raster.z);
          } else {
            depth = 1 / (w0 * (1 / v0Raster.z) + w1 * (1 / v1Raster.z) +
                         w2 * (1 / v2Raster.z));
          }
          // Point is behind previously computed point
          if (z_buffer[j * image_width + k] < depth) {
            continue;
          }
          z_buffer[j * image_width + k] = depth;

          if (perspective_correct) {
            w0 *= depth;
            w1 *= depth;
            w2 *= depth;
            red += v0Raster.z * w0;
            green += v1Raster.z * w1;
            blue += v2Raster.z * w2;
          } else {
            red += w0;
            green += w1;
            blue += w2;
          }
          ++visible_subpixels;
        }
      }
      // Only render the pixel if there is something to render
      if (red > 0 || blue > 0 || green > 0) {
        image_buffer[j * image_width + k] =
            RGBA((int)(255 * red / visible_subpixels),
                 (int)(255 * green / visible_subpixels),
                 (int)(255 * blue / visible_subpixels), 255);
      }
    }
  }
}
#endif // CG_H_
