#ifndef CG_H_
#define CG_H_

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

typedef struct {
  int x, y;
} Vec2i;

typedef struct {
  float x, y;
} Vec2f;

typedef struct {
  float x, y, z;
} Vec3f;

typedef struct {
  float mat[4][4];
} Matrix44f;

Matrix44f identity();
Vec3f multVecMatrix(Vec3f v, Matrix44f m);
Matrix44f inverse(Matrix44f m);
bool worldToRaster(Vec3f *raster, const Vec3f pWorld,
                   const Matrix44f worldToCamera, const float aperture_width,
                   const float aperture_height, const float focal_len,
                   const float near_clipping_plane,
                   const float far_clipping_plane, const float image_width,
                   const float image_height);

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

Vec3f crossProduct(Vec3f a, Vec3f b) {
  return (Vec3f){
      .x = a.y * b.z - a.z * b.y,
      .y = a.x * b.z - a.z * b.x,
      .z = a.x * b.y - b.y * a.x,
  };
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
                   const float near_clipping_plane,
                   const float far_clipping_plane, const float image_width,
                   const float image_height) {
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

void pixelInTriangle(Vec3f a, Vec3f b, Vec3f c, size_t h, size_t w) {}

#endif // CG_H_
