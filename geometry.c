#include "geometry.h"
#include <math.h>
#include <stdio.h>

Matrix44f identity() {
  Matrix44f identity = {.mat = {
                            {1, 0, 0, 0},
                            {0, 1, 0, 0},
                            {0, 0, 1, 0},
                            {0, 0, 0, 1},
                        }};
  return identity;
}

Matrix44f multMatrix(Matrix44f a, Matrix44f b) {
  Matrix44f res;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      res.mat[i][j] = a.mat[i][0] * b.mat[0][j] + a.mat[i][1] * b.mat[1][j] +
                      a.mat[i][2] * b.mat[2][j] + a.mat[i][3] * b.mat[3][j];
    }
  }
  return res;
}

Matrix44f transpose(Matrix44f m) {
  Matrix44f res = m;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      if (i == j)
        continue;

      res.mat[i][j] = m.mat[j][i];
    }
  }
  return res;
}

Vec3f multVecMatrix(Vec3f v, Matrix44f m) {
  Vec3f res;
  // v.w == 1 homogenous coordinate
  res.x =
      v.x * m.mat[0][0] + v.y * m.mat[1][0] + v.z * m.mat[2][0] + m.mat[3][0];
  res.y =
      v.x * m.mat[0][1] + v.y * m.mat[1][1] + v.z * m.mat[2][1] + m.mat[3][1];
  res.z =
      v.x * m.mat[0][2] + v.y * m.mat[1][2] + v.z * m.mat[2][2] + m.mat[3][2];
  float w =
      v.x * m.mat[0][3] + v.y * m.mat[1][3] + v.z * m.mat[2][3] + m.mat[3][3];

  // w should be 1 as it is a homogeneous coordinate, normalize if not 1
  if (w != 1) {
    res.x /= w;
    res.y /= w;
    res.z /= w;
  }
  return res;
}

Vec3f mult_scalar_vec(float s, Vec3f v) {
  return (Vec3f){v.x * s, v.y * s, v.z * s};
}

float dot(Vec3f a, Vec3f b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

Vec3f crossProduct(Vec3f a, Vec3f b) {
  return (Vec3f){
      .x = a.y * b.z - a.z * b.y,
      .y = a.x * b.z - a.z * b.x,
      .z = a.x * b.y - b.y * a.x,
  };
}

Vec3f vec_add(Vec3f a, Vec3f b) {
  return (Vec3f){
      .x = a.x + b.x,
      .y = a.y + b.y,
      .z = a.z + b.z,
  };
}

Vec3f vec_minus(Vec3f a, Vec3f b) {
  return (Vec3f){
      .x = a.x - b.x,
      .y = a.y - b.y,
      .z = a.z - b.z,
  };
}

float vec_magnitude(Vec3f v) { return sqrtf(dot(v, v)); }

Vec3f normalize(Vec3f a) {
  float magnitude = vec_magnitude(a);
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

Matrix44f lookAt(Vec3f from, Vec3f to) {
  Vec3f forward = normalize(vec_minus(from, to));
  Vec3f temp_up = {0, 1, 0};
  Vec3f right = normalize(crossProduct(temp_up, forward));
  Vec3f up = crossProduct(forward, right);

  return (Matrix44f){.mat = {{right.x, right.y, right.z, 0},
                             {up.x, up.y, up.z, 0},
                             {forward.x, forward.y, forward.z, 0},
                             {from.x, from.y, from.z, 1}}};
}

// https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation#Quaternion-derived_rotation_matrix
Matrix44f quat_rotation_matrix(Quat *q) {
  Vec3f v = q->v;
  const float s = q->s;
  const float x = v.x;
  const float y = v.y;
  const float z = v.z;
  return (Matrix44f){
      .mat =
          {
              {1 - 2 * (y * y + z * z), 2 * (x * y - z * s),
               2 * (x * y + y * s), 0},
              {2 * (x * y + z * s), 1 - 2 * (x * x + z * z),
               2 * (y * z - x * s), 0},
              {2 * (x * z - y * s), 2 * (y * z + x * s),
               1 - 2 * (x * x + y * y), 0},
              {0, 0, 0, 1},
          },
  };
}

// https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation#Quaternions
Quat quat_multiply(Quat q1, Quat q2) {
  const float s = q1.s;
  const float t = q2.s;
  Vec3f v = q1.v;
  Vec3f w = q2.v;
  return QUAT_NEW(s * t - dot(v, w),
                  vec_add(vec_add(mult_scalar_vec(s, w), mult_scalar_vec(t, v)),
                          crossProduct(v, w)));
}

Quat quat_from_axis_angle(Vec3f axis, float radians) {

  return QUAT_NEW(cosf(radians / 2),
                  mult_scalar_vec(sinf(radians / 2), normalize(axis)));
}
