#ifndef GEOMETRY_H
#define GEOMETRY_H

typedef struct {
  int x, y;
} Vec2i;

typedef struct {
  float x, y;
} Vec2f;

typedef struct {
  float x, y, z;
} Vec3f;

Vec3f mult_scalar_vec(float s, Vec3f v);
float dot(Vec3f a, Vec3f b);
Vec3f crossProduct(Vec3f a, Vec3f b);
Vec3f add_vec(Vec3f a, Vec3f b);
Vec3f minus_vec(Vec3f a, Vec3f b);
Vec3f normalize(Vec3f a);
float edgeFunction(const Vec3f a, const Vec3f b, const Vec3f c);
Vec3f barycentric(Vec3f v0, Vec3f v1, Vec3f v2, Vec3f p);

typedef struct {
  float mat[4][4];
} Matrix44f;

Matrix44f identity();
Vec3f multVecMatrix(Vec3f v, Matrix44f m);
Matrix44f inverse(Matrix44f m);
Matrix44f lookAt(Vec3f from, Vec3f to);

typedef struct {
  float s; // real scalar part
  Vec3f v; // imaginary part
} Quat;

#define QUAT_NEW(s, v)                                                         \
  (Quat) { (s), (v) }
Matrix44f quat_rotation_matrix(Quat *q);
Quat quat_multiply(Quat q1, Quat q2);
Quat quat_from_axis_angle(Vec3f axis, float radians);

#endif
