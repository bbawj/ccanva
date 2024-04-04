#ifndef CAMERA_H
#define CAMERA_H

#include "geometry.h"
#include <stdbool.h>
#include <stdint.h>
#include <sys/types.h>

#define ROTATION_VEL 0.05
#define TRACK_VEL 0.05
#define DOLLY_VEL 0.05

typedef enum {
  NONE,
  TUMBLE,
  TRACK,
  DOLLY,
} CameraMoveType;

typedef struct {
  CameraMoveType moveType;
  Vec3f pos;
  Vec3f lookat;
  float phi;   // The up-down angle
  float theta; // The left-right angle
  float dist_to_lookat;
  Quat rotation;
  Matrix44f rotation_matrix;
  Matrix44f cam_to_world;
} Camera;

bool init_camera(Camera *c, Vec3f pos, Vec3f lookat);
void camera_move(Camera *c, uint64_t delta, int32_t x_rel, int32_t y_rel);

#endif
