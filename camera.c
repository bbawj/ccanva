#include "camera.h"
#include "geometry.h"
#include "log.h"
#include <math.h>
#include <stdio.h>

void static update_cam_to_world(Camera *c) {
  Vec3f camera_dir = multVecMatrix((Vec3f){0, 0, 1}, c->rotation_matrix);
  c->pos = add_vec(c->lookat, mult_scalar_vec(c->dist_to_lookat, camera_dir));
  c->cam_to_world = c->rotation_matrix;
  c->cam_to_world.mat[3][0] = c->pos.x;
  c->cam_to_world.mat[3][1] = c->pos.y;
  c->cam_to_world.mat[3][2] = c->pos.z;
}

bool init_camera(Camera *c, Vec3f pos, Vec3f lookat) {
  c->moveType = NONE;
  c->pos = pos;
  c->lookat = lookat;
  c->phi = 0;
  c->theta = 0;
  c->dist_to_lookat = 20;
  c->rotation = QUAT_NEW(1, ((Vec3f){0, 0, 0}));
  c->rotation_matrix = identity();

  update_cam_to_world(c);

  return true;
}

void camera_move(Camera *c, uint64_t delta, int x_rel, int y_rel) {
  switch (c->moveType) {
  case NONE:
    return;
  case TUMBLE: {
    c->phi += x_rel * ROTATION_VEL;
    if (c->phi > 2 * M_PI)
      c->phi = 0;
    else if (c->phi < 0) {
      c->phi = 2 * M_PI;
    }
    c->theta += y_rel * ROTATION_VEL;
    if (c->theta > 2 * M_PI)
      c->theta = 0;
    else if (c->theta < 0) {
      c->theta = 2 * M_PI;
    }
    Quat elevation = quat_from_axis_angle((Vec3f){1, 0, 0}, c->theta);
    Quat sideways = quat_from_axis_angle((Vec3f){0, 1, 0}, c->phi);
    c->rotation = quat_multiply(elevation, sideways);
    c->rotation_matrix = quat_rotation_matrix(&c->rotation);
  } break;
  case TRACK: {
    c->lookat.x += x_rel * TRACK_VEL;
    c->lookat.y += y_rel * TRACK_VEL;
  } break;
  case DOLLY: {
    c->dist_to_lookat += y_rel * DOLLY_VEL;
  } break;
  }

  DEBUG("camera phi: %f, theta: %f, dist: %f\n", c->phi, c->theta,
        c->dist_to_lookat);
  update_cam_to_world(c);
}
