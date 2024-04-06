#ifndef LIGHT_H
#define LIGHT_H

#include "geometry.h"

typedef struct {
  Vec3f dir;
  Vec3f color;
  float intensity;
} Light;

#endif // !LIGHT_H
