#ifndef __TYPES_H__
#define __TYPES_H__

#include "stdint.h"

typedef struct{
   uint16_t LFW, RFC,
   	   	   LBC, RBW;
} Rotor4;
typedef struct{
   float x, y, z;
} Vector3;
typedef struct{
   int16_t x, y, z;
} Vector3_int16;

void Vector3_sub(Vector3 * a, Vector3 * b, Vector3 * res);
#endif
