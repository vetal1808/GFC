#include "types.h"

void Vector3_sub(Vector3 * a, Vector3 * b, Vector3 * res){
	res->x = a->x - b->x;
	res->y = a->y - b->y;
	res->z = a->z - b->z;
}
