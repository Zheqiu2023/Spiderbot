#ifndef _ROTATE_H
#define _ROTATE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "motor.h"

/******************* define *******************/
#define CIRCLE_MULTIPLE		1000		/* 圈数放大倍数 */

/******************* variables *******************/
extern uint8_t flag[9];
extern const float traj1[][3];
extern const float traj2[][3];
extern uint8_t cnt;

/******************* function *******************/
void tarangle_switch(const float (*traj)[3]);
float angle_conversion(float theta);

#ifdef __cplusplus
}
#endif
#endif
