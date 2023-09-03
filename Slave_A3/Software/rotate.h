#ifndef _ROTATE_H
#define _ROTATE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "motor.h"

/******************* define *******************/
#define LEG_CIRCLE 1000 * 1.5 * 36	/* ��С��Ŀ��Ȧ�� */
#define COXA_CIRCLE 1000 * 4		/* ����Ŀ��Ȧ�� */

/******************* variables *******************/
extern uint8_t flag[9];

/******************* function *******************/
void direction_set(uint8_t *flag, angle_t *ad, motor_t *md);
void tarangle_set(uint8_t state);

#ifdef __cplusplus
}
#endif
#endif
