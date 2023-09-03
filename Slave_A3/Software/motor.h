#ifndef _MOTOR_H
#define _MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

/******************* define *******************/
#include <stdint.h>

#include "pid.h"

/******************* variables *******************/
typedef struct {
    uint16_t angle_current;   //电机返回的当前角度
    int32_t circle;           //电机转过的总圈数
    uint16_t angle_previous;  //之前的角度
} angle_t;

typedef struct {
    float continuous_angle;  //连续化角度
    int16_t speed;
    int16_t current;
} motor_data;

typedef struct {
    PID_AbsoluteType pid_angle;
    PID_AbsoluteType pid_speed;
    PID_AbsoluteType pid_current;
    motor_data rx_data;
    int32_t tarangle;
    int16_t tarspeed;
    int16_t tarcurrent;
    int16_t output;
} motor_t;

extern motor_t motor_2006[9];
extern angle_t angle_data[9];

/******************* function *******************/
void angle_continuous(angle_t *ad);
void data_receive(uint8_t *rec_data, angle_t *ad, motor_t *md);
void data_process(angle_t *ad, motor_t *md);
void output_transmit(void);

#ifdef __cplusplus
}
#endif
#endif
