/**
 * @File Name: motor.c
 * @brief 
 * @author qz (zheqiu2021@163.com)
 * @version 0.1
 * @date 2022-04-08
 * 
 * @copyright Copyright (c) 2022
 */
#include "motor.h"

#include "can.h"
#include "rotate.h"

/* 角度阈值 */
#define ANGLE_THRESHOLD 2000

motor_t motor_2006[9] = {0};
angle_t angle_data[9] = {0};

/**
 * @brief  角度连续化
 * @param  ad
 */
void angle_continuous(angle_t *ad) {
    if ((ad->angle_current <= ANGLE_THRESHOLD) &&
        (ad->angle_previous >= (8192 - ANGLE_THRESHOLD))) {
        ad->circle++;
    } else if ((ad->angle_current >= (8192 - ANGLE_THRESHOLD)) &&
               (ad->angle_previous <= ANGLE_THRESHOLD)) {
        ad->circle--;
    }
}

/**
 * @brief  数据接收
 * @param  rec_data
 * @param  ad
 * @param  md
 */
void data_receive(uint8_t *rec_data, angle_t *ad, motor_t *md) {
    /* 将收到的8位数据合并为16位 */
    ad->angle_previous = ad->angle_current;
    ad->angle_current = rec_data[0] << 8 | rec_data[1];
    angle_continuous(ad);

    md->rx_data.continuous_angle =
        1000 * (ad->circle + ad->angle_current / 8192.0f);

    md->rx_data.speed = rec_data[2] << 8 | rec_data[3];

    md->rx_data.current = rec_data[4] << 8 | rec_data[5];
}

/**
 * @brief  数据处理
 * @param  ad
 * @param  md
 */
void data_process(angle_t *ad, motor_t *md) {
    /* 角度环 */
    md->tarspeed = pid_absolute_update(
        md->tarangle, md->rx_data.continuous_angle, &md->pid_angle);

    /* 速度环 */
    md->output =
        pid_absolute_update(md->tarspeed, md->rx_data.speed, &md->pid_speed);
    //    /* 电流环 */
    //    md->output = pid_absolute_update(md->tarcurrent, md->rx_data.current,
    //                                     &md->pid_current);

    md->output = (md->output > 10000) ? 10000 : (md->output);
    md->output = (md->output < -10000) ? (-10000) : (md->output);
}

/**
 * @brief  数据发送
 */
void output_transmit(void) {
    uint8_t txdata1_can1[8] = {0}; /* 标识符：0x200 */
    uint8_t txdata2_can1[4] = {0}; /* 标识符：0x1FF */
    uint8_t txdata_can2[6] = {0};

    /* can1: 0x201--0x204 */
    txdata1_can1[0] = (uint8_t)(motor_2006[0].output >> 8);
    txdata1_can1[1] = (uint8_t)motor_2006[0].output;
    txdata1_can1[2] = (uint8_t)(motor_2006[1].output >> 8);
    txdata1_can1[3] = (uint8_t)motor_2006[1].output;
    txdata1_can1[4] = (uint8_t)(motor_2006[2].output >> 8);
    txdata1_can1[5] = (uint8_t)motor_2006[2].output;
    txdata1_can1[6] = (uint8_t)(motor_2006[3].output >> 8);
    txdata1_can1[7] = (uint8_t)motor_2006[3].output;

    can1_transmit(0X200, txdata1_can1);
    /* can1: 0x205--0x206 */
    txdata2_can1[0] = (uint8_t)(motor_2006[4].output >> 8);
    txdata2_can1[1] = (uint8_t)motor_2006[4].output;
    txdata2_can1[2] = (uint8_t)(motor_2006[5].output >> 8);
    txdata2_can1[3] = (uint8_t)motor_2006[5].output;

    can1_transmit(0X1FF, txdata2_can1);
    /* can2: 0x201--0x203 */
    txdata_can2[0] = (uint8_t)(motor_2006[6].output >> 8);
    txdata_can2[1] = (uint8_t)motor_2006[6].output;
    txdata_can2[2] = (uint8_t)(motor_2006[7].output >> 8);
    txdata_can2[3] = (uint8_t)motor_2006[7].output;
    txdata_can2[4] = (uint8_t)(motor_2006[8].output >> 8);
    txdata_can2[5] = (uint8_t)motor_2006[8].output;

    can2_transmit(0X200, txdata_can2);

    HAL_Delay(1);
}
