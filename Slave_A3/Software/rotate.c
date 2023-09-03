/**
 * @File Name: rotate.c
 * @brief 
 * @author qz (zheqiu2021@163.com)
 * @version 0.1
 * @date 2022-04-08
 * 
 * @copyright Copyright (c) 2022
 */
#include "rotate.h"

uint8_t flag[9] = {0}; /* 正反转标志 */

/**
 * @brief  电机转动方向设置
 * @param  flag
 * @param  ad
 * @param  md
 */
void direction_set(uint8_t *flag, angle_t *ad, motor_t *md) {
    /* 根据圈数设置正反转标志和目标角度 */
    if (ad->circle >= (LEG_CIRCLE / 1000) && *flag == 0) {
        *flag = 1;
        md->tarangle = 0;
    } else if (ad->circle <= 0 && *flag == 1) {
        *flag = 0;
    }
}

/**
 * @brief  目标角度设置
 * @param  state
 */
void tarangle_set(uint8_t state) {
    switch (state) {
//        case 1:							/* 可有可无 */
//			/* 基节目标角度 */
//            motor_2006[0].tarangle = 0;
//            motor_2006[3].tarangle = 0;
//            motor_2006[6].tarangle = 0;
//            /* 大腿目标角度 */
//            motor_2006[1].tarangle = 0;
//            motor_2006[4].tarangle = 0;
//            motor_2006[7].tarangle = 0;
//            /* 小腿目标角度 */
//            motor_2006[2].tarangle = 0;
//            motor_2006[5].tarangle = 0;
//            motor_2006[8].tarangle = 0;
//			break;
        case 2:
            /* 基节目标角度 */
            motor_2006[0].tarangle = COXA_CIRCLE;
            motor_2006[3].tarangle = -COXA_CIRCLE;
            motor_2006[6].tarangle = COXA_CIRCLE;
            /* 大腿目标角度 */
            motor_2006[1].tarangle = LEG_CIRCLE;
            motor_2006[4].tarangle = LEG_CIRCLE;
            motor_2006[7].tarangle = LEG_CIRCLE;
            /* 小腿目标角度 */
            motor_2006[2].tarangle = LEG_CIRCLE;
            motor_2006[5].tarangle = LEG_CIRCLE;
            motor_2006[8].tarangle = LEG_CIRCLE;
            break;
        case 3:
            /* 基节目标角度 */
            motor_2006[0].tarangle = 0;
            motor_2006[3].tarangle = 0;
            motor_2006[6].tarangle = 0;
//            /* 大腿目标角度 */				/* 可有可无 */
//            motor_2006[1].tarangle = 0;
//            motor_2006[4].tarangle = 0;
//            motor_2006[7].tarangle = 0;
//            /* 小腿目标角度 */
//            motor_2006[2].tarangle = 0;
//            motor_2006[5].tarangle = 0;
//            motor_2006[8].tarangle = 0;
            break;
    }
}
