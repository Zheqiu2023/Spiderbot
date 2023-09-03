/**
 * @File Name: pid.c
 * @brief 
 * @author qz (zheqiu2021@163.com)
 * @version 0.1
 * @date 2022-04-08
 * 
 * @copyright Copyright (c) 2022
 */
#include "pid.h"

#include "motor.h"
#include "pid_config.h"

pid_config angle_config;
pid_config speed_config;
pid_config current_config;

/**
 * @brief  pid�㷨
 * @param  Target           
 * @param  Current          
 * @param  PID              
 * @return float            
 */
float pid_absolute_update(float Target, float Current, PID_AbsoluteType *PID) {
    PID->errNow = Target - Current;
    PID->errP = PID->errNow;   //��ȡ���ڵ�������kp����
    PID->errI += PID->errNow;  //�����֣�����ki����
    /*�������޺�����*/
    if (PID->errLim != 0)  
    {
        if (PID->errI > PID->errLim)
            PID->errI = PID->errLim;
        else if (PID->errI < -PID->errLim)
            PID->errI = -PID->errLim;
    }
    /* ����������޺����� */
    if (PID->errPLim != 0) {
        if (PID->errP > PID->errPLim) {
            PID->errP = PID->errPLim;
        } else if (PID->errP < -PID->errPLim) {
            PID->errP = -PID->errPLim;
        }
    }

    if (PID->errNow < 5) {
        PID->errI = 0;
    }

    PID->errD = PID->errNow - PID->errOld;  //���΢�֣�����kd����
    PID->errOld = PID->errNow;              //�������ڵ����
    PID->ctrOut = PID->kp * PID->errP + PID->ki * PID->errI +
                  PID->kd * PID->errD;  //�������ʽPID���

    return PID->ctrOut;
}

/**
 * @brief  pid��ʼ��
 * @param  pid              
 * @param  kp               
 * @param  ki               
 * @param  kd               
 * @param  kp_lim           
 * @param  ki_lim           
 */
void pid_init(PID_AbsoluteType *pid, float kp, float ki, float kd, float kp_lim,
              float ki_lim) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->errPLim = kp_lim;
    pid->errLim = ki_lim;
}
							
/**
 * @brief  pid��������
 */
void pid_set(void) {
    /* ����pid���� */
    for (int i = 0; i < 9; i += 3) {
        /******************* �ǶȻ� *******************/
        angle_config.init_kp = 1.0;
        angle_config.init_ki = 0.05;
        angle_config.init_kd = 0.0;
        angle_config.init_kp_lim = 9999;
        angle_config.init_ki_lim = 9999;

        pid_init(&motor_2006[i].pid_angle, angle_config.init_kp,
                 angle_config.init_ki, angle_config.init_kd,
                 angle_config.init_kp_lim, angle_config.init_ki_lim);
        /******************* �ٶȻ� *******************/
        speed_config.init_kp = 7;
        speed_config.init_ki = 0.0;
        speed_config.init_kd = 0;
        speed_config.init_kp_lim = 9999;
        speed_config.init_ki_lim = 9999;

        pid_init(&motor_2006[i].pid_speed, speed_config.init_kp,
                 speed_config.init_ki, speed_config.init_kd,
                 speed_config.init_kp_lim, speed_config.init_ki_lim);
//        /******************* ������ *******************/
//        current_config.init_kp = 0.0;
//        current_config.init_ki = 0;
//        current_config.init_kd = 0;
//        current_config.init_kp_lim = 9999;
//        current_config.init_ki_lim = 9999;

//        pid_init(&motor_2006[i].pid_current, current_config.init_kp,
//                 current_config.init_ki, current_config.init_kd,
//                 current_config.init_kp_lim,
//                 current_config.init_ki_lim);
    }
		
    /* ����pid���� */
    for (int i = 1; i < 9; i += 3) {
        /******************* �ǶȻ� *******************/
        angle_config.init_kp = 1.0;
        angle_config.init_ki = 0.0;
        angle_config.init_kd = 0.0;
        angle_config.init_kp_lim = 9999;
        angle_config.init_ki_lim = 9999;

        pid_init(&motor_2006[i].pid_angle, angle_config.init_kp,
                 angle_config.init_ki, angle_config.init_kd,
                 angle_config.init_kp_lim, angle_config.init_ki_lim);
        /******************* �ٶȻ� *******************/
        speed_config.init_kp = 1;
        speed_config.init_ki = 0.1;
        speed_config.init_kd = 0.0;
        speed_config.init_kp_lim = 9999;
        speed_config.init_ki_lim = 9999;

        pid_init(&motor_2006[i].pid_speed, speed_config.init_kp,
                 speed_config.init_ki, speed_config.init_kd,
                 speed_config.init_kp_lim, speed_config.init_ki_lim);
//        /******************* ������ *******************/
//        current_config.init_kp = 0.0;
//        current_config.init_ki = 0;
//        current_config.init_kd = 0;
//        current_config.init_kp_lim = 9999;
//        current_config.init_ki_lim = 9999;

//        pid_init(&motor_2006[i].pid_current, current_config.init_kp,
//                 current_config.init_ki, current_config.init_kd,
//                 current_config.init_kp_lim,
//                 current_config.init_ki_lim);
    }
		
    /* С��pid���� */
    for (int i = 2; i < 9; i += 3) {
        /******************* �ǶȻ� *******************/
        angle_config.init_kp = 1.0;
        angle_config.init_ki = 0.0;
        angle_config.init_kd = 0.0;
        angle_config.init_kp_lim = 9999;
        angle_config.init_ki_lim = 9999;

        pid_init(&motor_2006[i].pid_angle, angle_config.init_kp,
                 angle_config.init_ki, angle_config.init_kd,
                 angle_config.init_kp_lim, angle_config.init_ki_lim);
        /******************* �ٶȻ� *******************/
        speed_config.init_kp = 1;
        speed_config.init_ki = 0.1;
        speed_config.init_kd = 0;
        speed_config.init_kp_lim = 9999;
        speed_config.init_ki_lim = 9999;

        pid_init(&motor_2006[i].pid_speed, speed_config.init_kp,
                 speed_config.init_ki, speed_config.init_kd,
                 speed_config.init_kp_lim, speed_config.init_ki_lim);
        //        /******************* ������ *******************/
        //        current_config.init_kp = 0.0;
        //        current_config.init_ki = 0;
        //        current_config.init_kd = 0;
        //        current_config.init_kp_lim = 9999;
        //        current_config.init_ki_lim = 9999;

        //        pid_init(&motor_2006[i].pid_current, current_config.init_kp,
        //                 current_config.init_ki, current_config.init_kd,
        //                 current_config.init_kp_lim,
        //                 current_config.init_ki_lim);
    }
}
