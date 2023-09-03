#ifndef _PID_H
#define _PID_H

#ifdef __cplusplus
 extern "C" {
#endif

/******************* variables *******************/
typedef struct
{
    float kp;
    float ki;
    float kd;
    float errILim_up;   //��������
    float errILim_down; //��������
    float errPLim; //��������
    float errLim;
    float errNow;
    float errOld;
    float errP;
    float errI;
    float errD;
    float ctrOut;
} PID_AbsoluteType;         /* ����ʽpid */

/******************* function *******************/
float pid_absolute_update(float Target, float Current, PID_AbsoluteType *PID);
void pid_init(PID_AbsoluteType *pid, float kp, float ki, float kd, float kp_lim, float ki_lim);

#ifdef __cplusplus
}
#endif
#endif
