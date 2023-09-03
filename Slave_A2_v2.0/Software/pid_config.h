#ifndef _PID_CONFIG_H
#define _PID_CONFIG_H

#ifdef __cplusplus
 extern "C" {
#endif

/******************* variables *******************/
typedef struct {
	float init_kp;
	float init_ki;
	float init_kd;
	float init_kp_lim;
	float init_ki_lim;
} pid_config;

extern pid_config angle_config;
extern pid_config speed_config;
extern pid_config current_config;

/******************* function *******************/
void pid_set(void);

#ifdef __cplusplus
}
#endif
#endif
