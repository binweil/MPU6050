#ifndef __DMP_H
#define __DMP_H
void IMUupdate(float gx,float gy,float gz,float ax,float ay,float az);
float GetRoll(float aacx,float aacy,float aacz);
float GetPitch(float aacx,float aacy,float aacz);
float GetYaw(float gyroz);

void KalmanFilterz(float gyroz);
float Kalman_Filter_X(float accel,float gyro);
float Kalman_Filter_Y(float accel,float gyro);
float Kalman_Filter_Z(float accel,float gyro);
float KalFilter(float newAngle, float newRate,float dt);
float Filter(float aacx);

#endif

