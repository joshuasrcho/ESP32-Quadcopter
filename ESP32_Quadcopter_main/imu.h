#ifndef IMU_H
#define IMU_H

void initIMU();
float* getYPR();
void getRotation(float* x, float* y, float* z);

#endif
