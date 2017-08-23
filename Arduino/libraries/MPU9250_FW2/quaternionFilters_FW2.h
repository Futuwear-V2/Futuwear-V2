#ifndef _QUATERNIONFILTERS_H_
#define _QUATERNIONFILTERS_H_

#include <Arduino.h>

class Quaternion {
public:
    void MahonyQuaternionUpdate(float ax, float ay, float az,
                                float gx, float gy, float gz,
                                float mx, float my, float mz);
    const float * getQ();

private:
    // Vector to hold quaternion
    float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float lastUpdate = 0.0f;
};

#endif // _QUATERNIONFILTERS_H_