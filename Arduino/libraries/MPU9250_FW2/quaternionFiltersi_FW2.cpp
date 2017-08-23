#include "quaternionFilters_FW2.h"

// These are the free parameters in the Mahony filter and fusion scheme, Kp
// for proportional feedback, Ki for integral
#define Kp 2.0f * 5.0f

float invSqrt(float);

// Similar to Madgwick scheme but uses proportional and integral filtering on
// the error between estimated reference vectors and measured ones.
void Quaternion::MahonyQuaternionUpdate(float ax, float ay, float az,
                                        float gx, float gy, float gz,
                                        float mx, float my, float mz)
{
  // short name local variable for readability
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];
  float norm;
  float hx, hy, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float ex, ey, ez;
  float pa, pb, pc;

  // Auxiliary variables to avoid repeated arithmetic
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;

  float Now = micros();
  float deltat = 0.0f;
  if (lastUpdate != 0.0f)
    deltat = ((Now - lastUpdate) / 1000000.0f);
  lastUpdate = Now;

  // Normalise accelerometer measurement
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    norm = invSqrt(ax * ax + ay * ay + az * az);

    ax *= norm;
    ay *= norm;
    az *= norm;
  }

  // Normalise magnetometer measurement
  if (!((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))) {
    norm = invSqrt(mx * mx + my * my + mz * mz);

    mx *= norm;
    my *= norm;
    mz *= norm;
  }

  // Reference direction of Earth's magnetic field
  hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
  hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
  bx = sqrtf((hx * hx) + (hy * hy));
  bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

  // Estimated direction of gravity and magnetic field
  vx = q2q4 - q1q3;
  vy = q1q2 + q3q4;
  vz = q1q1 - 0.5f + q4q4;
  wx = bx * (0.5f - q3q3 - q4q4) + bz * (q2q4 - q1q3);
  wy = bx * (q2q3 - q1q4) + bz * (q1q2 + q3q4);
  wz = bx * (q1q3 + q2q4) + bz * (0.5f - q2q2 - q3q3);

  // Error is cross product between estimated direction and measured direction of gravity
  ex = (ay * vz - az * vy) + (my * wz - mz * wy);
  ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
  ez = (ax * vy - ay * vx) + (mx * wy - my * wx);

  // Apply feedback terms
  gx = gx + Kp * ex;
  gy = gy + Kp * ey;
  gz = gz + Kp * ez;
 
  // Integrate rate of change of quaternion
  pa = q2;
  pb = q3;
  pc = q4;
  q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
  q2 = pa + ( q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
  q3 = pb + ( q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
  q4 = pc + ( q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

  // Normalise quaternion
  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
  norm = 1.0f / norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;
}

// Fast inverse square root
float invSqrt(float x) {
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}

const float * Quaternion::getQ () { return q; }