/*
  Sends base64 encoded data through the serial port, so that it can be sent forward
  to the server by test.js. This version is used only for testing.

  Based on https://github.com/sparkfun/SparkFun_MPU-9250_Breakout_Arduino_Library
*/

#include "quaternionFilters_FW2.h"
#include "MPU9250_FW2.h"
#include "Base64.h" // Using ESP32's base64 library

#define PI_F 3.141593f

#define DELAY 10 // Delay between output messages (ms)
#define N_IMUS 1 // Number of connected IMUs

// #define CALIBRATE_MAG
#define CALIBRATION_DELAY 1000
#define CALIBRATION_DURATION 20

MPU9250 IMU[N_IMUS] = {MPU9250(16)};
Quaternion Quat[N_IMUS];
base64 b64;

uint8_t IMUbytes[20];
String encodedData;

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000);

  for (int i = 0; i < N_IMUS; i++)
    InitMPU(IMU[i]);
#ifdef CALIBRATE_MAG
  CalibrateMag();
#else
  IMU[0].magBias[0] = -100;
  IMU[0].magBias[1] = 165;
  IMU[0].magBias[2] = 70;
#endif

  Serial.println("Mag biases");
  Serial.println(IMU[0].magBias[0]);
  Serial.println(IMU[0].magBias[1]);
  Serial.println(IMU[0].magBias[2]);
}

unsigned long outputTimer = 0;

void loop()
{
  // Try to read all IMUs
  for (int i = 0; i < N_IMUS; i++)
    ReadMPU(i);
    
  // Output data every DELAY milliseconds
  if (outputTimer <= millis()) {
    // Reset output timer
    outputTimer = millis() + DELAY;

    for (int i = 0; i < N_IMUS; i++) {
      const float *q = Quat[i].getQ();

      // Calculate yaw, pitch and roll from the quaternion
      // Angles are sent as integers with value from 0 to 1624 combined into
      // one unsigned long (32 bits)
      uint32_t yaw   = (atan2f(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] +
                       q[1] * q[1] - q[2] * q[2] - q[3] * q[3]) + PI_F) * 258.467621f;
      uint32_t pitch = (-asinf(2.0f * (q[1] * q[3] - q[0] * q[2])) + PI_F) * 258.467621f;
      uint32_t roll  = (atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] -
                       q[1] * q[1] - q[2] * q[2] + q[3] * q[3]) + PI_F) * 258.467621f;

      uint32_t IMUdata = pitch * 1625 * 1625 + yaw * 1625 + roll;

      // Repeat the same data 5 times to send comparable data to 5 IMUs
      for (int k = 0; k < 5; k++) {
        IMUbytes[0 + k * 4] = (IMUdata >> 24) & 0xFF;
        IMUbytes[1 + k * 4] = (IMUdata >> 16) & 0xFF;
        IMUbytes[2 + k * 4] = (IMUdata >>  8) & 0xFF;
        IMUbytes[3 + k * 4] =  IMUdata        & 0xFF;
      }
    }

    // Encode IMU data in base64
    encodedData = b64.encode(IMUbytes, 20);

    Serial.println(encodedData);
  }
}

void ReadMPU(int i)
{
  if (IMU[i].readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    IMU[i].writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);
    delayMicroseconds(100);

    IMU[i].readAccelData(IMU[i].accelCount);  // Read the x/y/z adc values
    IMU[i].ax = (float)IMU[i].accelCount[0] * IMU[i].aRes - IMU[i].accelBias[0];
    IMU[i].ay = (float)IMU[i].accelCount[1] * IMU[i].aRes - IMU[i].accelBias[1];
    IMU[i].az = (float)IMU[i].accelCount[2] * IMU[i].aRes - IMU[i].accelBias[2];

    IMU[i].readGyroData(IMU[i].gyroCount);  // Read the x/y/z adc valu
    IMU[i].gx = ((float)IMU[i].gyroCount[0] * IMU[i].gRes - IMU[i].gyroBias[0]) * 0.0174533f;
    IMU[i].gy = ((float)IMU[i].gyroCount[1] * IMU[i].gRes - IMU[i].gyroBias[1]) * 0.0174533f;
    IMU[i].gz = ((float)IMU[i].gyroCount[2] * IMU[i].gRes - IMU[i].gyroBias[2]) * 0.0174533f;

    IMU[i].readMagData(IMU[i].magCount);  // Read the x/y/z adc values
    IMU[i].mx = - (float)IMU[i].magCount[1] * IMU[i].mRes * IMU[i].factoryMagCalibration[1] - IMU[i].magBias[1];
    IMU[i].my = - (float)IMU[i].magCount[0] * IMU[i].mRes * IMU[i].factoryMagCalibration[0] - IMU[i].magBias[0];
    IMU[i].mz =   (float)IMU[i].magCount[2] * IMU[i].mRes * IMU[i].factoryMagCalibration[2] - IMU[i].magBias[2];
    IMU[i].writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x20);

    Quat[i].MahonyQuaternionUpdate(IMU[i].ax, IMU[i].ay, IMU[i].az,
                                   IMU[i].gx, IMU[i].gy, IMU[i].gz,
                                   IMU[i].mx, IMU[i].my, IMU[i].mz);
  } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
}

void InitMPU(MPU9250 &mpu)
{
  // Read the WHO_AM_I register, this is a good test of communication
  byte c = mpu.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  Serial.print(F("MPU9250 I AM 0x"));
  Serial.print(c, HEX);
  Serial.print(F(" I should be 0x"));
  Serial.println(0x71, HEX);

  if (c != 0x71) { // WHO_AM_I should always be 0x71
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);

    // Communication failed, stop here
    Serial.println(F("Communication failed, abort!"));
    Serial.flush();
    abort();
  }

  Serial.println(F("MPU9250 is online..."));

  // Start by performing self test and reporting values
  mpu.MPU9250SelfTest(mpu.selfTest);
  Serial.print(F("x-axis self test: acceleration trim within : "));
  Serial.print(mpu.selfTest[0],1); Serial.println("% of factory value");
  Serial.print(F("y-axis self test: acceleration trim within : "));
  Serial.print(mpu.selfTest[1],1); Serial.println("% of factory value");
  Serial.print(F("z-axis self test: acceleration trim within : "));
  Serial.print(mpu.selfTest[2],1); Serial.println("% of factory value");
  Serial.print(F("x-axis self test: gyration trim within : "));
  Serial.print(mpu.selfTest[3],1); Serial.println("% of factory value");
  Serial.print(F("y-axis self test: gyration trim within : "));
  Serial.print(mpu.selfTest[4],1); Serial.println("% of factory value");
  Serial.print(F("z-axis self test: gyration trim within : "));
  Serial.print(mpu.selfTest[5],1); Serial.println("% of factory value");

  // Calibrate gyro and accelerometers, load biases in bias registers
  mpu.calibrateMPU9250(mpu.gyroBias, mpu.accelBias);

  mpu.initMPU9250();
  // Initialize device for active mode read of acclerometer, gyroscope, and
  // temperature
  Serial.println("MPU9250 initialized for active data mode....");

  // Read the WHO_AM_I register of the magnetometer, this is a good test of
  // communication
  byte d = mpu.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
  Serial.print("AK8963 ");
  Serial.print("I AM 0x");
  Serial.print(d, HEX);
  Serial.print(" I should be 0x");
  Serial.println(0x48, HEX);

  if (d != 0x48) {
    Serial.print("Could not connect to AK8963: 0x");
    Serial.println(d, HEX);

    // Communication failed, stop here
    Serial.println(F("Communication failed, abort!"));
    Serial.flush();
    abort();
  }

  // Get magnetometer calibration from AK8963 ROM
  mpu.initAK8963(mpu.factoryMagCalibration);
  // Initialize device for active mode read of magnetometer
  Serial.println("AK8963 initialized for active data mode....");

  //  Serial.println("Calibration values: ");
  Serial.print("X-Axis factory sensitivity adjustment value ");
  Serial.println(mpu.factoryMagCalibration[0], 2);
  Serial.print("Y-Axis factory sensitivity adjustment value ");
  Serial.println(mpu.factoryMagCalibration[1], 2);
  Serial.print("Z-Axis factory sensitivity adjustment value ");
  Serial.println(mpu.factoryMagCalibration[2], 2);

  // Get sensor resolutions, only need to do this once
  mpu.getAres();
  mpu.getGres();
  mpu.getMres();
    
  // Disable I2C bypass mode for AK8963
  mpu.writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x20);
}

void CalibrateMag()
{
  uint16_t ii = 0, sample_count = 0;
  int32_t mag_bias[N_IMUS][3],
          mag_scale[N_IMUS][3];
  int16_t mag_max[N_IMUS][3],
          mag_min[N_IMUS][3],
          mag_temp[N_IMUS][3];

  for (int i = 0; i < N_IMUS; i++) {
    // Init values
    for (int k = 0; k < 3; k++) {
      mag_bias[i][k]  = 0;
      mag_scale[i][k] = 0;
      mag_max[i][k]   = 0;
      mag_min[i][k]   = 0x7FFF;
      mag_temp[i][k]  = 0;
    }
    // Make sure resolution has been calculated
    IMU[i].getMres();
  }

  Serial.println(F("Mag Calibration: Wave device in a figure 8 until done!"));
  Serial.println(F("  4 seconds to get ready followed by 15 seconds of sampling)"));
  delay(CALIBRATION_DELAY);

  // shoot for ~fifteen seconds of mag data
  // at 8 Hz ODR, new mag data is available every 125 ms
  sample_count = CALIBRATION_DURATION * 8;

  for (ii = 0; ii < sample_count; ii++)
  {
    for (int i = 0; i < N_IMUS; i++) {
      IMU[i].writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);
      delayMicroseconds(100);
      IMU[i].readMagData(mag_temp[i]);  // Read the mag data
      IMU[i].writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x20);

      for (int jj = 0; jj < 3; jj++)
      {
        if (mag_temp[i][jj] > mag_max[i][jj])
          mag_max[i][jj] = mag_temp[i][jj];
        if (mag_temp[i][jj] < mag_min[i][jj])
          mag_min[i][jj] = mag_temp[i][jj];
      }
    }

    delay(135); // At 8 Hz ODR, new mag data is available every 125 ms
  }

  for (int i = 0; i < N_IMUS; i++) {
    // Get hard iron correction
    // Get 'average' x mag bias in counts
    mag_bias[i][0]  = (mag_max[i][0] + mag_min[i][0]) / 2;
    // Get 'average' y mag bias in counts
    mag_bias[i][1]  = (mag_max[i][1] + mag_min[i][1]) / 2;
    // Get 'average' z mag bias in counts
    mag_bias[i][2]  = (mag_max[i][2] + mag_min[i][2]) / 2;

    // Save mag biases in G for main program
    IMU[i].magBias[0] = (float)mag_bias[i][0] * IMU[i].mRes * IMU[i].factoryMagCalibration[0];
    IMU[i].magBias[1] = (float)mag_bias[i][1] * IMU[i].mRes * IMU[i].factoryMagCalibration[1];
    IMU[i].magBias[2] = (float)mag_bias[i][2] * IMU[i].mRes * IMU[i].factoryMagCalibration[2];

    // Get soft iron correction estimate
    // Get average x axis max chord length in counts
    mag_scale[i][0]  = (mag_max[i][0] - mag_min[i][0]) / 2;
    // Get average y axis max chord length in counts
    mag_scale[i][1]  = (mag_max[i][1] - mag_min[i][1]) / 2;
    // Get average z axis max chord length in counts
    mag_scale[i][2]  = (mag_max[i][2] - mag_min[i][2]) / 2;

    float avg_rad = mag_scale[i][0] + mag_scale[i][1] + mag_scale[i][2];
    avg_rad /= 3.0;

    IMU[i].magScale[0] = avg_rad / ((float)mag_scale[i][0]);
    IMU[i].magScale[1] = avg_rad / ((float)mag_scale[i][1]);
    IMU[i].magScale[2] = avg_rad / ((float)mag_scale[i][2]);
  }

  Serial.println(F("Mag Calibration done!"));
}
