/* ===================================
 * MPU 6050 library
 * ===================================
 * Simple Reading of Gyro and Accelerometer Data
 * Conversion to Euler angles
 * Complementary filter
 *
 * @author Denis Zholob
 */

#ifndef SimpleMPU6050_h
#define SimpleMPU6050_h

// ================================================================================================================
// Importing Libraries
// ================================================================================================================
#include <Arduino.h>
#include <Wire.h>

// ================================================================================================================
// Declaring Constants (Magic numbers are BAD!)
// ================================================================================================================
// clang-format off
#define MPU6050_SIGNAL_PATH_RESET  0x68   // R/W
#define MPU6050_PWR_MGMT_1         0x6B   // R/W
#define MPU6050_GYRO_CONFIG        0x1B   // R/W
#define MPU6050_ACCEL_CONFIG       0x1C   // R/W

#define MPU6050_ACCEL_XOUT_H       0x3B   // R
#define MPU6050_ACCEL_XOUT_L       0x3C   // R
#define MPU6050_ACCEL_YOUT_H       0x3D   // R
#define MPU6050_ACCEL_YOUT_L       0x3E   // R
#define MPU6050_ACCEL_ZOUT_H       0x3F   // R
#define MPU6050_ACCEL_ZOUT_L       0x40   // R
#define MPU6050_TEMP_OUT_H         0x41   // R
#define MPU6050_TEMP_OUT_L         0x42   // R
#define MPU6050_GYRO_XOUT_H        0x43   // R
#define MPU6050_GYRO_XOUT_L        0x44   // R
#define MPU6050_GYRO_YOUT_H        0x45   // R
#define MPU6050_GYRO_YOUT_L        0x46   // R
#define MPU6050_GYRO_ZOUT_H        0x47   // R
#define MPU6050_GYRO_ZOUT_L        0x48   // R
// clang-format on

#define MPU6050_NUM_REQUESTED_BYTES 14        // 2*7 <- high and low byte for 3accel + 3gyro + temp
#define MPU6050_GYRO_CALIBRATION_SAMPLES 1000 // Calibration samples
#define MPU6050_GYRO_TO_ANGULAR_RATE 57.14286 //
// ================================================================================================================
// Declaring Structures
// ================================================================================================================

struct GyroscopeData {
  int x;
  int y;
  int z;
};

struct AccelerometerData {
  int x;
  int y;
  int z;
};

// struct EulerAngles {
//  float x;
//  float y;
//  float z;
// };

struct GyroscopeCalibration {
  double x;
  double y;
  double z;
};

struct AngularRate {
  double x;
  double y;
  double z;
};

class SimpleMPU6050 {
public:
  // SimpleMPU6050();
  // ~SimpleMPU650();
  void setup();
  void calibrateGyroscope();
  void readMPU6050Data();
  void getGyroscopeData(GyroscopeData *gyroscope_data);
  void getAccelerometerData(AccelerometerData *accelerometer_data);
  // void getEulerAngles(EulerAngles *euler_angles);
  void getTemperature(int *temperature);
  void getAngularRate(AngularRate *angular_rate);

private:
  GyroscopeCalibration _gyroscope_calibration;
  GyroscopeData _gyroscope_data;
  AccelerometerData _accelerometer_data;
  AngularRate _angular_rate;
  // EulerAngles _euler_angles;
  int _temperature;

  void configMPU650();
  void writeRegister(int register_address, int register_value);
  int readNextTwoRegisters();
  float complementaryFilter(float a, float b);
};

#endif
