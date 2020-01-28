/* ===================================
 * MPU 6050 Library Test
 * ===================================
 * Prints the sensor values to serial monitor
 *
 * @author Denis Zholob
 */

// ================================================================================================================
// Importing Libraries
// ================================================================================================================
#include "SimpleMPU6050.h"

// ================================================================================================================
// Declaring Objects
// ================================================================================================================
SimpleMPU6050 MPU6050_Obj;

GyroscopeData gyroscope;
AccelerometerData accelerometer;
// EulerAngles attitude;
int temperature;
AngularRate angular_rate;

// ================================================================================================================
// Setup routine: Runs once when you press reset or power on the board
// ================================================================================================================
void setup() {
  // Open the serial port and set the baud rate to 9600
  Serial.begin(9600);

  // IMU
  MPU6050_Obj.setup();
  MPU6050_Obj.calibrateGyroscope();
}

// ================================================================================================================
// Main program loop: Runs over and over again forever
// ================================================================================================================
void loop() {
  MPU6050_Obj.readMPU6050Data();
  MPU6050_Obj.getGyroscopeData(&gyroscope);
  MPU6050_Obj.getAccelerometerData(&accelerometer);
  MPU6050_Obj.getTemperature(&temperature);
  MPU6050_Obj.getAngularRate(&angular_rate);
  printAngularRateData();
}

// 
void printAllData(){
    printAngularRateData();
    printGyroData();
    printAccelData();
    printTemperatureData();
}

// 
void printAngularRateData(){
  Serial.print("Roll-X: ");
  Serial.print(angular_rate.x);
  Serial.print("    Pitch-Y: ");
  Serial.print(angular_rate.y);
  Serial.print("        Yaw-Z: ");
  Serial.println(angular_rate.z);
}

// 
void printGyroData(){
  Serial.print("Gyro-X: ");
  Serial.print(gyroscope.x);
  Serial.print("    Gyro-Y: ");
  Serial.print(gyroscope.y);
  Serial.print("        Gyro-Z: ");
  Serial.println(gyroscope.z);
}

// 
void printAccelData(){
  Serial.print("Accel-X: ");
  Serial.print(accelerometer.x);
  Serial.print("    Accel-Y: ");
  Serial.print(accelerometer.y);
  Serial.print("        Accel-Z: ");
  Serial.println(accelerometer.z);
}

// 
void printTemperatureData(){
  Serial.print("Temperature: ");
  Serial.println(temperature);
}
