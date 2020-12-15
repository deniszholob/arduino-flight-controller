/* ===================================
 * MPU 6050 library
 * ===================================
 * Simple Reading of Gyro and Accelerometer Data
 * Conversion to Euler angles
 * Complementary filter
 *
 * @author Denis Zholob
 */

// ================================================================================================================
// Importing Libraries
// ================================================================================================================
#include "SimpleMPU6050.h"

// ================================================================================================================
// Public Functions
// ================================================================================================================

// Constructor
// SimpleMPU6050::SimpleMPU6050(){
//   setup();    // Setup gyro settings
// }

// Setup
void SimpleMPU6050::setup() {
  Serial.println("Starting MPU6050...");
  Wire.begin();   // Start I2C as master
  configMPU650(); // Setup the registers of the MPU-6050 (500dfs / +/-8g) and start the gyro
  delay(250);     // Give the gyro time to start
  Serial.println("MPU6050 Ready!");
}

void SimpleMPU6050::calibrateGyroscope() {
  Serial.println("Starting Gyro Calibration...");
  // Run this code 2000 times
  for (int i = 0; i < MPU6050_GYRO_CALIBRATION_SAMPLES; i++) {
    readMPU6050Data();                             // Read the raw acc and gyro data from the MPU-6050
    _gyroscope_calibration.x += _gyroscope_data.x; // Add the gyro x-axis offset to the x calibration variable
    _gyroscope_calibration.y += _gyroscope_data.y; // Add the gyro y-axis offset to the y calibration variable
    _gyroscope_calibration.z += _gyroscope_data.z; // Add the gyro z-axis offset to the z calibration variable

    // Show calibration progress
    if (i % 100 == 0) {
      Serial.print(i / 100);
      Serial.print(".");
    }
    delay(3); // Delay 3us to simulate the 250Hz program loop
  }
  Serial.println("");

  _gyroscope_calibration.x /= MPU6050_GYRO_CALIBRATION_SAMPLES; // Divide the x calibration variable by 2000 to get the avarage offset
  _gyroscope_calibration.y /= MPU6050_GYRO_CALIBRATION_SAMPLES; // Divide the y calibration variable by 2000 to get the avarage offset
  _gyroscope_calibration.z /= MPU6050_GYRO_CALIBRATION_SAMPLES; // Divide the z calibration variable by 2000 to get the avarage offset

  Serial.print("Gyro Calib X: ");
  Serial.print(_gyroscope_calibration.x);
  Serial.print("  Gyro Calib Y: ");
  Serial.print(_gyroscope_calibration.y);
  Serial.print("    Gyro Calib Z: ");
  Serial.println(_gyroscope_calibration.z);

  Serial.println("Gyro Calibration Done!");
}

// Reads gyro, acc, and temp data from the device
void SimpleMPU6050::readMPU6050Data() {
  Wire.beginTransmission(MPU6050_SIGNAL_PATH_RESET); // Start communicating with the MPU-6050
  Wire.write(MPU6050_ACCEL_XOUT_H);                  // Send the requested starting register
  Wire.endTransmission(false);                       // Hold the I2C-bus

  // Request 14 bytes from the MPU-6050
  // Third parameter is true: relase I2C-bus after data is read.
  Wire.requestFrom(MPU6050_SIGNAL_PATH_RESET, MPU6050_NUM_REQUESTED_BYTES, true);

  // Wait until all the bytes are received
  while (Wire.available() < MPU6050_NUM_REQUESTED_BYTES) {};

  // Read in Device Data
  _accelerometer_data.x = readNextTwoRegisters();
  _accelerometer_data.y = readNextTwoRegisters();
  _accelerometer_data.z = readNextTwoRegisters();
  _temperature = readNextTwoRegisters();
  _gyroscope_data.x = readNextTwoRegisters();
  _gyroscope_data.y = readNextTwoRegisters();
  _gyroscope_data.z = readNextTwoRegisters();
}

// Fills in passed in variable with new data
void SimpleMPU6050::getGyroscopeData(GyroscopeData *gyroscope_data) {
  gyroscope_data->x = _gyroscope_data.x - _gyroscope_calibration.x;
  gyroscope_data->y = _gyroscope_data.y - _gyroscope_calibration.y;
  gyroscope_data->z = _gyroscope_data.z - _gyroscope_calibration.z;
}

// Fills in passed in variable with new data
void SimpleMPU6050::getAccelerometerData(AccelerometerData *accelerometer_data) {
  accelerometer_data->x = _accelerometer_data.x;
  accelerometer_data->y = _accelerometer_data.y;
  accelerometer_data->z = _accelerometer_data.z;
}

// Fills in passed in variable with new data
void SimpleMPU6050::getTemperature(int *temperature) {
  *temperature = _temperature;
}

// Fills in passed in variable with new data
// Angular rate units is deg/sec.
void SimpleMPU6050::getAngularRate(AngularRate *angular_rate) {
  float ang_rate; // Temp var for calculation

  // Calculate new rate for X
  ang_rate = (_gyroscope_data.x - _gyroscope_calibration.x) / MPU6050_GYRO_TO_ANGULAR_RATE;
  _angular_rate.x = complementaryFilter(_angular_rate.x, ang_rate);

  // Calculate new rate for Y
  ang_rate = (_gyroscope_data.y - _gyroscope_calibration.y) / MPU6050_GYRO_TO_ANGULAR_RATE;
  _angular_rate.y = complementaryFilter(_angular_rate.y, ang_rate);

  // Calculate new rate for Z
  ang_rate = (_gyroscope_data.z - _gyroscope_calibration.z) / MPU6050_GYRO_TO_ANGULAR_RATE;
  _angular_rate.z = complementaryFilter(_angular_rate.z, ang_rate);

  // Uptate Values
  angular_rate->x = _angular_rate.x;
  angular_rate->y = _angular_rate.y;
  angular_rate->z = _angular_rate.z;
}


// ================================================================================================================
// Private Functions
// ================================================================================================================

// Wakes up the device
// Sets up gyro and accel sensitivity
void SimpleMPU6050::configMPU650() {
  // Activate the MPU-6050
  writeRegister(MPU6050_PWR_MGMT_1, 0x00);

  // Configure the gyro (500dps full scale)
  writeRegister(MPU6050_GYRO_CONFIG, 0x08);

  // Configure the accelerometer (+/-8g)
  writeRegister(MPU6050_ACCEL_CONFIG, 0x10);
}

// Helper function to write to device registers via I2C
void SimpleMPU6050::writeRegister(int register_address, int register_value) {
  Wire.beginTransmission(MPU6050_SIGNAL_PATH_RESET); // Start communicating with the MPU-6050
  Wire.write(register_address);                      // Send the requested starting register
  Wire.write(register_value);                        // Set the requested starting register
  Wire.endTransmission();                            // End the transmission
}

// Use Wire to read in 2 bytes and add them to make an int
int SimpleMPU6050::readNextTwoRegisters() {
  return Wire.read() << 8 | Wire.read(); // Add the low and high byte
}

// Complementatry filter weights older values more than new values
float SimpleMPU6050::complementaryFilter(float a, float b) {
  return (a * 0.8) + (b * 0.2);
}
