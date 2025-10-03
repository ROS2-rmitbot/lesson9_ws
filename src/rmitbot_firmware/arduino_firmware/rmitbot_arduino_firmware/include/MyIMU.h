#ifndef MY_IMU_H // Include guard to prevent multiple inclusions of this header file
#define MY_IMU_H

#include <Arduino.h> // Arduino library for basic functions
#include "ICM_20948.h"

#define I2C_SDA 23 // Pin used on ESP32 for I2C SDA
#define I2C_SCL 22 // Pin used on ESP32 for I2C SCL

void IMUBegin();     // Function to initialize the IMU
void IMUGetData();   // Function to get the data from the IMU
void IMUGetData_Uncalibrated(); // Function to get the uncalibrated data from the IMU
void IMUCalibrate(); // Function to calibrate the IMU

#endif
