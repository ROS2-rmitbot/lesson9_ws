#ifndef MY_ENCODER_H // Standard guard while writing a library to prevent multiple inclusions of this header file
#define MY_ENCODER_H

#include <Arduino.h>      // Arduino library for basic functions
#include <ESP32Encoder.h> // ESP32Encoder library for handling encoders

class Encoder
{
public:
    Encoder(byte ENCA, byte ENCB); // Constructor for the Encoder class
    void begin();                  // Method to set the pin mode and enable the encoder
    long getCount();               // Method to read the encoder value
    double getVelocity();          // Method to get the velocity of the encoder

private:
    byte _ENCA, _ENCB;                    // Pin used on ESP32 for the ENC_A and ENC_B
    int _ENC_RES = 330;                   // Encoder resolution
    unsigned long w_time;                 // Time variable used to calculate the velocity
    double th, th_prev, w, w_prev, w_raw; // Variables used to calculate the velocity
    double alpha = .95;                   // Filter coefficient
    ESP32Encoder _encoder;                // ESP32Encoder object to handle the encoder
};

#endif
