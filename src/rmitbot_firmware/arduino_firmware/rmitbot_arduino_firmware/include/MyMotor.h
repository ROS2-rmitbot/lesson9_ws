#ifndef MY_MOTOR_H // Include guard to prevent multiple inclusions of this header file
#define MY_MOTOR_H

#include <Arduino.h>

class Motor
{
public:
    Motor(byte MOTA, byte MOTB,
          int PWMA, int PWMB);       // Constructor to initialize the motor object
    void begin();                    // Method to set the pin mode and enable the motors
    void send_pwm(double motor_cmd); // Method to send the PWM signal to the motors

private:
    byte _MOTA, _MOTB;     // Pin used on ESP32 for the MOT_A
    int _PWMA, _PWMB;      // PWM Channel attached to MOT_A
    // int _PWM_FREQ = 20000; // Frequency of the PWM signal
    // int _PWM_RES = 8;      // Resolution of the PWM signal
    int _PWM_FREQ = 12000; // Frequency of the PWM signal
    int _PWM_RES = 12;      // Resolution of the PWM signal
};

#endif