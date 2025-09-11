#include "MyMotor.h"

Motor::Motor(byte MOTA, byte MOTB, int PWMA, int PWMB)

{
    _MOTA = MOTA;
    _MOTB = MOTB;
    _PWMA = PWMA;
    _PWMB = PWMB;
}

void Motor::begin()
{
    // Configure the motor pins as output
    pinMode(_MOTA, OUTPUT);
    pinMode(_MOTB, OUTPUT);
    // Set up the PWM channels for the motors
    ledcSetup(_PWMA, _PWM_FREQ, _PWM_RES);
    ledcSetup(_PWMB, _PWM_FREQ, _PWM_RES);
    // Attach the PWM channels to the motor pins
    ledcAttachPin(_MOTA, _PWMA);
    ledcAttachPin(_MOTB, _PWMB);
}

void Motor::send_pwm(double motor_cmd)
{
    if (motor_cmd < 0)
    {
        ledcWrite(_PWMA, 1);
        ledcWrite(_PWMB, abs(motor_cmd));
    }
    else
    {
        ledcWrite(_PWMB, 1);
        ledcWrite(_PWMA, abs(motor_cmd));
    }
}