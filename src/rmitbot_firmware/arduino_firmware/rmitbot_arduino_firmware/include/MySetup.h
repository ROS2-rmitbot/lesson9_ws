
// ==============================================
// Pin definitions for the encoders
// ==============================================
#define ENC1_A 23 // Pin used on ESP32 for the ENC1_A
#define ENC1_B 22 // Pin used on ESP32 for the ENC1_B
#define ENC2_A 4 // Pin used on ESP32 for the ENC2_A
#define ENC2_B 16 // Pin used on ESP32 for the ENC2_B
#define ENC3_A 14 // Pin used on ESP32 for the ENC3_A
#define ENC3_B 13 // Pin used on ESP32 for the ENC3_B
#define ENC4_A 39 // Pin used on ESP32 for the ENC4_A
#define ENC4_B 36 // Pin used on ESP32 for the ENC4_B

// ==============================================
// Pin definitions for the motors
// ==============================================
#define MOT1_A 21 // Pin used on ESP32 for the MOT1_A
#define MOT1_B 19  // Pin used on ESP32 for the MOT1_B
#define MOT2_A 17 // Pin used on ESP32 for the MOT2_A
#define MOT2_B 18  // Pin used on ESP32 for the MOT2_B
#define MOT3_A 26 // Pin used on ESP32 for the MOT3_A
#define MOT3_B 27  // Pin used on ESP32 for the MOT3_B
#define MOT4_A 33 // Pin used on ESP32 for the MOT4_A
#define MOT4_B 25  // Pin used on ESP32 for the MOT4_B

// ==============================================
// PWM Channel definitions for the motors
// ==============================================
#define PWM1_A 0 // PWM Channel attached to MOT1_A
#define PWM1_B 1 // PWM Channel attached to MOT1_B
#define PWM2_A 2 // PWM Channel attached to MOT2_A
#define PWM2_B 3 // PWM Channel attached to MOT2_B
#define PWM3_A 4 // PWM Channel attached to MOT3_A
#define PWM3_B 5 // PWM Channel attached to MOT3_B
#define PWM4_A 6 // PWM Channel attached to MOT4_A
#define PWM4_B 7 // PWM Channel attached to MOT4_B

// ==============================================
// Variables
// ==============================================
unsigned long Serial_time = 0; // Serial time in us
double w1, w1_ref, MOT1_cmd;   // Speed, reference ,and command for the motor 1
double w2, w2_ref, MOT2_cmd;   // Speed, reference ,and command for the motor 2
double w3, w3_ref, MOT3_cmd;   // Speed, reference ,and command for the motor 3
double w4, w4_ref, MOT4_cmd;   // Speed, reference ,and command for the motor 4
volatile long EncoderTick1;    // Encoder tick count for encoder 1
volatile long EncoderTick2;    // Encoder tick count for encoder 2
volatile long EncoderTick3;    // Encoder tick count for encoder 3
volatile long EncoderTick4;    // Encoder tick count for encoder 4
