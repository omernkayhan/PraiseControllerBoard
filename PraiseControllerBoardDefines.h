#ifndef PraiseControllerBoardDefines_h
#define PraiseControllerBoardDefines_h

#define MOTOR_A_DIR 7
#define MOTOR_A_PWM 6

#define MOTOR_B_DIR 8
#define MOTOR_B_PWM 9

#define LED_R 10
#define LED_G 11
#define LED_B 5

#define BUZZER 12

#define TACTICAL_SWITCH 4

#define ENCODER 3
#define ENCODER_INTERRUPT_TYPE CHANGE
#define ENCODER_MULTIPLIER 0.1

#define QTR_0 A5
#define QTR_1 A4
#define QTR_2 A3
#define QTR_3 A2
#define QTR_4 A1
#define QTR_5 A0
#define QTR_6 A7
#define QTR_7 A6

#define PID_KP_DEFAULT 1
#define PID_KD_DEFAULT 1
#define PID_KI_DEFAULT 1

#define BASE_SPEED_DEFAULT 1
#define MAX_SPEED_DEFAULT 1
#define MIN_SPEED_DEFAULT 1

#define LEFT false
#define RIGHT true

#endif