#ifndef PraiseControllerBoard_h
#define PraiseControllerBoard_h

#include "PraiseControllerBoardDefines.h"

class PraiseControllerBoardClass {
public:

    int QTRSensorPins[8] = {QTR_0, QTR_1, QTR_2, QTR_3, QTR_4, QTR_5, QTR_6, QTR_7};
    int QTRSensors[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    double PidKP = PID_KP_DEFAULT, PidKD = PID_KD_DEFAULT, PidKI = PID_KI_DEFAULT, PidError, PidLastError, PidDerivative, PidIntegral, PidControl;

    double DriveSpeed = BASE_SPEED_DEFAULT, BaseSpeed = BASE_SPEED_DEFAULT, MaxSpeed = MAX_SPEED_DEFAULT, MinSpeed = MIN_SPEED_DEFAULT, CalculatedSpeedMotorA, CalculatedSpeedMotorB;

    PraiseControllerBoardClass();
    PraiseControllerBoardClass(double baseSpeed, double minSpeed, double maxSpeed);
    PraiseControllerBoardClass(double baseSpeed, double minSpeed, double maxSpeed, double kp, double kd, double ki);

    void Setup();

    void DriveMotorA(double speed);
    void DriveMotorB(double speed);
    void DriveMotors(double aMotorSpeed, double bMotorSpeed, int time);
    void StopMotors(int time);

    void ResetSpeed();
    void SetSpeed(double speed);
    void SpeedUp(double increaseSpeed = 0.1);
    void SpeedDown(double decreaseSpeed = 0.1);

    void ReadLineFollowerSensors();

    void CalculateError();
    void CalculatePID();
    void CalculateMotorSpeed();

    void Drive();
    void DriveWithSpeed(double speed);
    void DriveWithDistance(double distance, double speed = -9999);

    void TurnLeftAndCatchLine(int time = 100);
    void TurnRightAndCatchLine(int time = 100);
    void CatchTurns();
    void CatchVLine(bool direction);

    void ResetEncoder();
    void SetEncoderMultiplier(double multiplier);
    double GetEncoderDimension();
    int GetEncoderCounter();

    void RGBLed(int r, int g, int b);
    void FlashLedFullColor(int flashCount = 3, int time = 100);
    void FlashLedR(int flashCount = 3, int time = 100);
    void FlashLedG(int flashCount = 3, int time = 100);
    void FlashLedB(int flashCount = 3, int time = 100);
    void ScrollLed(int scrollCount = 3, int time = 100);
    void Buzzer(int time);

private:
    static void CalculateEncoderDimension();
    static void EncoderUp();

};

extern PraiseControllerBoardClass praise;

#endif
