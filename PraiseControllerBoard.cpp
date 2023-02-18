#include "Arduino.h"
#include "PraiseControllerBoard.h"

//Use With #include <PraiseControllerBoard.h>

int EncoderCounter;
double EncoderDimension;
double EncoderMultiplier;

PraiseControllerBoardClass::PraiseControllerBoardClass() {

}

void PraiseControllerBoardClass::Setup() {

    pinMode(MOTOR_A_DIR, OUTPUT);
    pinMode(MOTOR_A_PWM, OUTPUT);
    pinMode(MOTOR_B_DIR, OUTPUT);
    pinMode(MOTOR_B_PWM, OUTPUT);
    pinMode(LED_R, OUTPUT);
    pinMode(LED_G, OUTPUT);
    pinMode(LED_B, INPUT);
    pinMode(BUZZER, OUTPUT);

    pinMode(TACTICAL_SWITCH, INPUT);
    pinMode(ENCODER, INPUT);
    for (int i = 0; i < 8; i++) pinMode(QTRSensorPins[i], INPUT);

    attachInterrupt(digitalPinToInterrupt(ENCODER), EncoderUp, CHANGE);

    FlashLedFullColor(5, 100);

}

PraiseControllerBoardClass::PraiseControllerBoardClass(double baseSpeed, double minSpeed, double maxSpeed) {
    PraiseControllerBoardClass();
    BaseSpeed = baseSpeed;
    MinSpeed = minSpeed;
    MaxSpeed = maxSpeed;
}

PraiseControllerBoardClass::PraiseControllerBoardClass(double baseSpeed, double minSpeed, double maxSpeed, double kp, double kd, double ki) {
    PraiseControllerBoardClass(baseSpeed, minSpeed, maxSpeed);
    PidKP = kp;
    PidKD = kd;
    PidKI = ki;
}

void PraiseControllerBoardClass::DriveMotorA(double speed) {
    speed = constrain(speed, -255, 255);
    digitalWrite(MOTOR_A_DIR, speed > 0);
    analogWrite(MOTOR_A_PWM, abs(speed));
}

void PraiseControllerBoardClass::DriveMotorB(double speed) {
    speed = constrain(speed, -255, 255);
    digitalWrite(MOTOR_B_DIR, speed > 0);
    analogWrite(MOTOR_B_PWM, abs(speed));
}

void PraiseControllerBoardClass::DriveMotors(double aMotorSpeed, double bMotorSpeed, int time = 0) {
    DriveMotorA(aMotorSpeed);
    DriveMotorB(bMotorSpeed);
    delay(time);
}

void PraiseControllerBoardClass::StopMotors(int time = 0) {
    DriveMotorA(0);
    DriveMotorB(0);
    delay(time);
}

void PraiseControllerBoardClass::ReadLineFollowerSensors() {
    for (int i = 0; i < 8; i++) QTRSensors[i] = analogRead(QTRSensorPins[i]);
}

void PraiseControllerBoardClass::CalculateError() {
    int sT = 0, T = 0;
    for (int i = 0; i < 8; i++) {
        sT += QTRSensors[i];
        T += QTRSensors[i] * i * 1000;
    }
    PidError = (sT > 0) ? ((T / sT) - 3500) : PidError;
}

void PraiseControllerBoardClass::CalculatePID() {
    PidIntegral += PidError;
    PidDerivative = PidError - PidLastError;
    PidControl = (PidError * PidKP) + (PidIntegral * PidKI) + (PidDerivative * PidKD);
    PidLastError = PidError;
}

void PraiseControllerBoardClass::CalculateMotorSpeed() {
    CalculatedSpeedMotorA = constrain(DriveSpeed + PidControl, MinSpeed, MaxSpeed);
    CalculatedSpeedMotorB = constrain(DriveSpeed - PidControl, MinSpeed, MaxSpeed);
}

void PraiseControllerBoardClass::Drive() {
    ReadLineFollowerSensors();
    CalculateError();
    CalculatePID();
    CalculateMotorSpeed();
    DriveMotors(CalculatedSpeedMotorA, CalculatedSpeedMotorB);
}

void PraiseControllerBoardClass::DriveWithSpeed(double speed) {
    SetSpeed(speed);
    ReadLineFollowerSensors();
    CalculateError();
    CalculatePID();
    CalculateMotorSpeed();
    DriveMotors(CalculatedSpeedMotorA, CalculatedSpeedMotorB);
}

void PraiseControllerBoardClass::DriveWithDistance(double distance, double speed /*= 9999*/) {
    speed = (speed == -9999) ? DriveSpeed : speed;
    ResetEncoder();
    while(GetEncoderDimension() < distance){
        DriveWithSpeed(DriveSpeed);
    }
    StopMotors();
}

void PraiseControllerBoardClass::ResetSpeed() {
    DriveSpeed = BaseSpeed;
}

void PraiseControllerBoardClass::SetSpeed(double speed) {
    DriveSpeed = speed;
}

void PraiseControllerBoardClass::SpeedUp(double speed /*= 0.1*/) {
    DriveSpeed += speed;
}

void PraiseControllerBoardClass::SpeedDown(double speed /*= 0.1*/) {
    DriveSpeed -= speed;
}

void PraiseControllerBoardClass::CalculateEncoderDimension() {
    EncoderDimension = EncoderCounter * EncoderMultiplier;
}

void PraiseControllerBoardClass::EncoderUp() {
    EncoderCounter++;
    CalculateEncoderDimension();
}

void PraiseControllerBoardClass::ResetEncoder() {
    EncoderCounter = 0;
    CalculateEncoderDimension();
}

double PraiseControllerBoardClass::GetEncoderDimension() {
    return EncoderDimension;
}

int PraiseControllerBoardClass::GetEncoderCounter() {
    return EncoderCounter;
}

void PraiseControllerBoardClass::SetEncoderMultiplier(double multiplier) {
    EncoderMultiplier = multiplier;
}

void PraiseControllerBoardClass::TurnLeftAndCatchLine(int time /*= 100*/) {
    DriveMotors(DriveSpeed, DriveSpeed, time);
    ReadLineFollowerSensors();
    while (QTRSensors[4] > 500){
        DriveMotors(DriveSpeed / 20, DriveSpeed);
        ReadLineFollowerSensors();
    }
}

void PraiseControllerBoardClass::TurnRightAndCatchLine(int time /*= 100*/) {
    DriveMotors(DriveSpeed, DriveSpeed, time);
    ReadLineFollowerSensors();
    while (QTRSensors[3] > 500){
        DriveMotors(DriveSpeed / 20, 0);
        ReadLineFollowerSensors();
    }
}

void PraiseControllerBoardClass::CatchTurns() {
    ReadLineFollowerSensors();
    if(QTRSensors[0] > 500 && QTRSensors[1] > 500 && QTRSensors[2] > 500){
        TurnLeftAndCatchLine();
    }else if(QTRSensors[5] > 500 && QTRSensors[6] > 500 && QTRSensors[7] > 500){
        TurnRightAndCatchLine();
    }else{
        Drive();
    }
}

void PraiseControllerBoardClass::CatchVLine(bool direction) {
    ReadLineFollowerSensors();
    if(QTRSensors[1] > 500 && QTRSensors[2] > 500 && QTRSensors[5] > 500 && QTRSensors[6] > 500){
        if(direction == LEFT){
            TurnLeftAndCatchLine(0);
        }else{
            TurnRightAndCatchLine(0);
        }
    }
}

void PraiseControllerBoardClass::RGBLed(int r, int g, int b) {
    analogWrite(LED_R, r);
    analogWrite(LED_G, g);
    analogWrite(LED_B, b);
}

void PraiseControllerBoardClass::FlashLedFullColor(int flashCount /*= 3*/, int time /*= 100*/) {
    for (int i = 0; i < flashCount; i++) {
        RGBLed(255, 255, 255);
        delay(time);
        RGBLed(0, 0, 0);
        delay(time);
    }
}

void PraiseControllerBoardClass::FlashLedR(int flashCount /*= 3*/, int time /*= 100*/) {
    for (int i = 0; i < flashCount; i++) {
        RGBLed(255, 0, 0);
        delay(time);
        RGBLed(0, 0, 0);
        delay(time);
    }
}

void PraiseControllerBoardClass::FlashLedG(int flashCount /*= 3*/, int time /*= 100*/) {
    for (int i = 0; i < flashCount; i++) {
        RGBLed(0, 255, 0);
        delay(time);
        RGBLed(0, 0, 0);
        delay(time);
    }
}

void PraiseControllerBoardClass::FlashLedB(int flashCount /*= 3*/, int time /*= 100*/) {
    for (int i = 0; i < flashCount; i++) {
        RGBLed(0, 0, 255);
        delay(time);
        RGBLed(0, 0, 0);
        delay(time);
    }
}

void PraiseControllerBoardClass::ScrollLed(int scrollCount /*= 3*/, int time /*= 100*/) {
    for (int i = 0; i < scrollCount; i++) {
        FlashLedR(1, time);
        FlashLedG(1, time);
        FlashLedB(1, time);
    }
}

void PraiseControllerBoardClass::Buzzer(int time /*= 100*/) {
    digitalWrite(BUZZER, HIGH);
    delay(time);
    digitalWrite(BUZZER, LOW);
    delay(time);
}

PraiseControllerBoardClass praise = PraiseControllerBoardClass();
