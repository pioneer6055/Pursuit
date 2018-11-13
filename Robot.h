#pragma once

#include "AHRS.h"
#include "Pursuit.h"
#include "WPILib.h"
#include <iostream>

class Robot : public frc::TimedRobot
{
public:
    void RobotInit() override;
    void AutonomousInit() override;
    void AutonomousPeriodic() override;
    void TeleopInit() override;
    void TeleopPeriodic() override;
    void TestInit() override;
    void TestPeriodic() override;
    void DisabledInit() override;
    void DisabledPeriodic() override;
    double GetHeading();
    void ZeroHeading();
    double GetNormalizedHeading(double heading);
    double GetDistance();
    double GetLeftDistance();
    double GetRightDistance();
    int GetThumbWheel();

private:
    Spark *MotorLeft;
    Spark *MotorRight;
    DifferentialDrive *DriveTrain;
    Counter *LeftDistCounter;
    Counter *RightDistCounter;
    DigitalInput *ThumbWheel_1;
    DigitalInput *ThumbWheel_2;
    DigitalInput *ThumbWheel_4;
    DigitalInput *ThumbWheel_8;
    AHRS *Gyro;
    Pursuit *AutoPursuit;
    int TICKS_PER_REVOLUTION = 699; 
    double WHEEL_CIRCUMFERENCE_FEET = 1.112647;
    double HeadingOffset = 0.0f;
    Timer *ElapsedTimer;
    int ThumbWheel = 0;
    int Auto_State = 0;
};

