#include "Robot.h"

void Robot::RobotInit()
{
    std::cout << "[ROBOT INIT]\n";
    SetPeriod(0.02);
    MotorLeft = new Spark(0);
    MotorRight = new Spark(1);
    DriveTrain = new DifferentialDrive(*MotorLeft,*MotorRight);
    LeftDistCounter = new Counter(0);
    RightDistCounter = new Counter(1);
    ThumbWheel_1 = new DigitalInput(10); //DI on NAVX
    ThumbWheel_2 = new DigitalInput(11); //DI on NAVX
    ThumbWheel_4 = new DigitalInput(12); //DI on NAVX
    ThumbWheel_8 = new DigitalInput(13); //DI on NAVX
    AutoPursuit = new Pursuit();
    DriveTrain->SetSafetyEnabled(false);
    MotorLeft->SetSafetyEnabled(false);
    MotorRight->SetSafetyEnabled(false);
    MotorLeft->SetInverted(true);
    MotorRight->SetInverted(false);
    ElapsedTimer = new Timer();
    ElapsedTimer->Start();
    try
    {
        Gyro = new AHRS(SPI::Port::kMXP,60);
        std::cout << "NAVX Initialized OK\n";
    }
    catch (std::exception& ex )
    {
        std::string err_string = "Error initializing navX:  ";
        err_string += ex.what();
        DriverStation::ReportError(err_string.c_str());
    }
}

void Robot::AutonomousInit()
{
    std::cout << "[AUTONOMOUS INIT]\n";
    ZeroHeading();
    //determines which autonomous profile to run
    ThumbWheel = GetThumbWheel();
    Auto_State = 0;
    ElapsedTimer->Reset();
    LeftDistCounter->Reset();
    RightDistCounter->Reset();
}

void Robot::AutonomousPeriodic()
{
    switch(ThumbWheel)
    {
        case 1:
            switch(Auto_State)
            {
                case 0:
                    AutoPursuit->PURSUIT_TURN_GAIN = 350;
                    AutoPursuit->PURSUIT_TARGET_PERCENT = 0.85;
                    AutoPursuit->PURSUIT_MAX_SPEED = 1;
                    AutoPursuit->ClearPath();
                    AutoPursuit->AddWaypoint(2,0);
                    AutoPursuit->AddWaypoint(4,0);
                    AutoPursuit->Initialize(ElapsedTimer->Get());
                    Auto_State++;
                    break;
                case 1:
                    Pursuit_Output po = AutoPursuit->Execute(GetDistance(),GetHeading(),ElapsedTimer->Get());
                    MotorLeft->Set(po.LeftMotorOut);
                    MotorRight->Set(po.RightMotorOut);
                    if(po.PathComplete)
                    {
                        std::cout << "Auto Complete in " << ElapsedTimer->Get() << " seconds\n";
                        std::cout << "LeftDist= " << GetLeftDistance() << " ft\n";
                        std::cout << "RightDist= " << GetRightDistance() << " ft\n";
                        Auto_State++;
                    }
                    break;
            }
            break;
        case 2:
            switch(Auto_State)
            {
                case 0:
                    AutoPursuit->PURSUIT_TURN_GAIN = 350;
                    AutoPursuit->PURSUIT_TARGET_PERCENT = 0.85;
                    AutoPursuit->PURSUIT_MAX_SPEED = 1;
                    AutoPursuit->ClearPath();
                    AutoPursuit->AddWaypoint(3,0);
                    AutoPursuit->AddWaypoint(3,3);
                    AutoPursuit->AddWaypoint(0,3);
                    AutoPursuit->AddWaypoint(0,0);
                    AutoPursuit->Initialize(ElapsedTimer->Get());
                    Auto_State++;
                    break;
                case 1:
                    Pursuit_Output po = AutoPursuit->Execute(GetDistance(),GetHeading(),ElapsedTimer->Get());
                    MotorLeft->Set(po.LeftMotorOut);
                    MotorRight->Set(po.RightMotorOut);
                    if(po.PathComplete)
                    {
                        std::cout << "Auto Complete in " << ElapsedTimer->Get() << " seconds\n";
                        Auto_State++;
                    }
                    break;
            }
            break;
        case 3:
            switch(Auto_State)
            {
                case 0:
                    AutoPursuit->PURSUIT_TURN_GAIN = 100;
                    AutoPursuit->PURSUIT_TARGET_PERCENT = 0.95;
                    AutoPursuit->PURSUIT_MAX_SPEED = 1;
                    AutoPursuit->ClearPath();
                    AutoPursuit->AddWaypoint(4,0);
                    AutoPursuit->AddWaypoint(0,0);
                    AutoPursuit->Initialize(ElapsedTimer->Get());
                    Auto_State++;
                    break;
                case 1:
                    Pursuit_Output po = AutoPursuit->Execute(GetDistance(),GetHeading(),ElapsedTimer->Get());
                    MotorLeft->Set(po.LeftMotorOut);
                    MotorRight->Set(po.RightMotorOut);
                    if(po.PathComplete)
                    {
                        std::cout << "Auto Complete in " << ElapsedTimer->Get() << " seconds\n";
                        std::cout << "LeftDist= " << GetLeftDistance() << " ft\n";
                        std::cout << "RightDist= " << GetRightDistance() << " ft\n";
                        Auto_State++;
                    }
                    break;
            }
            break;
        default:
            DriveTrain->ArcadeDrive(0.0,0.0);
            break;
    }
}

void Robot::TeleopInit()
{
    std::cout << "[TELEOP INIT]\n";
}

void Robot::TeleopPeriodic()
{

}

void Robot::TestInit()
{
    std::cout << "[TEST INIT]\n";
}

void Robot::TestPeriodic()
{

}

void Robot::DisabledInit()
{
    std::cout << "[DISABLED]\n";
}

void Robot::DisabledPeriodic()
{

}

double Robot::GetHeading()
{
    try
    {
        double offsetYaw = GetNormalizedHeading(Gyro->GetYaw()) - HeadingOffset;
        if(offsetYaw < 0) offsetYaw += 360;
        return offsetYaw;
    }
    catch(std::exception& ex)
    {
        std::string err_string = "[Get_Heading] ";
        err_string += ex.what();
        std::cout << "[ERROR Get_Heading] " << err_string << std::endl;
        return 0.0f;
    }
}

void Robot::ZeroHeading()
{
    try
    {
        HeadingOffset = GetNormalizedHeading(Gyro->GetYaw());
    }
    catch(std::exception& ex)
    {
        std::string err_string = "[ZeroHeading] ";
        err_string += ex.what();
        std::cout << "[ERROR ZeroHeading] " << err_string << std::endl;
        HeadingOffset = 0.0f;
    }
}

double Robot::GetNormalizedHeading(double heading)
{
    if(heading > 180) return heading -= 360;
    else if(heading < -180) return heading += 360;
    else return heading;
}

double Robot::GetDistance()
{
    double ldc = (double)LeftDistCounter->Get() * 0.5;
    double rdc = (double)RightDistCounter->Get() * 0.5;
    double tpr = (double)TICKS_PER_REVOLUTION;
    return ((ldc +rdc) / tpr)  * WHEEL_CIRCUMFERENCE_FEET;
}

double Robot::GetLeftDistance()
{
    double ldc = (double)LeftDistCounter->Get();
    double tpr = (double)TICKS_PER_REVOLUTION;
    return (ldc / tpr)  * WHEEL_CIRCUMFERENCE_FEET;
}

double Robot::GetRightDistance()
{
    double rdc = (double)RightDistCounter->Get();
    double tpr = (double)TICKS_PER_REVOLUTION;
    return (rdc / tpr)  * WHEEL_CIRCUMFERENCE_FEET;
}

int Robot::GetThumbWheel()
{
    bool d1 = ThumbWheel_1->Get();
    bool d2 = ThumbWheel_2->Get();
    bool d4 = ThumbWheel_4->Get();
    bool d8 = ThumbWheel_8->Get();

    int ret = 0;
    if(!d1 && d2 && d4 && !d8) ret = 9;
    if(d1 && d2 && d4 && !d8) ret = 8;
    if(!d1 && !d2 && !d4 && d8) ret = 7;
    if(d1 && !d2 && !d4 && d8) ret = 6;
    if(!d1 && d2 && !d4 && d8) ret = 5;
    if(d1 && d2 && !d4 && d8) ret = 4;
    if(!d1 && !d2 && d4 && d8) ret = 3;
    if(d1 && !d2 && d4 && d8) ret = 2;
    if(!d1 && d2 && d4 && d8) ret = 1;
    std::cout << "ThumbWheel = " << ret << std::endl;
    return ret;
}

START_ROBOT_CLASS(Robot)
