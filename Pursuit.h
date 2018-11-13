#pragma once

#include <vector>
#include <iostream>
#include <iomanip>
#include "math.h"


struct Pursuit_Output
{
    double LeftMotorOut;
    double RightMotorOut;
    bool PathComplete;
};

class Vector2D 
{
    public :
    float x, y;
    Vector2D();
    Vector2D(float xInit, float yInit);
    float getLength();
    float distanceTo(Vector2D v);
};

class Pursuit
{
private:
    double Normalize(double heading);
    Vector2D GetCurrentPosition();
    double GetBearing();
    double GetBearing2();

    std::vector <Vector2D> PursuitPath;
    Vector2D Pursuit_WayPointStart;
    Vector2D Pursuit_CurrentPoint;
    bool PURSUIT_DEBUG_MODE = true;
public:
    bool     Pursuit_Initialized = false;
    double   Pursuit_Distance = 0.0;        //distance traveled toward current waypoint target
    double   Pursuit_DistanceOffset = 0.0;
    double   Pursuit_TargetDistance = 0.0;  //distance to reach current waypoint target
    double   Pursuit_Bearing = 0.0;         //direction to current waypoint target
    double   Pursuit_Heading = 0.0;         //direction robot is currently pointing
    unsigned Pursuit_Index = 0;             //index into waypoint array
    double   Pursuit_Speed = 0.0;           //current speed
    double   Pursuit_Elapsed = 0.0;         //last elapsed seconds value
    double   PURSUIT_MAX_SPEED = 1;
    double   PURSUIT_MIN_SPEED = 0.35;
    double   PURSUIT_TURN_GAIN = 500;       //reduce to slow down turns, increase to speed up turns
    double   PURSUIT_TARGET_PERCENT = 0.85; //how much of current waypoint distance must be completed before starting next waypoint

    Pursuit();
    Pursuit_Output Execute(double dist, double heading, double elapsedSeconds);
    void ClearPath();
    //x+ is forward, y+ is to the right
    void AddWaypoint(double x, double y);
    void Initialize(double elapsedSeconds);
};
