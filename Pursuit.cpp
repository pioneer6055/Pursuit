#include "Pursuit.h"

Pursuit::Pursuit()
{ }

//call repeatedly to execute path following of waypoints
//must add waypoints and call initialize first
Pursuit_Output Pursuit::Execute(double dist, double heading, double elapsedSeconds)
{
    Pursuit_Output Pout;
    static double lastLeft = 0.0;
    static double lastRight = 0.0;
    
    if(Pursuit_Initialized)
    {
        //run until robot reaches final target
        if(Pursuit_Index < PursuitPath.size())
        {
            //calculate distance traveled for this target
            //Pursuit_Distance = dist - Pursuit_DistanceOffset;
            static double lastDist = 0.0;
            double deltaDist = fabs(dist - lastDist);
            lastDist = dist;
            if(fabs(Normalize(Pursuit_Bearing - Pursuit_Heading)) < 90)
                Pursuit_Distance += deltaDist;
            //get bearing to target
            Pursuit_Bearing = GetBearing();

            //once target is reached, increment target
            if(Pursuit_Distance - Pursuit_DistanceOffset > (PURSUIT_TARGET_PERCENT * Pursuit_TargetDistance))
            {
                if(PURSUIT_DEBUG_MODE) 
                {
                    std::cout << std::fixed << std::setprecision(1) << "[Waypoint " << Pursuit_Index+1 << " Done]";
                    std::cout << "  Length= " << Pursuit_Distance - Pursuit_DistanceOffset;
                    std::cout << "  Bearing= " << Pursuit_Bearing;
                    std::cout << "  Heading= " << Pursuit_Heading;
                    std::cout << "  Left= " << lastLeft;
                    std::cout << "  Right= " << lastRight << std::endl;
                }
                Pursuit_WayPointStart.x = Pursuit_CurrentPoint.x;
                Pursuit_WayPointStart.y = Pursuit_CurrentPoint.y;
                Pursuit_Index++;
                Pursuit_Bearing = GetBearing();
                Pursuit_TargetDistance = Pursuit_WayPointStart.distanceTo(PursuitPath[Pursuit_Index]);
                Pursuit_DistanceOffset += Pursuit_Distance;
                if(PURSUIT_DEBUG_MODE)
                {
                    if(Pursuit_Index < PursuitPath.size()) 
                    {
                        std::cout << std::fixed << std::setprecision(1) << "[Waypoint " << Pursuit_Index+1 << " Started]";
                        std::cout << "  Point.X= " << Pursuit_WayPointStart.x;
                        std::cout << "  Point.Y= " << Pursuit_WayPointStart.y;
                        std::cout << "  TargetDist= " << Pursuit_TargetDistance;
                        std::cout << "  DistOffset= " << Pursuit_DistanceOffset;
                        std::cout << "  Bearing= " << Pursuit_Bearing;
                        std::cout << "  Heading= " << Pursuit_Heading << std::endl;
                    }
                }
            }

            //calculate heading correction
            Pursuit_Heading = heading;
            double headingError = Normalize(Pursuit_Bearing - Pursuit_Heading);
            //calculate adaptive gain (the farther from target - the bigger the gain)
            double turnGain = headingError / PURSUIT_TURN_GAIN;

            //calculate speed
            Pursuit_Speed = 0.0;
            double distPercent = (Pursuit_Distance - Pursuit_DistanceOffset)/Pursuit_TargetDistance;
            double speedMax = PURSUIT_MAX_SPEED * (1-(fabs(headingError)/90));
            Pursuit_Speed = speedMax;
            //ramp up on first third of first leg
            if(Pursuit_Index == 0 && distPercent < 0.33) Pursuit_Speed = (distPercent * 3) * speedMax;
            //ramp down on last third of last leg
            if(Pursuit_Index == PursuitPath.size() - 1 && distPercent >= 0.66) Pursuit_Speed = ((1 - distPercent) * 3) * speedMax;
            //clamp to min speed
            if(Pursuit_Speed < PURSUIT_MIN_SPEED) Pursuit_Speed = PURSUIT_MIN_SPEED;
            //clamp to max speed
            if(Pursuit_Speed > PURSUIT_MAX_SPEED) Pursuit_Speed = PURSUIT_MAX_SPEED;
            //force forward
            Pursuit_Speed = fabs(Pursuit_Speed) * -1;

            //add speed + turning gain to get motor output
            Pout.LeftMotorOut = Pursuit_Speed - turnGain;
            Pout.RightMotorOut = Pursuit_Speed + turnGain;
            Pout.PathComplete = false;
            if(elapsedSeconds - Pursuit_Elapsed > 0.25)
            {
                if(PURSUIT_DEBUG_MODE) printf("[%d] L=%.2f R=%.2f Pos.X= %.1f  Pos.Y=%.1f Brg=%.1f Hdg=%.1f D=%.1f DT=%.1f\n",Pursuit_Index+1,Pout.LeftMotorOut,Pout.RightMotorOut,Pursuit_CurrentPoint.x,Pursuit_CurrentPoint.y,Pursuit_Bearing,Pursuit_Heading,Pursuit_Distance,Pursuit_TargetDistance);
                {
                    std::cout << std::fixed << std::setprecision(1) << "[" << Pursuit_Index+1 << "]";
                    std::cout << "  Length= " << Pursuit_Distance - Pursuit_DistanceOffset;
                    std::cout << "  Bearing= " << Pursuit_Bearing;
                    std::cout << "  Heading= " << Pursuit_Heading;
                    std::cout << "  Left= " << lastLeft;
                    std::cout << "  Right= " << lastRight << std::endl;
                }
                Pursuit_Elapsed = elapsedSeconds;
            }
        }
        else
        {
            Pout.PathComplete = true;
            Pursuit_Speed = 0.0;
            Pout.LeftMotorOut = 0.0;
            Pout.RightMotorOut = 0.0;
        }
        lastLeft = Pout.LeftMotorOut;
        lastRight = Pout.RightMotorOut;
    }
    else printf("PURSUIT MUST BE INITIALIZED\n");
    return Pout;
}

//call before adding waypoints
void Pursuit::ClearPath()
{
    PursuitPath.clear();
}

//add waypoints before calling initialize
void Pursuit::AddWaypoint(double x, double y)
{
    Vector2D v;
    v.x = x;
    v.y = y;
    PursuitPath.push_back(v);
}

//call initialize after adding waypoints and before calling execute
void Pursuit::Initialize(double elapsedSeconds)
{
    Pursuit_Initialized = false;
    Pursuit_Elapsed = elapsedSeconds;
    Pursuit_Index = 0;
    //set initial location and speed
    Pursuit_WayPointStart.x = 0;
    Pursuit_WayPointStart.y = 0;
    Pursuit_Speed = 0.0;
    Pursuit_Distance = 0.0;
    Pursuit_DistanceOffset = 0.0;
    Pursuit_Bearing = 0.0;
    Pursuit_Heading = 0.0;
    Pursuit_TargetDistance = 0.0;
    if(PursuitPath.size() >= 2)
    {
        Pursuit_Initialized = true;
        //get bearing to first target
        Pursuit_Bearing = GetBearing();
        //get distance to first target
        Pursuit_TargetDistance = Pursuit_WayPointStart.distanceTo(PursuitPath[Pursuit_Index]);
        if(PURSUIT_DEBUG_MODE) 
        {
            std::cout << std::fixed << std::setprecision(1) << "[Initial] WaypointCount " << PursuitPath.size();
            std::cout << "  Bearing= " << Pursuit_Bearing;
            std::cout << "  Heading= " << Pursuit_Heading;
            std::cout << "  TargetDist= " << Pursuit_TargetDistance << std::endl;
        }
    }
    else printf("PURSUIT PATH REQUIRES AT LEAST 2 WAYPOINTS\n");
}

double Pursuit::Normalize(double heading)
{
    if(heading > 180) return heading -= 360;
    else if(heading < -180) return heading += 360;
    else return heading;
}

double r2d(double rad)
{
    return rad * 57.29577951;
}

double d2r(double deg)
{
    return deg * 0.01745329;
}

Vector2D Pursuit::GetCurrentPosition()
{
    Vector2D rtn;

    rtn.x = Pursuit_WayPointStart.x + (Pursuit_Distance - Pursuit_DistanceOffset) * cos(d2r(Pursuit_Heading));
    rtn.y = Pursuit_WayPointStart.y + (Pursuit_Distance - Pursuit_DistanceOffset) * sin(d2r(Pursuit_Heading));
    return(rtn);
}

double Pursuit::GetBearing()
{
    Pursuit_CurrentPoint.x = Pursuit_WayPointStart.x + (Pursuit_Distance - Pursuit_DistanceOffset) * cos(d2r(Pursuit_Heading));
    Pursuit_CurrentPoint.y = Pursuit_WayPointStart.y + (Pursuit_Distance - Pursuit_DistanceOffset) * sin(d2r(Pursuit_Heading));
    Vector2D Target = PursuitPath[Pursuit_Index];
    double deg = 0;
    try
    {
        deg = r2d(atan2(Target.y - Pursuit_CurrentPoint.y,Target.x - Pursuit_CurrentPoint.x));
    }
    catch(std::exception& ex)
    {}
    if(deg < 0) deg = 360+deg;
    if(deg >=360) deg = deg-360;
    return deg;
}

Vector2D::Vector2D()
{
    x = 0;
    y = 0;
}

Vector2D::Vector2D( float xInit, float yInit )
{
    x = xInit;
    y = yInit;
}

float Vector2D::getLength()
{
    return sqrt(x*x+y*y);
}

float Vector2D::distanceTo( Vector2D v )
{
    Vector2D distanceVector = Vector2D(v.x - x, v.y - y );
    return distanceVector.getLength();
}
