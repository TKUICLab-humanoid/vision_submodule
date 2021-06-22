#include <VisionBase/VisionBase.h>

VisionBase::VisionBase()
{

}
VisionBase::~VisionBase()
{
    
}

int VisionBase::Angle_Adjustment(int angle)
{
    if (angle < -360)
        return angle + 360;
    else if (angle >= 360)
        return angle - 360;
    else
        return angle;
}

double VisionBase::normalize_angle(double phi) 
{
    //Normalize phi to be between -pi and pi
    while(phi > 180.0) {
        phi = phi - 2.0 * 180.0;
    }

    while(phi < -180.0) {
        phi = phi + 2.0 * 180.0;
    }
    return phi;
}

double VisionBase::normalize_angle_RAD(double phi) 
{
    //Normalize phi to be between 0 and pi
    phi = (90.0 - phi) * DEG2RAD;

    return phi;
}

void VisionBase::AngleLUT()
{
    double ang_PI;
    for (int ang = 0; ang <= 360; ang++)
    {
        ang_PI = ang * PI / 180;
        Angle_sin.push_back(sin(ang_PI));
        Angle_cos.push_back(cos(ang_PI));
    }
}

bool VisionBase::LineorNot(Vec4i line)
{
    Point Xstart = Point(line[0],line[1]);
    Point Xend = Point(line[2],line[3]);
    if(Xstart.x-Xend.x == 0 && Xstart.y-Xend.y == 0)
    {
        return 0;
    }else{
        return 1;
    }
}

int VisionBase::Frame_Area(int coordinate, int range)
{
    if (coordinate < 0)
        coordinate = 0;
    else if (coordinate >= range)
        coordinate = range - 1;
    return coordinate;
}

string VisionBase::GetPath(string file_name)
{
    string path;
    file_name = "/" + file_name;
    path = tool->getPackagePath("vision") + file_name;

    return path;
}

Point VisionBase::MinIntersectPoint(Vec4i line, Point A, int mindistance)
{
    Point start = Point(line[0],line[1]);
    Point end = Point(line[2],line[3]);
    int x1 = start.x;
    int y1 = start.y;
    int x2 = end.x;
    int y2 = end.y;
    float para_a = (float)((float)(y1-y2)/(float)(x1-x2));
    float para_b = (float)((float)((x1*y2)-(x2*y1))/(float)(x1-x2));
    Point minIntersectPoint = Point(0,0);
    for(int i = 0; i <= abs(x2-x1); i++ )
    {
        float x = 0;
        float y = 0;
        int dis = 0;
        if(x2<x1)
        {
            x = x2 + i;
        }else{
            x = x1 + i;
        }
        if(x>639||x<0)
        {
            continue;
        }
        y = (x*para_a)+para_b;
        if(y>479||y<0)
        {
            continue;
        }
        dis = int(round(sqrt(pow(A.x-x,2)+pow(A.y-y,2))));
        if(dis >= mindistance -1 && dis <= mindistance +1)
        {
            minIntersectPoint = Point(x,y);
        }
    }
    // ROS_INFO("MinIntersectPoint = (%d,%d)",minIntersectPoint.x,minIntersectPoint.y);
    return minIntersectPoint;
}