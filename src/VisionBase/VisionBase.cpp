#include <VisionBase/VisionBase.h>

VisionBase::VisionBase()
{

}
VisionBase::~VisionBase()
{
    
}

int VisionBase::Angle_Adjustment(int angle)
{
    if (angle < 0)
        return angle + 360;
    else if (angle >= 360)
        return angle - 360;
    else
        return angle;
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

Point VisionBase::MinIntersectPoint(Vec4i line, Point A, double mindistance)
{
    Point start = Point(line[0],line[1]);
    Point end = Point(line[2],line[3]);
    int x1 = start.x;
    int y1 = start.y;
    int x2 = end.x;
    int y2 = end.y;
    float para_a = (float)(y1-y2)/(x1-x2);
    float para_b = (float)(x1*y2-x2*y1)/(x1-x2);
    Point minIntersectPoint = Point(0,0);
    for(int i = 0; i < abs(x2-x1); i++ )
    {
        float x = 0;
        float y = 0;
        float dis = 0;
        if(x2<x1)
        {
            x = x2 + i;
        }else{
            x = x1 + i;
        }
        y = x*para_a+para_b;
        dis = sqrt(pow(A.x-x,2)+pow(A.y-y,2));
        if(dis == mindistance)
        {
            minIntersectPoint = Point(x,y);
        }
    }
    // ROS_INFO("MinIntersectPoint = (%d,%d)",minIntersectPoint.x,minIntersectPoint.y);
    return minIntersectPoint;
}