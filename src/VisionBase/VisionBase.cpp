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

bool VisionBase::onImage(int x, int y)
{
    return x >= 0 && x < 640 && y >= 0 && y < 480;
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

    int x1 = 0;
    int y1 = 0;
    int x2 = 0;
    int y2 = 0;
    if( start.x > end.x )
    {
        x1 = end.x;
        y1 = end.y;
        x2 = start.x;
        y2 = start.y;
    }else{
        x1 = start.x;
        y1 = start.y;
        x2 = end.x;
        y2 = end.y;
    }
    // float para_a = (float)(y1-y2)/(x1-x2);
    // float para_b = (float)(x1*y2-x2*y1)/(x1-x2);
    Point minIntersectPoint = Point(0,0);
    double min_value = 0.0;
    
    Point2f Linelength = Point(x2,y2)-Point(x1,y1);
    int Linestep = ceil(max(fabs(Linelength.x),fabs(Linelength.y)));
    Point2f Linegap = Linelength / Linestep;
    Point2f p1 = Point(x1,y1);
    // ROS_INFO(" Linelength %d %d",Linelength.x,Linelength.y);
    // ROS_INFO("Linestep %d",Linestep);
    // ROS_INFO("Linegap %f %f",Linegap.x,Linegap.y);
    // ROS_INFO("%d %d %d %d",x1,y1,x2,y2);

    for(int i = 0; i < Linestep; i++ )
    {
        int x = round(p1.x);
        int y = round(p1.y);
        // ROS_INFO("%d %d",x,y);
        if(!onImage(x, y))
        {
            p1 = p1 + Linegap ;
            continue;
        } 
        double dis = sqrt(pow(320-x,2)+pow(478-y,2));
        if( i == 0)
        {
            min_value = dis;
            minIntersectPoint = Point(x,y);
        }
        else if(min_value > dis)
        {
            minIntersectPoint = Point(x,y);
            min_value = dis;
        }
        p1 = p1 + Linegap;
        
        // ROS_INFO(" x = %d y = %d dis = %f min_value = %f,mindistance = %f",x,y,dis ,min_value,mindistance);
    }
    // ROS_INFO("MinIntersectPoint = (%d,%d)",minIntersectPoint.x,minIntersectPoint.y);
    return minIntersectPoint;
}