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