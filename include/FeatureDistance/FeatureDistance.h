#include<ObjectDetected/ObjectDetected.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

using namespace std;
using namespace cv;

class FeatureDistance : public ObjectDetected
{
    private:
	    vector<unsigned int> StandPackage;
        double camera_angle_offest;
        double RobotHeight;
        double RobotWidth;
	    ToolInstance *tool;
    public:
        float camera_height;    //機器人高度
        float camera2robot_dis;

        float image_bottom_angle;
        float half_VFOV_angle;
        float half_HFOV_angle;
        float image_center_horizontal_length;

        float Vertical_Head_Angle;
        float Horizontal_Head_Angle;

        float vertical_angle;
        float horizontal_angle;

        float Roll_init;
        float Pitch_init;
        float Robot_Roll;
        float Robot_Pitch;

        int image_top_length;         //畫面最遠距離
        int image_bottom_length;      //畫面最底距離
        int image_top_width_length;       //畫面最遠寬度(一半)
        int image_bottom_width_length;       //畫面最底寬度(一半)

        bool whiteline_flag = false;
    public:
        FeatureDistance();
        ~FeatureDistance();

        void calcImageAngle(motordata Horizontal_Head,motordata Vertical_Head);
        void calcMotorAngle(int Horizontal_pos,int Vertical_pos);
        Distance measureLine(int Feature_x, int Feature_y);
        Distance measureObject(int Feature_x, int Feature_y);
        double CalcRobotHeight();
};
