#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/flann.hpp>
#include <cv_bridge/cv_bridge.h>
#include <highgui.h>
#include <sys/time.h>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <string.h>

#include "tku_libs/TKU_tool.h"

#define PI              M_PI
#define DEG2RAD         M_PI/180
#define MOTORDEG        0.087890625  //360/4096 (deg)
#define L_CAMERA        4.45         //The length of camera to Vertical motor (cm)
#define L_Calf          14.0         //踝關節～膝關節長度 (cm) 小腿
#define L_Thigh         14.0         //膝關節～髖關節長度 (cm) 大腿
#define L_BodyError     7.134625798  // L_4 = 24.95 (cm), L_5 = 3.123 (cm)
#define L_Body          25.14469385  //髖關節～垂直頭馬達長度 (cm)
#define L_FOOT          3.472        //腳底板下面～踝關節長度 (cm)
#define L_Shoes         1.4          //鞋釘厚度 (cm)
#define W_Front         7.5          //Front Width of Shoes (cm)
#define W_Behind        6.5          //Behind Width of Shoes (cm)
#define AVGERRORANGLE   0            //Motor angle - the angle of image bottom 5.3
#define VFOV            41.0         //Vertical FOV (deg)   41.0 depth camera: 57 my messure: 39.4 //60.0
#define HFOV            55.0         //Horizontal FOV (deg) 64.0 depth camera: 86 my messure: 51.3
#define HW_Camera       1.0          //Half Width of Camera (cm)
#define R_Ball          6.5          //Radius of ball (cm)
#define R_Robot         11.0         //Radius of robot (cm)

using namespace std;
using namespace cv;

struct coordinate
{
    int X;
    int Y;
};

struct feature_point
{
    int x;
    int y;
    int scan_line_cnt;
};

struct  motordata
{
    int pos;
    int speed;
};

struct Distance
{
    int x_dis;
    int y_dis;
    int dis;
};

class VisionBase
{
    protected:
        vector<double> Angle_sin;
        vector<double> Angle_cos;
    public:
        cv::Mat depth_buffer;
        cv::Mat color_buffer;
        cv::Mat orign;
        cv::Mat visual_map;

        ToolInstance *tool = ToolInstance::getInstance();
    public:
        VisionBase();
        ~VisionBase();

        int Angle_Adjustment(int angle);
        void AngleLUT();
        int Frame_Area(int coordinate, int range);
        double CalcRobotHeight();
        
        string GetPath(string file_name);
};
