#include <ros/ros.h>
#include <librealsense2/rs.hpp>
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
#include <iostream>
#include <ctime>
#include "tku_libs/TKU_tool.h"
#include <geometry_msgs/Vector3Stamped.h>

#define PI              M_PI
#define DEG2RAD         M_PI/180
#define RAD2DEG         180/M_PI
#define MOTORDEG        0.08789     //360/4096 (deg)
#define VFOV            40.27706125    //Vertical FOV (deg) 41.0408
#define HFOV            55.0     //Horizontal FOV (deg) 52.3473  67.3801
#define AVGERRORANGLE   0.0      //Motor angle - the angle of image bottom
#define L_CAMERA        4.45         //The length of camera to Vertical motor (cm)
#define L_Calf          14.0         //踝關節～膝關節長度 (cm) 小腿
#define L_Thigh         14.0         //膝關節～髖關節長度 (cm) 大腿
#define L_BodyError     7.134625798  // L_4 = 24.95 (cm), L_5 = 3.123 (cm)
#define L_Body          25.14469385  //髖關節～垂直頭馬達長度 (cm)
#define L_FOOT          3.472        //腳底板下面～踝關節長度 (cm)
#define L_Shoes         1.4          //鞋釘厚度 (cm)
#define W_Front         7.5          //Front Width of Shoes (cm)
#define W_Behind        6.5          //Behind Width of Shoes (cm)
#define HW_Camera       1.0          //Half Width of Camera (cm)



using namespace std;
using namespace cv;

struct PointScore
{
    Point pixelpoint;
    int Score;
};

struct Coordinate
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

struct Distancefloat
{
    float x_dis;
    float y_dis;
    float dis;
};

enum class CameraType
{
    Monocular = 1,
    stereo = 2
};

struct Pixel3Dpoint{
  float x;
  float y;
  float z;
};

struct Intrinsicscolor{
  float PPX;
  float PPY;
  float Fx;
  float Fy;
};

static bool Scorecompare(PointScore &s1, PointScore &s2){
   return s1.Score < s2.Score;
}

static bool tocompare(Vec4i &s1, Vec4i &s2){
   return s1[0] > s2[0];
}

static bool sortPoints(Point &lhs,Point &rhs){
   return  (lhs.x < rhs.x) || (lhs.x==rhs.x && lhs.y < rhs.y);;
}

static bool sortPointsy(Point &lhs,Point &rhs){
   return  (lhs.y > rhs.y) || (lhs.y==rhs.y && lhs.x < rhs.x);;
}

static bool right_turn(Point &P1, Point &P2, Point &P3)
{
    return ((P3.x-P1.x)*(P2.y-P1.y) - (P3.y-P1.y)*(P2.x-P1.x)) > 0;
}

class VisionBase
{
    protected:
        vector<double> Angle_sin;
        vector<double> Angle_cos;
    public:
        cv::Mat color_buffer;
        cv::Mat depth_buffer;
        cv::Mat orign;
        cv::Mat visual_map;

        ToolInstance *tool = ToolInstance::getInstance();
    public:
        VisionBase();
        ~VisionBase();
        float camera_angle;
        float pixelDistance;
        int PixelX;
        int PixelY;
        float pixelDepth;
        float Pixelx;
        float Pixely;
        int Angle_Adjustment(int angle);
        void AngleLUT();
        int Frame_Area(int coordinate, int range);
        double CalcRobotHeight();
        
        string GetPath(string file_name);
        Point MinIntersectPoint(Vec4i line, Point A, int mindistance);
        double normalize_angle(double phi) ;
        double normalize_angle_RAD(double phi) ;
        bool LineorNot(Vec4i line);
        int B_,G_,R_;
        int _threshold,_minLineLength,_maxLineGap;
};
