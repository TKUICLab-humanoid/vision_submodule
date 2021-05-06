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
#define CAMERA_HEIGHT   3.925        //The height of camera to Vertical motor  (cm)
#define SLEG_L          14        //踝關節～膝關節長度  (cm)
#define BLEG_L          14        //膝關節～髖關節長度  (cm)
#define BODY_L          22.892        //髖關節～垂直頭馬達長度  (cm)
#define FOOT_H          2.556        //腳底板～踝關節長度  (cm)
#define MOTORDEG        0.08789     //360/4096 (deg)
#define VFOV            40.27706125    //Vertical FOV (deg) 41.0408
#define HFOV            48.3473     //Horizontal FOV (deg) 52.3473  67.3801
#define AVGERRORANGLE   5.3      //Motor angle - the angle of image bottom
#define DEG2RAD         M_PI/180
#define RAD2DEG        180/M_PI

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

        int Angle_Adjustment(int angle);
        void AngleLUT();
        int Frame_Area(int coordinate, int range);
        double CalcRobotHeight();
        
        string GetPath(string file_name);
        Point MinIntersectPoint(Vec4i line, Point A, int mindistance);
        bool LineorNot(Vec4i line);

};
