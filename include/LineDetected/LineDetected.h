#include "ModelBase/ModelBase.h"
#include <cmath>
#include <algorithm>
#include <image_transport/image_transport.h>

#include "sensor_msgs/Image.h"
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include "tku_msgs/ObservationData.h"
#include "tku_msgs/LineData.h"
#include "tku_msgs/Cooridinate.h"
#include "tku_msgs/AllLineData.h"

using namespace cv;
using namespace std;

class LineDetected : public ModelBase 
{

    public:
        tku_msgs::ObservationData JustLine_Data;
        vector<Vec4i> all_lines;
        vector<Vec4i> all_lines1;
        vector<Vec4i> tmp;
        vector<Vec4i> check_lines;
        vector<Vec4i> reduce_similar_lines;
        vector<Vec4i> merge_similar_lines;

        LineDetected();
        ~LineDetected();
        //---------------------------------------
        Mat ImagePreprocessing(const Mat iframe);
        Mat orign;
        Mat imageGamma;
        Mat nobackgroud_image;
        Mat morph;
        Mat edge;
        Mat Gmask;
        Mat original_frame;
        Mat hough_frame;
        Mat merge_hough_frame;
        int Bluev;
        int Greenv;
        int Redv;
        int R_value;
        int G_value;
        int B_value;
        int Huemin;
        int Huemax;
        int Saturationmin;
        int Saturationmax;
        int Brightnessmin;
        int Brightnessmax;
        int H;
        int S;
        int V;
        int ROI_point;
        //-------------------------------------
        double dis2(coordinate a, coordinate b) ;
        //-------------------------------------
        int dir(coordinate A, coordinate B, coordinate P);
        //-------------------------------------
        double disMin(coordinate A, coordinate B, coordinate P);
        //-------------------------------------
        double MinDistance(Vec4i X,Vec4i Y);
        //-------------------------------------
        double AngleDiff(Vec4i X,Vec4i Y);
        double mX;
        double mY;
        //-------------------------------------
        coordinate Midpoint(Vec4i line);
        //-------------------------------------
        double Slope(Vec4i line);
        //-------------------------------------
        Mat ImageCanny(const Mat iframe);
        void Merge(Vec4i X,Vec4i Y);
        double XDistance;
        double YDistance;
        double theta;
        int thetaZeroY;
        Vec4i NewLine;
        Vec4i MaxLine;
        int hough_threshold;
        double hough_minLineLength;
        double hough_maxLineGap;
        void LoadHoughFile();
        void SaveHoughFile();
        int checkline(const Mat image_Enhance,const Mat canny,Vec4i line);
        //-------------------------------------
        vector<Vec4i> complement(vector<Vec4i>  all_line,Vec4i remove) ;
        //-------------------------------------
        Mat Merge_similar_line(const Mat iframe,const Mat canny_iframe,const Mat original_frame);
        //-------------------------------------
        Mat fitLineRANSAC(Mat drawing,vector<vector<Point> > allfieldpoints);
        Pixel3Dpoint deproject_pixel2point(coordinate point,float depth);
        double calculate_3D(coordinate a, coordinate b);
        timespec diff(timespec start, timespec end);


};
