#include "ModelBase/ModelBase.h"
#include <cmath>
#include <algorithm>

using namespace cv;
using namespace std;


class LineDetected : public ModelBase 
{
    public:
        int ErRoR;
        
        vector<Vec4i> all_lines;
        vector<Vec4i> tmp;
        vector<Vec4i> reduce_similar_lines;
        vector<Vec4i> merge_similar_lines;
        vector<int> ROI_point_tmp;
        LineDetected();
        ~LineDetected();
        //---------------------------------------
        Mat ImagePreprocessing(const Mat iframe);
        Mat GreenField;
        Mat img_hsv;
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
        void Merge(Vec4i X,Vec4i Y);
        double XDistance;
        double YDistance;
        double theta;
        int thetaZeroY;
        Vec4i NewLine;
        //-------------------------------------
        vector<Vec4i> complement(vector<Vec4i>  all_line,Vec4i remove) ;
        //-------------------------------------
        Mat Merge_similar_line(const Mat canny_iframe,const Mat original_frame);
        //-------------------------------------
        int getHistograph(const Mat grayImage);
        Mat hist;
        int i_diff;
        int temp_max;
        int i_max;
        int temp_min;
        int i_min;
};
