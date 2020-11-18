#include "ModelBase/ModelBase.h"
#include <cmath>
#include <algorithm>

using namespace cv;
using namespace std;

struct LineINF
{
    Distance middlepoint;
    double Line_length;
    double Line_theta;   
};


class LineDetected : public ModelBase 
{
    public:
        int frame_rows;
        int frame_cols;
        vector<Vec4i> all_lines;
        vector<Vec4i> tmp;
        vector<Vec4i> check_lines;
        vector<Vec4i> reduce_similar_lines;
        vector<Vec4i> merge_similar_lines;
        LineDetected();
        ~LineDetected();
        //---------------------------------------
        Mat ImagePreprocessing(const Mat iframe);
        Mat ImageCanny(const Mat iframe);
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
        int thetaZeroX;
        Vec4i NewLine;
        Vec4i MaxLine;
        int hough_threshold;
        double hough_minLineLength;
        double hough_maxLineGap;
        //-------------------------------------
        vector<Vec4i> complement(vector<Vec4i>  all_line,Vec4i remove) ;
        //-------------------------------------
        int checkline(Mat image_Enhance,Mat canny,Vec4i line);
        //-------------------------------------
        Mat Merge_similar_line(const Mat iframe,const Mat canny_iframe,const Mat original_frame);
        //-------------------------------------
        Mat nocheckline(const Mat iframe,const Mat canny_iframe,const Mat original_frame);
};
