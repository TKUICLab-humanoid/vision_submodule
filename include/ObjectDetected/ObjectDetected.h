#include<VisionBase/VisionBase.h>

using namespace std;
using namespace cv;


class ObjectDetected : public VisionBase
{
    private:

    public:
        vector<feature_point> Filed_feature_point;
        vector<Rect> soccer_data;
        vector<Rect> goal_data;

        CascadeClassifier cascader2soccer;
        CascadeClassifier cascader2goal;
    public:
        ObjectDetected();
        ~ObjectDetected();

        Mat convertTo3Channels(const Mat &binImg);
        Mat White_Line(const cv::Mat iframe);
        Mat FindObject(const cv::Mat iframe);
};