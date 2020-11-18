#include <ObjectDetected/ObjectDetected.h>

ObjectDetected::ObjectDetected()
{

}
ObjectDetected::~ObjectDetected()
{

}

Mat ObjectDetected::convertTo3Channels(const Mat &binImg)
{
    Mat three_channel = Mat::zeros(binImg.rows, binImg.cols, CV_8UC3);
    vector<Mat> channels;
    for (int i = 0; i < 3; i++)
    {
        channels.push_back(binImg);
    }
    merge(channels, three_channel);
    return three_channel;
}

Mat ObjectDetected::White_Line(const cv::Mat iframe)
{
    cv::Mat threshold(iframe.rows, iframe.cols, CV_8UC3, Scalar(0, 0, 0));
    cv::Mat oframe(iframe.rows, iframe.cols, CV_8UC3, Scalar(0, 0, 0));
    //whitedis.data.clear();
    //======================threshold===================
    for (int i = 0; i < iframe.rows; i++)
    {
        for (int j = 0; j < iframe.cols; j++)
        {
            int gray_present = (iframe.data[(i * iframe.cols * 3) + (j * 3) + 0] + iframe.data[(i * iframe.cols * 3) + (j * 3) + 1] + iframe.data[(i * iframe.cols * 3) + (j * 3) + 2]) / 3;
            int gray_next = (iframe.data[(i * iframe.cols * 3) + ((j + 1) * 3) + 0] + iframe.data[(i * iframe.cols * 3) + ((j + 1) * 3) + 1] + iframe.data[(i * iframe.cols * 3) + ((j + 1) * 3) + 2]) / 3;
            int gray_next2 = (iframe.data[(i * iframe.cols * 3) + ((j + 2) * 3) + 0] + iframe.data[(i * iframe.cols * 3) + ((j + 2) * 3) + 1] + iframe.data[(i * iframe.cols * 3) + ((j + 2) * 3) + 2]) / 3;
            //int gray = 170;
            if (gray_present <= 70 && gray_next <= 70 && gray_next2 <= 70)
            //if (iframe.data[(i * iframe.cols * 3) + (j * 3) + 0] < gray && iframe.data[(i * iframe.cols * 3) + (j * 3) + 1] < gray && iframe.data[(i * iframe.cols * 3) + (j * 3) + 2] < gray) 
            {
                threshold.data[(i * threshold.cols * 3) + (j * 3) + 0] = 0;
                threshold.data[(i * threshold.cols * 3) + (j * 3) + 1] = 0;
                threshold.data[(i * threshold.cols * 3) + (j * 3) + 2] = 0;
            }
            else
            {
                threshold.data[(i * threshold.cols * 3) + (j * 3) + 0] = 255;
                threshold.data[(i * threshold.cols * 3) + (j * 3) + 1] = 255;
                threshold.data[(i * threshold.cols * 3) + (j * 3) + 2] = 255;
            }
        }
    }
    Mat element = getStructuringElement(MORPH_RECT, Size(6,6));
    Mat element_1 = getStructuringElement(MORPH_RECT, Size(9,9));
    morphologyEx(threshold, threshold, CV_MOP_OPEN, element);
    morphologyEx(threshold, threshold, CV_MOP_CLOSE, element_1);
    morphologyEx(threshold, threshold, CV_MOP_OPEN, element);
    Mat edge;
    cv::Canny(threshold, edge, 50, 150, 3);
    edge=convertTo3Channels(edge);
    //cv::imshow("edge", edge);
    //=====================draw the scan line===========
    //Mat edge = iframe.clone();
    //cv::Mat oframe(iframe.rows, iframe.cols, CV_8UC3, Scalar(0, 0, 0));
    oframe = edge.clone();
    //oframe = convertTo3Channels(oframe);
    //edge = convertTo3Channels(edge);
    int centerx = oframe.cols/2;
    int centery = oframe.rows;
    int OuterMsg = 600;
    int InnerMsg = 5;
    AngleLUT();

    Filed_feature_point.clear();
    for (int angle = 0; angle <= 180; angle = angle + 5)
    {
        //OuterMsg = centerx / (cos(angle * DEG2RAD)) + 5;
        bool find_feature_flag = false;
        int angle_be = Angle_Adjustment(angle);
        int scan_line_present = angle_be / 5;
        for (int r = InnerMsg; r <= OuterMsg; r++)
        {   
            int x_ = r * Angle_cos[angle_be];
            int y_ = r * Angle_sin[angle_be];
            int x = Frame_Area(centerx + x_, oframe.cols);
            int y = Frame_Area(centery - y_, oframe.rows);

            feature_point tmp;
            if (edge.data[(y * edge.cols + x) * 3 + 0] != 255)
            {
                if (angle_be == 0 || angle_be == 180)
                {
                    oframe.data[(y * oframe.cols + x) * 3 + 0] = 255;
                    oframe.data[(y * oframe.cols + x) * 3 + 1] = 150;
                    oframe.data[(y * oframe.cols + x) * 3 + 2] = 0;
                }
                else
                {
                    oframe.data[(y * oframe.cols + x) * 3 + 0] = 0;
                    oframe.data[(y * oframe.cols + x) * 3 + 1] = 0;
                    oframe.data[(y * oframe.cols + x) * 3 + 2] = 255;
                }
            }
            if(edge.data[(y * edge.cols + x) * 3 + 0] == 255 )
            {
                if(Filed_feature_point.size() == 0)
                {
                    tmp.x = x;
                    tmp.y = y;
                    tmp.scan_line_cnt = scan_line_present;
                    Filed_feature_point.push_back(tmp);
                    //circle(oframe, Point(x, y), 3, Scalar(0, 255, 255), 3);
                    find_feature_flag = true;
                }
                else
                {
                    if(scan_line_present == Filed_feature_point[Filed_feature_point.size() - 1].scan_line_cnt)
                    {
                        tmp.x = x;
                        tmp.y = y;
                        tmp.scan_line_cnt = scan_line_present;
                        Filed_feature_point.push_back(tmp);
                        //circle(oframe, Point(x, y), 3, Scalar(0, 255, 255), 3);
                        find_feature_flag = true;
                    }
                    else
                    {
                        tmp.x = x;
                        tmp.y = y;
                        tmp.scan_line_cnt = scan_line_present;
                        Filed_feature_point.push_back(tmp);
                        //circle(oframe, Point(x, y), 3, Scalar(0, 255, 255), 3);
                        find_feature_flag = true;
                    }
                }
                /*if(Filed_feature_point.size() == 13 || Filed_feature_point.size() == 18 || Filed_feature_point.size() == 23)
                {
                    circle(oframe, Point(x, y), 3, Scalar(255, 255, 255), 2);
                }
                else
                {
                    circle(oframe, Point(x, y), 3, Scalar(255, 0, 0), 1);
                }*/
            }
            if(x == 0 || x == (oframe.cols - 1) || y == 0)
            {
                if(!find_feature_flag)
                {
                    tmp.x = -1;
                    tmp.y = -1;
                    tmp.scan_line_cnt = angle_be / 10;
                    Filed_feature_point.push_back(tmp);
                }
                break;
            }
        }
    }
    ROS_INFO("Filed_feature_point.size = %d",Filed_feature_point.size());
    ROS_INFO("Filed_feature_point.scan_line_angle = %d",Filed_feature_point[Filed_feature_point.size() - 1].scan_line_cnt);
    return oframe;
}

Mat ObjectDetected::FindObject(const cv::Mat iframe)
{
    //Mat gray(320, 240, CV_8UC3, Scalar(0, 0, 0));
    Mat gray;
    Mat oframe = iframe.clone();
    //resize(iframe, gray, cv::Size(640, 480));
    //resize(oframe, oframe, cv::Size(640, 480));

    cvtColor(iframe, gray, COLOR_BGR2GRAY);

    //cascader2soccer.detectMultiScale(gray, soccer_data, 1.1, 10, 0, Size(20, 20));
    //cascader2goal.detectMultiScale(gray, goal_data, 1.1, 10, 0, Size(20, 20));
    ROS_INFO("11");
    for (size_t t = 0; t < soccer_data.size(); t++)
	{
		rectangle(oframe, soccer_data[t], Scalar(255, 0, 0), 2, 8, 0);
	}
    for (size_t t = 0; t < goal_data.size(); t++)
	{
		rectangle(oframe, goal_data[t], Scalar(0, 0, 255), 2, 8, 0);
	}

    resize(oframe, oframe, cv::Size(320, 240));
    return oframe;
}
