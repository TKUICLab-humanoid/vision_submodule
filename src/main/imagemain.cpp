#include "main/imagemain.h"

Vision_main::Vision_main(ros::NodeHandle &nh)
{
    this->nh = &nh;
    image_transport::ImageTransport it(nh);
    
    // for realsense D435i
    Imagesource_subscriber = nh.subscribe("/camera/color/image_raw", 1, &Vision_main::GetImagesourceFunction,this);
    Depthimage_subscriber = nh.subscribe("/camera/aligned_depth_to_color/image_raw", 1, &Vision_main::DepthCallback,this);
    
    // Imagesource_subscriber = nh.subscribe("/usb_cam/image_raw", 1, &Vision_main::GetImagesourceFunction,this);
    HeadAngle_subscriber = nh.subscribe("/package/HeadMotor", 10, &Vision_main::HeadAngleFunction,this);
    IMUData_Subscriber = nh.subscribe("/package/sensorpackage", 1, &Vision_main::GetIMUDataFunction,this);

    //--------------BGR---------------
    BGRValue_subscriber = nh.subscribe("BGRValue_Topic", 1000, &Vision_main::ChangeBGRValue,this);
    BGR_service = nh.advertiseService("LoadBGRInfo", &Vision_main::LoadBGRInfo,this);
    
    //--------------Hough-------------
    HoughValue_subscriber = nh.subscribe("HoughValue_Topic", 1000, &Vision_main::ChangeHoughValue,this);
    Hough_service = nh.advertiseService("LoadHoughInfo", &Vision_main::LoadHoughInfo,this);
    
    //--------------------------------
    ObservationData_Publisher = nh.advertise<tku_msgs::ObservationData>("/vision/observation_data", 10);
    ImageLengthData_Publisher = nh.advertise<tku_msgs::ImageLengthData>("/vision/imagelength_data", 10);
    SoccerData_Publisher = nh.advertise<tku_msgs::SoccerDataList>("/vision/soccer_topic",10);

    Object_Frame_Publisher = it.advertise("/vision/object_image", 1);
    mask_Frame_Publisher = it.advertise("/vision/mask_image", 1);
    Monitor_Frame_Publisher = it.advertise("/vision/monitor_image", 1);
    Measure_Frame_Publisher = it.advertise("/vision/measure_image", 1);
    Gamma_Frame_Publisher = it.advertise("/vision/Gamma_image", 1);
    MerHough_Publisher = it.advertise("/vision/merhough_image", 1);
    
    pitch_pre = 0.0;
    roll_pre = 0.0;
    B_ = 0;
    G_ = 0;
    R_ = 0;

}
Vision_main::~Vision_main()
{
    
}
void Vision_main::DepthCallback(const sensor_msgs::ImageConstPtr& depth_img) 
{
    cv_bridge::CvImagePtr cv_depth_ptr;
    try
    {
      cv_depth_ptr = cv_bridge::toCvCopy(depth_img, sensor_msgs::image_encodings::TYPE_16UC1);
      depth_buffer = cv_depth_ptr->image;
      //resize(depth_buffer, depth_buffer, cv::Size(320, 240));
      //imshow("depth_buffer",depth_buffer);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("DepthCallback cv_bridge exception: %s", e.what());
      return;
    }
    // cout << "image data(240, 320): " << (depth_buffer.at<uint16_t>(240, 320))*0.1 <<" cm" << endl;//獲取圖像坐標240,320的深度值,單位是公分
}
void Vision_main::ChangeBGRValue(const tku_msgs::BGRValue& msg)
{
    Model_Base->BGRColorRange->BuValue = (float)msg.BValue;
    Model_Base->BGRColorRange->GrValue = (float)msg.GValue;
    Model_Base->BGRColorRange->ReValue = (float)msg.RValue;
    Model_Base->SaveBGRFile();
}

void Vision_main::LoadBGRValue()
{
    ROS_INFO("LoadBGRValue");
    Model_Base->LoadBGRFile();
    B_ = Model_Base->BGRColorRange->BuValue;
    G_ = Model_Base->BGRColorRange->GrValue;
    R_ = Model_Base->BGRColorRange->ReValue;
    ROS_INFO("B = %f G = %f R = %f",B_,G_,R_);
}

bool Vision_main::LoadBGRInfo(tku_msgs::BGRInfo::Request &req, tku_msgs::BGRInfo::Response &res)
{
    res.BValue = B_;
    res.GValue = G_;
    res.RValue = R_;
    return true;
    
}

void Vision_main::ChangeHoughValue(const tku_msgs::HoughValue& msg)
{
    hough_threshold = (float)msg.Hough_threshold;
    hough_minLineLength = (float)msg.Hough_minLineLength;
    hough_maxLineGap = (float)msg.Hough_maxLineGap;
    SaveHoughFile();
}

void Vision_main::LoadHoughValue()
{
    LoadHoughFile();
    threshold_ = hough_threshold;
    minLineLength_ = hough_minLineLength;
    maxLineGap_ = hough_maxLineGap;
    ROS_INFO("threshold_ = %d minLineLength_ = %d maxLineGap_ = %d",threshold_,minLineLength_,maxLineGap_);
}
bool Vision_main::LoadHoughInfo(tku_msgs::HoughInfo::Request &req, tku_msgs::HoughInfo::Response &res)
{
    res.Hough_threshold = threshold_;
    res.Hough_minLineLength = minLineLength_;
    res.Hough_maxLineGap = maxLineGap_;
    return true;
}

void Vision_main::GetImagesourceFunction(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      color_buffer = cv_ptr->image;
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("GetImagesourceFunction cv_bridge exception: %s", e.what());
      return;
    }
}
void Vision_main::HeadAngleFunction(const tku_msgs::HeadPackage &msg)
{
    if(msg.ID == 1)
    {
        Horizontal_Head.pos = msg.Position;
        Horizontal_Head.speed = msg.Speed;
    }
    else
    {
        Vertical_Head.pos = msg.Position;
        Vertical_Head.speed = msg.Speed;
    }
    ROS_INFO("Vertical_Head_ver = %d",Vertical_Head.pos);
    ROS_INFO("Horizontal_Head_ver = %d",Horizontal_Head.pos);
    calcImageAngle(Horizontal_Head,Vertical_Head);
    ImageLengthData.focus = camera2robot_dis;
    ImageLengthData.top = image_top_length;
    ImageLengthData.bottom = image_bottom_length;
    ImageLengthData.top_width = image_top_width_length;
    ImageLengthData.bottom_width = image_bottom_width_length;
    ImageLengthData.horizontal_head_angle = Horizontal_Head_Angle;
    ImageLengthData_Publisher.publish(ImageLengthData);
    //ROS_INFO("ImageLengthData.focus = %d",ImageLengthData.focus);
    //ROS_INFO("ImageLengthData.top = %d",ImageLengthData.top);
    //ROS_INFO("ImageLengthData.bottom = %d",ImageLengthData.bottom);
    //ROS_INFO("ImageLengthData.top_width = %d",ImageLengthData.top_width);
    //ROS_INFO("ImageLengthData.bottom_width = %d",ImageLengthData.bottom_width);
}
void Vision_main::GetIMUDataFunction(const tku_msgs::SensorPackage &msg)
{
    if(!msg.IMUData.empty())
    {
        if(msg.IMUData[0] == 0.0 && msg.IMUData[1] == 0.0 && msg.IMUData[2] == 0.0)
        {
            Robot_Pitch = pitch_pre;
            Robot_Roll = roll_pre;
        }
        else
        {
            Robot_Roll = msg.IMUData[0];
            Robot_Pitch = msg.IMUData[1];
            roll_pre = Robot_Roll;
            pitch_pre = Robot_Pitch;
        }
        //ROS_INFO("Robot_Roll = %f",Robot_Roll);
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "vision");

    ros::NodeHandle nh;
    ROS_INFO("strategy_init");
	Vision_main vision_main(nh);

    ros::spinOnce();

    ros::Rate loop_rate(60);
    ROS_INFO("strategy_init");
    vision_main.strategy_init();

   while (nh.ok())
    {
        vision_main.strategy_main();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

void Vision_main::strategy_init()
{
    ROS_INFO("strategy_init");
    CalcRobotHeight();
    if(!cascader2soccer.load(GetPath("cascade2soccer.xml")))
    {
        ROS_INFO("could not load cascader2soccer.xml");

    }
    if(!cascader2goal.load(GetPath("cascade2goal.xml")))
    {
        ROS_INFO("could not load cascader2goal.xml");

    }
    LoadBGRValue();
}

void Vision_main::strategy_main()
{
    if(!color_buffer.empty())
    {
        Mat oframe = color_buffer.clone();
        line(oframe, Point(oframe.cols/2,oframe.rows), Point(oframe.cols/2,0), Scalar(0,0,255), 1);
        line(oframe, Point(0,oframe.rows/2), Point(oframe.cols,oframe.rows/2), Scalar(0,0,255), 1);
        resize(oframe, oframe, cv::Size(320, 240));
        //imshow("oframe",oframe);

        Mat imagePreprocessing = ImagePreprocessing(color_buffer);
        edge = ImageCanny(imagePreprocessing);
        
        Mat aftercanny = edge.clone();
        merge_hough_frame = Merge_similar_line(imagePreprocessing,aftercanny,color_buffer);
	    // imshow("line",line);
        Observation_Data.landmark = JustLine_Data.landmark;
        cv::Mat Object_frame = FindObject(color_buffer);
        //imshow("Object_frame", Object_frame);

        cv::Size dst_sz(color_buffer.cols,color_buffer.rows);
        cv::Point2f center(dst_sz.height/2,dst_sz.width/2);

        cv::Mat rot_mat = cv::getRotationMatrix2D(center, -1 * (Robot_Roll), 1.0);

        cv::Mat dst;
        cv::warpAffine(color_buffer, dst, rot_mat, dst_sz);
        //imshow("dst",dst);

        cv::Mat monitor = White_Line(aftercanny);

        calcImageAngle(Horizontal_Head,Vertical_Head);
        ImageLengthData.focus = camera2robot_dis;
        ImageLengthData.top = image_top_length;
        ImageLengthData.bottom = image_bottom_length;
        ImageLengthData.top_width = image_top_width_length;
        ImageLengthData.bottom_width = image_bottom_width_length;
        ImageLengthData.horizontal_head_angle = Horizontal_Head_Angle;
        ImageLengthData_Publisher.publish(ImageLengthData);
        //ROS_INFO("ImageLengthData.focus = %d",ImageLengthData.focus);
        /*ROS_INFO("ImageLengthData.top = %d",ImageLengthData.top);
        ROS_INFO("ImageLengthData.bottom = %d",ImageLengthData.bottom);
        ROS_INFO("ImageLengthData.top_width = %d",ImageLengthData.top_width);
        ROS_INFO("ImageLengthData.bottom_width = %d",ImageLengthData.bottom_width);*/
        tku_msgs::FeaturePoint feature_point_tmp;
        int distance_last = -1;
        int scan_line_last;
        if(Field_feature_point.size() < 40)     // no feature point = 36
        {
            whiteline_flag = false;
        }
        else
        {
            whiteline_flag = true;
        }
        for(int i = 0; i < Field_feature_point.size(); i++)
        {
            Distance distance;
            tku_msgs::Distance tmp;
            distance = measure(Field_feature_point[i].x,Field_feature_point[i].y,CameraType::stereo);
            if(i == 0)
            {
                tmp.x_dis = distance.x_dis;
                tmp.y_dis = distance.y_dis;
                tmp.dis = distance.dis;
                distance_last = distance.dis;
                scan_line_last = Field_feature_point[i].scan_line_cnt;
                feature_point_tmp.feature_point.push_back(tmp);
                if(tmp.dis > 0)
                {
                    circle(monitor, Point(Field_feature_point[i].x, Field_feature_point[i].y), 3, Scalar(0, 255, 255), 3);
                }
            }
            else if(distance_last >= 0)
            {
                if(scan_line_last == Field_feature_point[i].scan_line_cnt)
                {
                    int p2p_dis = abs(distance_last - distance.dis);
                    if(p2p_dis > 10)
                    {
                        tmp.x_dis = distance.x_dis;
                        tmp.y_dis = distance.y_dis;
                        tmp.dis = distance.dis;
                        distance_last = distance.dis;
                        scan_line_last = Field_feature_point[i].scan_line_cnt;
                        feature_point_tmp.feature_point.push_back(tmp);
                        circle(monitor, Point(Field_feature_point[i].x, Field_feature_point[i].y), 3, Scalar(0, 255, 255), 3);
                    }
                }
                else
                {
                    Observation_Data.scan_line.push_back(feature_point_tmp);
                    feature_point_tmp.feature_point.clear();

                    tmp.x_dis = distance.x_dis;
                    tmp.y_dis = distance.y_dis;
                    tmp.dis = distance.dis;
                    distance_last = distance.dis;
                    scan_line_last = Field_feature_point[i].scan_line_cnt;
                    feature_point_tmp.feature_point.push_back(tmp);
                    if(tmp.dis > 0)
                    {
                        circle(monitor, Point(Field_feature_point[i].x, Field_feature_point[i].y), 3, Scalar(0, 255, 255), 3);
                    }
                }
            }
            else
            {
                Observation_Data.scan_line.push_back(feature_point_tmp);
                feature_point_tmp.feature_point.clear();

                tmp.x_dis = distance.x_dis;
                tmp.y_dis = distance.y_dis;
                tmp.dis = distance.dis;
                distance_last = distance.dis;
                scan_line_last = Field_feature_point[i].scan_line_cnt;
                feature_point_tmp.feature_point.push_back(tmp);
                if(tmp.dis > 0)
                {
                    circle(monitor, Point(Field_feature_point[i].x, Field_feature_point[i].y), 3, Scalar(0, 255, 255), 3);
                }
            }
            if(i == (Field_feature_point.size() - 1))
            {
                if(scan_line_last == Field_feature_point[i].scan_line_cnt)
                {
                    int p2p_dis = abs(distance_last - distance.dis);
                    if(p2p_dis > 10)
                    {
                        tmp.x_dis = distance.x_dis;
                        tmp.y_dis = distance.y_dis;
                        tmp.dis = distance.dis;
                        distance_last = distance.dis;
                        scan_line_last = Field_feature_point[i].scan_line_cnt;
                        feature_point_tmp.feature_point.push_back(tmp);
                        circle(monitor, Point(Field_feature_point[i].x, Field_feature_point[i].y), 3, Scalar(0, 255, 255), 3);
                    }
                }
                else
                {
                    Observation_Data.scan_line.push_back(feature_point_tmp);
                    feature_point_tmp.feature_point.clear();

                    tmp.x_dis = distance.x_dis;
                    tmp.y_dis = distance.y_dis;
                    tmp.dis = distance.dis;
                    feature_point_tmp.feature_point.push_back(tmp);
                    if(tmp.dis > 0)
                    {
                        circle(monitor, Point(Field_feature_point[i].x, Field_feature_point[i].y), 3, Scalar(0, 255, 255), 3);
                    }
                }
                Observation_Data.scan_line.push_back(feature_point_tmp);
                feature_point_tmp.feature_point.clear();
            }
        }
        resize(monitor, monitor, cv::Size(320, 240));
        // namedWindow("monitor",WINDOW_NORMAL);
        // imshow("monitor",monitor);
        if(soccer_data.size() == 0 && goal_data.size() == 0)
        {
            tku_msgs::SoccerData tmp;
            tmp.x = 0;
            tmp.y = 0;
            tmp.height = 0;
            tmp.width = 0;
            tmp.object_mode = 2;
            Soccer.object_cnt = 1;
            Soccer.ObjectList.push_back(tmp);
            SoccerData_Publisher.publish(Soccer);
            soccer_data.clear();
        }
        else
        {
            int cnt = 0;
            if(soccer_data.size() != 0)
            {
                for(size_t t = 0; t < soccer_data.size(); t++)
                {
                    tku_msgs::SoccerData tmp;
                    Distance distance;
                    // ROS_INFO("Soccer_X = %d",soccer_data[t].x);
                    // ROS_INFO("Soccer_Y = %d",soccer_data[t].y);
                    // ROS_INFO("Soccer_Height = %d",soccer_data[t].height);
                    // ROS_INFO("Soccer_Width = %d\n",soccer_data[t].width);

                    tmp.x = soccer_data[t].x;
                    tmp.y = soccer_data[t].y;
                    tmp.height = soccer_data[t].height;
                    tmp.width = soccer_data[t].width;
                    tmp.object_mode = 0;
                    int x = soccer_data[t].x + (soccer_data[t].width / 2);
                    int y = soccer_data[t].y + soccer_data[t].height;
                    distance = measure(x,y,CameraType::stereo);
                    tmp.distance.x_dis = distance.x_dis;
                    tmp.distance.y_dis = distance.y_dis;
                    tmp.distance.dis = distance.dis;
                    Soccer.ObjectList.push_back(tmp);
                    cnt++;
                    // ROS_INFO("mode = %d",tmp.object_mode);
                    // ROS_INFO("x_soccer = %d",Soccer.ObjectList[t].x);
		            // ROS_INFO("soccer_dis = %d",tmp.distance);
                }
                soccer_data.clear();
            }
            if(goal_data.size() != 0)
            {
                for(size_t t = 0; t < goal_data.size(); t++)
                {
                    tku_msgs::SoccerData tmp;
                    Distance distance;
                    // ROS_INFO("goal_data_X = %d",goal_data[t].x);
                    // ROS_INFO("goal_data_Y = %d",goal_data[t].y);
                    // ROS_INFO("goal_data_Height = %d",goal_data[t].height);
                    // ROS_INFO("goal_data_Width = %d\n",goal_data[t].width);

                    tmp.x = goal_data[t].x;
                    tmp.y = goal_data[t].y;
                    tmp.height = goal_data[t].height;
                    tmp.width = goal_data[t].width;
                    tmp.object_mode = 1;
                    int x = soccer_data[t].x + (soccer_data[t].width / 2);
                    int y = soccer_data[t].y + soccer_data[t].height;
                    distance = measure(x,y,CameraType::stereo);
                    tmp.distance.x_dis = distance.x_dis;
                    tmp.distance.y_dis = distance.y_dis;
                    tmp.distance.dis = distance.dis;
                    Soccer.ObjectList.push_back(tmp);
                    cnt++;
                    // ROS_INFO("mode = %d",tmp.object_mode);
                }
                goal_data.clear();
            }
            ROS_INFO("cnt = %d",cnt);
            Soccer.object_cnt = cnt;
            SoccerData_Publisher.publish(Soccer);
            Soccer.ObjectList.clear();
        }
        Observation_Data.imagestate = whiteline_flag;
        ObservationData_Publisher.publish(Observation_Data);
        //ROS_INFO("13x = %d y = %d dis = %d",FeaturePoint_distance.x_dis[13],FeaturePoint_distance.y_dis[13],FeaturePoint_distance.dis[13]);
        //ROS_INFO("18x = %d y = %d dis = %d",FeaturePoint_distance.x_dis[18],FeaturePoint_distance.y_dis[18],FeaturePoint_distance.dis[18]);
        //ROS_INFO("23x = %d y = %d dis = %d",FeaturePoint_distance.x_dis[23],FeaturePoint_distance.y_dis[23],FeaturePoint_distance.dis[23]);
        //ROS_INFO("13x = %d y = %d",Field_feature_point[13].X,Field_feature_point[13].Y);
        //ROS_INFO("18x = %d y = %d",Field_feature_point[18].X,Field_feature_point[18].Y);
        //ROS_INFO("23x = %d y = %d",Field_feature_point[23].X,Field_feature_point[23].Y);
        //ROS_INFO("camera_height = %f",camera_height);
        //ROS_INFO("distance_x = %d",FeaturePoint_distance.x_dis[18]);
        //ROS_INFO("distance_y = %d",FeaturePoint_distance.y_dis[18]);
        Observation_Data.scan_line.clear();
        Observation_Data.landmark.clear();
        // resize(orign, orign, cv::Size(320, 240));
        resize(imageGamma, imageGamma, cv::Size(320, 240));
        resize(nobackgroud_image, nobackgroud_image, cv::Size(320, 240));
        // resize(morph, morph, cv::Size(320, 240));
        resize(edge, edge, cv::Size(320, 240));
        resize(Gmask, Gmask, cv::Size(320, 240));
        resize(hough_frame, hough_frame, cv::Size(320, 240));
        resize(merge_hough_frame, merge_hough_frame, cv::Size(320, 240));

        Mat MyCombine(Gmask.rows, Gmask.cols*2, CV_8UC1, Scalar(0,255,255));
        Gmask.copyTo(MyCombine(Rect(0,0,Gmask.cols,Gmask.rows)));
	    edge.copyTo(MyCombine(Rect(Gmask.cols,0,Gmask.cols,Gmask.rows)));
        
        Mat MyCombine1(imageGamma.rows, imageGamma.cols*2, CV_8UC3, Scalar(0,255,255));
        imageGamma.copyTo(MyCombine1(Rect(0,0,imageGamma.cols,imageGamma.rows)));
	    nobackgroud_image.copyTo(MyCombine1(Rect(imageGamma.cols,0,imageGamma.cols,imageGamma.rows)));
        
        Mat MyCombine2(hough_frame.rows, hough_frame.cols*2, CV_8UC3, Scalar(0,255,255));
        hough_frame.copyTo(MyCombine2(Rect(0,0,hough_frame.cols,hough_frame.rows)));
	    merge_hough_frame.copyTo(MyCombine2(Rect(hough_frame.cols,0,hough_frame.cols,hough_frame.rows)));
        
        msg_object = cv_bridge::CvImage(std_msgs::Header(), "bgr8", Object_frame).toImageMsg();
        msg_monitor = cv_bridge::CvImage(std_msgs::Header(), "bgr8", monitor).toImageMsg();
        msg_measure = cv_bridge::CvImage(std_msgs::Header(), "bgr8", oframe).toImageMsg();
        msg_mask = cv_bridge::CvImage(std_msgs::Header(), "mono8", MyCombine).toImageMsg();
        msg_imageGamma = cv_bridge::CvImage(std_msgs::Header(), "bgr8", MyCombine1).toImageMsg();
        msg_hough = cv_bridge::CvImage(std_msgs::Header(), "bgr8", MyCombine2).toImageMsg();

    
        Object_Frame_Publisher.publish(msg_object);
        Monitor_Frame_Publisher.publish(msg_monitor);
        Measure_Frame_Publisher.publish(msg_measure);
        Gamma_Frame_Publisher.publish(msg_imageGamma);
        mask_Frame_Publisher.publish(msg_mask);
        MerHough_Publisher.publish(msg_hough);


        waitKey(1);
    }
}
