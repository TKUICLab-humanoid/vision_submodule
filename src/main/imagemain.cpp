#include "main/imagemain.h"

Vision_main::Vision_main(ros::NodeHandle &nh)
{
    this->nh = &nh;
    image_transport::ImageTransport it(nh);

    color_nh.setCallbackQueue(&color_queue);
    color_queue.callAvailable(ros::WallDuration());
    color_spinner = new ros::AsyncSpinner(1, &color_queue);
    color_spinner->start();

    depth_nh.setCallbackQueue(&depth_queue);
    depth_queue.callAvailable(ros::WallDuration());
    depth_spinner = new ros::AsyncSpinner(1, &depth_queue);
    depth_spinner->start();

    // for realsense D435i
    Depthimage_subscriber = depth_nh.subscribe("/camera/aligned_depth_to_color/image_raw", 1, &Vision_main::DepthCallback,this);
    Imagesource_subscriber = color_nh.subscribe("/camera/color/image_raw", 1, &Vision_main::GetImagesourceFunction,this);

    // Imagesource_subscriber = nh.subscribe("/usb_cam/image_raw", 1, &Vision_main::GetImagesourceFunction,this);
    HeadAngle_subscriber = nh.subscribe("/package/HeadMotor", 10, &Vision_main::HeadAngleFunction,this);
    IMUData_Subscriber = nh.subscribe("/package/sensorpackage", 1, &Vision_main::GetIMUDataFunction,this);
    
    //--------------HSV---------------
    ModelingButton_subscriber = nh.subscribe("ColorModelForm_Topic", 1000, &Vision_main::ModelingFunction,this);
    HSVValue_subscriber = nh.subscribe("HSVValue_Topic", 1000, &Vision_main::ChangeHSVValue,this);
    HSV_service = nh.advertiseService("LoadHSVInfo", &Vision_main::LoadHSVInfo,this);
    Build_service = nh.advertiseService("BuildModel", &Vision_main::CallBuildFunction,this);
    Save_service = nh.advertiseService("SaveHSV", &Vision_main::CallSaveHSVFunction,this);
    //--------------------------------
    ObservationData_Publisher = nh.advertise<tku_msgs::ObservationData>("/vision/observation_data", 10);
    ImageLengthData_Publisher = nh.advertise<tku_msgs::ImageLengthData>("/vision/imagelength_data", 10);
    SoccerData_Publisher = nh.advertise<tku_msgs::SoccerDataList>("/vision/soccer_topic",10);

    Object_Frame_Publisher = it.advertise("/vision/object_image", 1);
    Monitor_Frame_Publisher = it.advertise("/vision/monitor_image", 1);
    Measure_Frame_Publisher = it.advertise("/vision/measure_image", 1);

    pitch_pre = 0.0;
    roll_pre = 0.0;
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
      resize(depth_buffer, depth_buffer, cv::Size(640, 480));
      //resize(depth_buffer, depth_buffer, cv::Size(320, 240));
      //imshow("depth_buffer",depth_buffer);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("DepthCallback cv_bridge exception: %s", e.what());
      return;
    }
}

void Vision_main::ModelingFunction(const tku_msgs::ButtonColorForm& msg)
{
    if(msg.BuildingModel)
    {
        Model_Base->isBuildModel = true;
    }
    else
    {
        Model_Base->isBuildModel = false;
    }
}

void Vision_main::ChangeHSVValue(const tku_msgs::HSVValue& msg)
{
    Model_Base->hsvColorRange->HueMax = (float)msg.HMax/HueScrollBarMax;
    Model_Base->hsvColorRange->HueMin = (float)msg.HMin/HueScrollBarMax;
    Model_Base->hsvColorRange->SaturationMax = (float)msg.SMax/SaturationScrollBarMax;
    Model_Base->hsvColorRange->SaturationMin = (float)msg.SMin/SaturationScrollBarMax;
    Model_Base->hsvColorRange->BrightnessMax = (float)msg.VMax/BrightnessScrollBarMax;
    Model_Base->hsvColorRange->BrightnessMin = (float)msg.VMin/BrightnessScrollBarMax;
}
bool Vision_main::LoadHSVInfo(tku_msgs::HSVInfo::Request &HSVreq, tku_msgs::HSVInfo::Response &HSVres)
{
    Model_Base->hsvColorRange = Model_Base->HSVColorRange[HSVreq.ColorLabel];
    Model_Base->ColorSelected = HSVreq.ColorLabel;
    HSVres.Hmax = Model_Base->HSVColorRange[HSVreq.ColorLabel]->HueMax * HueScrollBarMax;
    HSVres.Hmin = Model_Base->HSVColorRange[HSVreq.ColorLabel]->HueMin * HueScrollBarMax;
    HSVres.Smax = Model_Base->HSVColorRange[HSVreq.ColorLabel]->SaturationMax * SaturationScrollBarMax;
    HSVres.Smin = Model_Base->HSVColorRange[HSVreq.ColorLabel]->SaturationMin * SaturationScrollBarMax;
    HSVres.Vmax = Model_Base->HSVColorRange[HSVreq.ColorLabel]->BrightnessMax * BrightnessScrollBarMax;
    HSVres.Vmin = Model_Base->HSVColorRange[HSVreq.ColorLabel]->BrightnessMin * BrightnessScrollBarMax;
    return true;
}
bool Vision_main::CallBuildFunction(tku_msgs::BuildModel::Request &req, tku_msgs::BuildModel::Response &res)
{
    if(req.Build)
    {
        Model_Base->HSV_BuildingColorModel();
        res.Already = true;
    }
}
bool Vision_main::CallSaveHSVFunction(tku_msgs::SaveHSV::Request &req, tku_msgs::SaveHSV::Response &res)
{
    if(req.Save)
    {
        Model_Base->SaveColorRangeFile();
        res.Already = true;
    }
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
    ROS_INFO("Vertical_Head.pos = %d",Vertical_Head.pos);
    ROS_INFO("Horizontal_Head.pos = %d",Horizontal_Head.pos);
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
        // Lee
        Robot_Roll = msg.IMUData[0];
        Robot_Pitch = msg.IMUData[1];
        calcImageAngle(Horizontal_Head,Vertical_Head);
        
        /* LightLight
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
        */
        //ROS_INFO("Robot_Roll = %f",Robot_Roll);
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "vision");

    ros::NodeHandle nh;
	Vision_main vision_main(nh);

    ros::Rate loop_rate(60);

    ros::spinOnce();
    vision_main.strategy_init();

    while (nh.ok())
    {
        ros::spinOnce();
        vision_main.strategy_main();
        loop_rate.sleep();
    }

    return 0;
}

void Vision_main::strategy_init()
{
    Roll_init = 0;
    Pitch_init = 0;
    ROS_INFO("Roll_init: %f", Roll_init);
    ROS_INFO("Pitch_init: %f", Pitch_init);
    CalcRobotHeight();
    if(!cascader2soccer.load(GetPath("cascade2soccer.xml")))
    {
        ROS_INFO("could not load cascader2soccer.xml");
    }
    if(!cascader2goal.load(GetPath("cascade2goal.xml")))
    {
        ROS_INFO("could not load cascader2goal.xml");
    }
}

void Vision_main::strategy_main()
{
    if(!color_buffer.empty() && !depth_buffer.empty())
    {
        cv::Mat oframe = color_buffer.clone();
        line(oframe, Point(oframe.cols/2,oframe.rows), Point(oframe.cols/2,0), Scalar(0,0,255), 1);
        line(oframe, Point(0,oframe.rows/2), Point(oframe.cols,oframe.rows/2), Scalar(0,0,255), 1);
        resize(oframe, oframe, cv::Size(320, 240));
        //imshow("oframe",oframe);

        cv::Mat imagePreprocessing = ImagePreprocessing(color_buffer);
        //Mat line = Merge_similar_line(imagePreprocessing,color_buffer);
	    //imshow("line",line);
	
        cv::Mat Object_frame = FindObject(color_buffer);
        //imshow("Object_frame", Object_frame);

        // cv::Size dst_sz(color_buffer.cols,color_buffer.rows);
        // cv::Point2f center(dst_sz.height/2,dst_sz.width/2);

        // cv::Mat rot_mat = cv::getRotationMatrix2D(center, -1 * (Robot_Roll), 1.0);

        // cv::Mat dst;
        // cv::warpAffine(color_buffer, dst, rot_mat, dst_sz);
        //imshow("dst",dst);

        cv::Mat monitor = White_Line(imagePreprocessing);

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
        if(Filed_feature_point.size() < 40)     // no feature point = 36
        {
            whiteline_flag = false;
        }
        else
        {
            whiteline_flag = true;
        }
        for(int i = 0; i < Filed_feature_point.size(); i++)
        {
            Distance distance;
            tku_msgs::Distance tmp;
            distance = measure(Filed_feature_point[i].x,Filed_feature_point[i].y);
            if(i == 0)
            {
                tmp.x_dis = distance.x_dis;
                tmp.y_dis = distance.y_dis;
                tmp.dis = distance.dis;
                distance_last = distance.dis;
                scan_line_last = Filed_feature_point[i].scan_line_cnt;
                feature_point_tmp.feature_point.push_back(tmp);
                if(tmp.dis > 0)
                {
                    circle(monitor, Point(Filed_feature_point[i].x, Filed_feature_point[i].y), 3, Scalar(0, 255, 255), 3);
                }
            }
            else if(distance_last >= 0)
            {
                if(scan_line_last == Filed_feature_point[i].scan_line_cnt)
                {
                    int p2p_dis = abs(distance_last - distance.dis);
                    if(p2p_dis > 10)
                    {
                        tmp.x_dis = distance.x_dis;
                        tmp.y_dis = distance.y_dis;
                        tmp.dis = distance.dis;
                        distance_last = distance.dis;
                        scan_line_last = Filed_feature_point[i].scan_line_cnt;
                        feature_point_tmp.feature_point.push_back(tmp);
                        circle(monitor, Point(Filed_feature_point[i].x, Filed_feature_point[i].y), 3, Scalar(0, 255, 255), 3);
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
                    scan_line_last = Filed_feature_point[i].scan_line_cnt;
                    feature_point_tmp.feature_point.push_back(tmp);
                    if(tmp.dis > 0)
                    {
                        circle(monitor, Point(Filed_feature_point[i].x, Filed_feature_point[i].y), 3, Scalar(0, 255, 255), 3);
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
                scan_line_last = Filed_feature_point[i].scan_line_cnt;
                feature_point_tmp.feature_point.push_back(tmp);
                if(tmp.dis > 0)
                {
                    circle(monitor, Point(Filed_feature_point[i].x, Filed_feature_point[i].y), 3, Scalar(0, 255, 255), 3);
                }
            }
            if(i == (Filed_feature_point.size() - 1))
            {
                if(scan_line_last == Filed_feature_point[i].scan_line_cnt)
                {
                    int p2p_dis = abs(distance_last - distance.dis);
                    if(p2p_dis > 10)
                    {
                        tmp.x_dis = distance.x_dis;
                        tmp.y_dis = distance.y_dis;
                        tmp.dis = distance.dis;
                        distance_last = distance.dis;
                        scan_line_last = Filed_feature_point[i].scan_line_cnt;
                        feature_point_tmp.feature_point.push_back(tmp);
                        circle(monitor, Point(Filed_feature_point[i].x, Filed_feature_point[i].y), 3, Scalar(0, 255, 255), 3);
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
                        circle(monitor, Point(Filed_feature_point[i].x, Filed_feature_point[i].y), 3, Scalar(0, 255, 255), 3);
                    }
                }
                Observation_Data.scan_line.push_back(feature_point_tmp);
                feature_point_tmp.feature_point.clear();
            }
        }
        resize(monitor, monitor, cv::Size(320, 240));
        // namedWindow("monitor",WINDOW_NORMAL);
        // imshow("monitor",monitor);
        int cnt = soccer_data.size() + goal_data.size();
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
                    distance = measure(x,y);
                    tmp.distance.x_dis = distance.x_dis;
                    tmp.distance.y_dis = distance.y_dis;
                    tmp.distance.dis = distance.dis;
                    ROS_INFO("distance.x_dis: %d", distance.x_dis);
                    ROS_INFO("distance.y_dis: %d", distance.y_dis);
                    ROS_INFO("distance.dis: %d", distance.dis);
                    Soccer.ObjectList.push_back(tmp);
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
                    distance = measure(x,y);
                    tmp.distance.x_dis = distance.x_dis;
                    tmp.distance.y_dis = distance.y_dis;
                    tmp.distance.dis = distance.dis;
                    Soccer.ObjectList.push_back(tmp);
                    // ROS_INFO("mode = %d",tmp.object_mode);
                }
                goal_data.clear();
            }
            Soccer.object_cnt = cnt;
            SoccerData_Publisher.publish(Soccer);
            Soccer.ObjectList.clear();
        }
        // ROS_INFO("cnt = %d", cnt);
        Observation_Data.imagestate = whiteline_flag;
        ObservationData_Publisher.publish(Observation_Data);
        //ROS_INFO("13x = %d y = %d dis = %d",FeaturePoint_distance.x_dis[13],FeaturePoint_distance.y_dis[13],FeaturePoint_distance.dis[13]);
        //ROS_INFO("18x = %d y = %d dis = %d",FeaturePoint_distance.x_dis[18],FeaturePoint_distance.y_dis[18],FeaturePoint_distance.dis[18]);
        //ROS_INFO("23x = %d y = %d dis = %d",FeaturePoint_distance.x_dis[23],FeaturePoint_distance.y_dis[23],FeaturePoint_distance.dis[23]);
        //ROS_INFO("13x = %d y = %d",Filed_feature_point[13].X,Filed_feature_point[13].Y);
        //ROS_INFO("18x = %d y = %d",Filed_feature_point[18].X,Filed_feature_point[18].Y);
        //ROS_INFO("23x = %d y = %d",Filed_feature_point[23].X,Filed_feature_point[23].Y);
        //ROS_INFO("camera_height = %f",camera_height);
        //ROS_INFO("distance_x = %d",FeaturePoint_distance.x_dis[18]);
        //ROS_INFO("distance_y = %d",FeaturePoint_distance.y_dis[18]);
        Observation_Data.scan_line.clear();

        msg_object = cv_bridge::CvImage(std_msgs::Header(), "bgr8", Object_frame).toImageMsg();
        msg_monitor = cv_bridge::CvImage(std_msgs::Header(), "bgr8", monitor).toImageMsg();
        msg_measure = cv_bridge::CvImage(std_msgs::Header(), "bgr8", oframe).toImageMsg();
        Object_Frame_Publisher.publish(msg_object);
        Monitor_Frame_Publisher.publish(msg_monitor);
        Measure_Frame_Publisher.publish(msg_measure);

        waitKey(1);
    }
}
