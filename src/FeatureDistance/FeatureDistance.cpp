#include <FeatureDistance/FeatureDistance.h>

// FeatureDistanceInstance* FeatureDistanceInstance::m_pInstance;

// FeatureDistanceInstance* FeatureDistanceInstance::getInstance()
// {
//     if(!m_pInstance)m_pInstance = new FeatureDistanceInstance();
//     return m_pInstance;
// }

// void FeatureDistanceInstance::deleteInstance()
// {
//     if(m_pInstance)
//     {
//         delete m_pInstance;
//         m_pInstance = NULL;
//     }
// }

FeatureDistance::FeatureDistance()
{  
    tool = ToolInstance::getInstance();
    //initialization  (the angle of head is 1350)                                                
    camera_height = 56.883;                 //攝影機高度

    camera2robot_dis = 0.0;                 //攝影機到機器人距離

    image_bottom_angle = 2.617;               //畫面最底與機器人之間夾角
    half_VFOV_angle = VFOV / 2.0;      //垂直視角一半
    half_HFOV_angle = HFOV / 2.0;    //水平視角一半
    image_center_horizontal_length = 320.0 / tan(half_HFOV_angle * DEG2RAD);     //水平畫面中間垂直長度（攝影機畫面）

    Vertical_Head_Angle = 28.6521;              //頭部垂直角度

    vertical_angle = 0.0;                       //物體垂直角度
    horizontal_angle = 0.0;                     //物體水平角度

    image_top_length = 53;         //畫面最遠距離
    image_bottom_length = 2;      //畫面最底距離
    image_top_width_length = 36;        //畫面最遠寬度(一半)
    image_bottom_width_length = 10;     //畫面最底寬度(一半)

    Roll_init = 0.0;
    Pitch_init = 0.0;

    Robot_Pitch = 0.0;
    Robot_Roll = 0.0;

    whiteline_flag = false;
}
FeatureDistance::~FeatureDistance()
{

}

float FeatureDistance::AvgPixelDistance(int Feature_x, int Feature_y)
{
    Size range = Size(2,2);
    Rect RectRange(Feature_x-range.width/2,Feature_y-range.height/2,range.width,range.height);
    float depth_scale = 0.1;//0.1為cm 1為mm
    float effective_distance = 0.0;
    float distance_sum = 0.0;
    int effective_pixel = 0;
    if(RectRange.y <= 0)
    {
        RectRange.y = 0;
    }
    if(RectRange.x <= 0)
    {
        RectRange.x = 0;
    }
    if(RectRange.y >=479)
    {
        RectRange.y = 473;
    }
    if(RectRange.x >= 639)
    {       
        RectRange.x = 633;
    }
    if(!depth_buffer.empty())
    {
        ROS_INFO("!depth_buffer.empty()");
        ROS_INFO("RectRange.y = %d, RectRange.height = %d", RectRange.y, RectRange.height);
        
        for(int y=RectRange.y;y<RectRange.y+RectRange.height;y++)
        {
            for(int x=RectRange.x;x<RectRange.x+RectRange.width;x++)
            {
                ROS_INFO("buffer = %f", depth_buffer.at<uint16_t>(y,x));
                //如果深度图下該點像素不為0，表示有距離信息
                if(depth_buffer.at<uint16_t>(y,x)){
                    distance_sum+=depth_scale*depth_buffer.at<uint16_t>(y,x);
                    effective_pixel++;
                    ROS_INFO("distance_sum = %f",distance_sum);
                    ROS_INFO("effective_pixel = %d",effective_pixel);
                }else{
                    // distance_sum+=0.0;
                }
            }
        }
        effective_distance = (distance_sum/float(effective_pixel));
        if(!std::isfinite(distance_sum)||!std::isfinite(effective_pixel) || distance_sum == 0.0 || effective_pixel == 0)
        {
            ROS_INFO("distance_sum = %f, effective_pixel = %f", distance_sum, effective_pixel);
            effective_distance = 0.0;
            ROS_INFO("effective_distance = %f",effective_distance);
        }        
        // ROS_INFO("effective_distance = %f",effective_distance);    
    }else{
        ROS_INFO("depth_buffer == empty");
        effective_distance = 0.0;
    }
    // ROS_INFO("effective_distance = %f",effective_distance);
    return effective_distance;
}



//To get the distance from object to robot foot
Distance FeatureDistance::measure(int Feature_x, int Feature_y,CameraType cameratype)
{
    ROS_INFO("%d, %d", Feature_x, Feature_y);
    float width_cnt = (float)Feature_x;
    float height_cnt = (float)Feature_y;
    float error_y;
    float error_x;
    float avgdistance = 0.0;
    Distance distance;
    distance.x_dis = 0;
    distance.y_dis = 0;
    distance.dis = 0;

    switch(cameratype)
    {
        case CameraType::stereo:

            avgdistance = AvgPixelDistance(Feature_x,Feature_y);
            ROS_INFO("avgdistance = %f",avgdistance);
            if(!depth_buffer.empty() && avgdistance != 0.000000)
            {
                ROS_INFO("x = %d, y = %d", Feature_x, Feature_y);
                ROS_INFO("start");
                
                float theta_y = 0.0;               
                float theta_x = 0.0;
                if(height_cnt > 240)
                {
                    error_y = height_cnt - 240.0;
                    theta_y = atan((error_y/(depth_buffer.rows/2))*tan(half_VFOV_angle*DEG2RAD));
                }
                else
                {
                    error_y = 240.0 - height_cnt;
                    theta_y = atan((error_y/(depth_buffer.rows/2))*tan(half_VFOV_angle*DEG2RAD));
                }
                if(width_cnt > 320)
                {
                    error_x = width_cnt - 320.0;
                    theta_x = atan2(error_x, image_center_horizontal_length);  
                }
                else
                {
                    error_x = 320.0 - width_cnt;
                    theta_x = atan2(error_x, image_center_horizontal_length);
                }
                if(!std::isfinite(theta_x))
                {
                    theta_x = 0.;
                }
                if(!std::isfinite(theta_y))
                {
                    theta_y = 0.;
                }
                double OYp = avgdistance * cos(theta_x);
                ROS_INFO("theta_x = %f theta_y = %f avgdistance = %f",theta_x*RAD2DEG,theta_y*RAD2DEG,avgdistance);
                // ROS_INFO("OYp = %f RealsenseIMUData[0] = %f",OYp,RealsenseIMUData[0]);
                if(std::isfinite(RealsenseIMUData[0]))
                {
                    // ROS_INFO("isfinite(RealsenseIMUData[0]");
                    Robot_H = OYp * cos((RealsenseIMUData[0]* DEG2RAD) + theta_y);                                

                    ROS_INFO("avgdistance(%d,%d) = %f, Robot_H = %f",Feature_x,Feature_y, avgdistance,Robot_H);
                    
                    if(std::isfinite(avgdistance) && avgdistance < 1000.0)
                    {
                        distance.y_dis = int(round(OYp * sin((RealsenseIMUData[0] * DEG2RAD) + theta_y)));
                        distance.x_dis = int(round(OYp * tan(theta_x)));
                        distance.dis = int(round(sqrt(pow(distance.x_dis,2)+pow(distance.y_dis,2))));
                        
                        Pixely = OYp * sin((RealsenseIMUData[0] * DEG2RAD) + theta_y);
                        Pixelx = OYp * tan(theta_x);
                        pixelDistance = sqrt(pow(Pixelx,2)+pow(Pixely,2));
                    }else{
                        ROS_INFO("Wrong depth distance");
                        measure(Feature_x,Feature_y,CameraType::Monocular);
                    }
                }else{
                    ROS_INFO("No IMU value");
                    if(std::isfinite(avgdistance) && avgdistance > camera_height && avgdistance < 1000.0)
                    {
                        camera_angle = acos(camera_height / avgdistance) * RAD2DEG - (round(AVGERRORANGLE) + (Vertical_Head_Angle-28.65)/8.785) ;
                        Robot_H = OYp * cos((camera_angle* DEG2RAD) + theta_y);                                
                        distance.y_dis = int(round(OYp * sin((camera_angle * DEG2RAD) + theta_y)+camera2robot_dis));
                        distance.x_dis = int(round(OYp * tan(theta_x)));
                        distance.dis = int(round(sqrt(pow(distance.x_dis,2)+pow(distance.y_dis,2))));
                        Pixely = OYp * sin((camera_angle * DEG2RAD) + theta_y);
                        Pixelx = OYp * tan(theta_x);
                        pixelDistance = sqrt(pow(Pixelx,2)+pow(Pixely,2));

                    }else{
                        ROS_INFO("Wrong depth distance");
                        measure(Feature_x,Feature_y,CameraType::Monocular);
                    }
                } 
                if(Horizontal_Head_Angle != 0.0)
                {
                    ROS_INFO("Horizontal_Head_Angle  = %d", Horizontal_Head_Angle);
                    if(Horizontal_Head_Angle < 0.0)
                    {
                        distance.y_dis = distance.dis * cos(Horizontal_Head_Angle * DEG2RAD);
                        distance.x_dis = distance.x_dis - (distance.dis * sin(Horizontal_Head_Angle * DEG2RAD));
                    }
                    else
                    {
                        distance.y_dis = distance.dis * cos(Horizontal_Head_Angle * DEG2RAD);
                        distance.x_dis = distance.x_dis - (distance.dis * sin(Horizontal_Head_Angle * DEG2RAD));
                    }
                }  
                distance.dis = int(round(sqrt(pow(distance.x_dis,2)+pow(distance.y_dis,2))));     
                // ROS_INFO("finish");
                ROS_INFO("stereo: x = %d, y = %d, dis = %d",distance.x_dis,distance.y_dis,distance.dis);
                float total_dis;
                if(distance.y_dis < 0)
                    total_dis = camera2robot_dis - distance.dis;
                else
                    total_dis = camera2robot_dis + distance.dis;
                ROS_INFO("total_dis = %f", total_dis);
                tool->Delay(1);
                break;
            }
 
        case CameraType::Monocular:

            // ROS_INFO("Monocular");
            if(width_cnt == -1 && height_cnt == -1)
            {
                distance.x_dis = -1;
                distance.y_dis = -1;
                distance.dis = -1;
            }
            else
            {
                if(height_cnt > 240)
                {
                    error_y = height_cnt - 240.0;
                    vertical_angle = image_bottom_angle + half_VFOV_angle - atan2(error_y , 640) * 180 / PI;
                    distance.y_dis = int(round(camera_height * tan(vertical_angle * DEG2RAD) + camera2robot_dis));
                }
                else
                {
                    error_y = 240.0 - height_cnt;
                    vertical_angle = image_bottom_angle + half_VFOV_angle + atan2(error_y , 640) * 180 / PI;
                    distance.y_dis = int(round(camera_height * tan(vertical_angle * DEG2RAD) /*- camera2robot_dis*/));
                }
                if(width_cnt > 320)
                {
                    error_x = width_cnt - 320.0;
                    horizontal_angle = atan2(error_x, image_center_horizontal_length) * 180 / PI;   //325.534
                    distance.x_dis = (camera_height / cos(vertical_angle * DEG2RAD)) * tan(horizontal_angle * DEG2RAD);
                }
                else
                {
                    error_x = 320.0 - width_cnt;
                    horizontal_angle = atan2(error_x, image_center_horizontal_length) * 180 / PI;   //325.534
                    distance.x_dis = -((camera_height / cos(vertical_angle * DEG2RAD)) * tan(horizontal_angle * DEG2RAD));
                }
                distance.dis = sqrt(pow(distance.y_dis,2) + pow(distance.x_dis,2));
            }
            // if(Horizontal_Head_Angle != 0.0)
            // {
            //     if(Horizontal_Head_Angle < 0.0)
            //     {
            //         distance.y_dis = distance.dis * cos(Horizontal_Head_Angle * DEG2RAD);
            //         distance.x_dis = distance.x_dis - (distance.dis * sin(Horizontal_Head_Angle * DEG2RAD));
            //     }
            //     else
            //     {
            //         distance.y_dis = distance.dis * cos(Horizontal_Head_Angle * DEG2RAD);
            //         distance.x_dis = distance.x_dis - (distance.dis * sin(Horizontal_Head_Angle * DEG2RAD));
            //     }
            // }
            // ROS_INFO("Monocular: x = %d, y = %d, dis = %d",distance.x_dis,distance.y_dis,distance.dis);
            //ROS_INFO("distance_d.x_dis = %d",distance.x_dis);
            //ROS_INFO("distance_d.y_dis = %d",distance.y_dis);
            //ROS_INFO("distance_d.dis = %d",distance.dis);
            break;
        
    }
    return distance;
}

void FeatureDistance::calcImageAngle(motordata Horizontal_Head,motordata Vertical_Head)
{
    calcMotorAngle(Horizontal_Head.pos,Vertical_Head.pos);
    float Moving_angle_error = camera_angle_offest + (Robot_Pitch - Pitch_init)-4;
    float foot2robot_dis;
    float foot2robot_angle;
    float camera_error;
    float camera_error_dis;
    float L_5;
    L_5 = 2.71;
    camera_error = L_5 * sin(Moving_angle_error * DEG2RAD);
    camera_error_dis = L_5 * cos(Moving_angle_error * DEG2RAD);
    if((Robot_Pitch - Pitch_init) >= 0)
    {
        foot2robot_dis = sqrt(pow(RobotHeight, 2)+pow(RobotWidth - W_Front, 2)); // front shoe
        foot2robot_angle = atan2(RobotHeight, RobotWidth - W_Front) * 180 / PI - (Robot_Pitch - Pitch_init);
    }
    else
    {
        foot2robot_dis = sqrt(pow(RobotHeight, 2)+pow(RobotWidth + W_Behind, 2)); // behind shoe
        foot2robot_angle = atan2(RobotHeight, RobotWidth + W_Behind) * 180 / PI - (Robot_Pitch - Pitch_init);
    }

    // camera_height = foot2robot_dis * sin(foot2robot_angle * DEG2RAD)  + L_CAMERA * sin((Vertical_Head_Angle - Moving_angle_error) * DEG2RAD);
    camera_height = RobotHeight - camera_error + L_CAMERA * sin((Vertical_Head_Angle - Moving_angle_error) * DEG2RAD);
    camera2robot_dis = RobotHeight * sin((Robot_Pitch - Pitch_init) * DEG2RAD) + camera_error_dis + L_CAMERA * cos((Vertical_Head_Angle - Moving_angle_error) * DEG2RAD);
    
    image_bottom_angle = Vertical_Head_Angle - half_VFOV_angle - AVGERRORANGLE - Moving_angle_error;
    image_top_length = camera2robot_dis + camera_height * tan((VFOV + image_bottom_angle) * DEG2RAD);
    image_bottom_length = camera2robot_dis + camera_height * tan(image_bottom_angle * DEG2RAD);
    image_top_width_length = (camera_height / cos((VFOV + image_bottom_angle) * DEG2RAD)) * tan(half_HFOV_angle * DEG2RAD);
    image_bottom_width_length = (camera_height / cos(image_bottom_angle * DEG2RAD)) * tan(half_HFOV_angle * DEG2RAD);

    ROS_INFO("");
    ROS_INFO("camera = %f", L_CAMERA * sin((Vertical_Head_Angle - Moving_angle_error) * DEG2RAD));
    ROS_INFO("length = %f", camera_height * tan((VFOV + image_bottom_angle) * DEG2RAD));
    ROS_INFO("length2 = %f", camera_height * tan((image_bottom_angle) * DEG2RAD));
    ROS_INFO("camera_error =       %f", camera_error);
    ROS_INFO("camera_error_dis =   %f", camera_error_dis);
    ROS_INFO("Robot_Pitch = %f, Pitch_init = %f", Robot_Pitch, Pitch_init);
    ROS_INFO("camera_angle_offest = %f", camera_angle_offest);
    ROS_INFO("Vertical_Head_Angle= %f,Moving_angle_error= %f", Vertical_Head_Angle, Moving_angle_error);
    ROS_INFO("foot2robot_dis =     %f, foot2robot_angle = %f", foot2robot_dis, foot2robot_angle);
    ROS_INFO("RobotHeight =        %f, RobotWidth = %f",RobotHeight, RobotWidth);
    ROS_INFO("camera_height =      %f",camera_height);
    ROS_INFO("camera2robot_dis =   %f",camera2robot_dis);
    ROS_INFO("image_bottom_angle = %f",image_bottom_angle);
    ROS_INFO("image_top_length =   %3d, image_top_width_length =   %3d", image_top_length, image_top_width_length);
    ROS_INFO("image_bottom_length= %3d, image_bottom_width_length= %3d", image_bottom_length, image_bottom_width_length);
    ROS_INFO("half_VFOV_angle =    %f, ", half_VFOV_angle);
}

void FeatureDistance::calcMotorAngle(int Horizontal_pos,int Vertical_pos)
{
    if(Vertical_pos > 2048)
    {
        Vertical_Head_Angle = 90 + (Vertical_pos - 2048) * MOTORDEG;
    }
    else
    {
        Vertical_Head_Angle = (Vertical_pos - 1024) * MOTORDEG;
    }
    Horizontal_Head_Angle = (Horizontal_pos - 2048) * MOTORDEG;
}

double FeatureDistance::CalcRobotHeight()
{
    StandPackage.clear();
    ROS_INFO("CalcRobotHeight");
    string parameter_path = "N";
	char source[200];
	char search[9] = "Desktop/"; 
	char *loc;
    char *src;
    char *dst;
	char standPath[200];
    int length = 0;
    int packagecnt = 0;
    int cnt = 3;

	while(parameter_path == "N" && ros::ok())
	{
		parameter_path = tool->getPackagePath("strategy");

	}
	strcpy(source, parameter_path.c_str());
    
	loc = strstr(source, search);
    src = source;
    dst = standPath;
    length = strlen(source)-strlen(loc)+strlen(search);

    while(length--)
    {
        *(dst++) = *(src++);
    }
    *(dst++) = '\0';

    strcat(standPath, "Standmotion/sector/29.ini");
    fstream fin;
    fin.open(standPath, ios::in);
    if(!fin)
    {
        ROS_INFO("Filename Error!!");
    }
    else
    {
        ROS_INFO("Filename Correct!!");
        try
        {
            packagecnt = tool->readvalue(fin, "PackageCnt", 0);
            StandPackage.push_back(tool->readvalue(fin, "Package", 2));
            for(int i = 1; i < packagecnt; i++)
            {
                StandPackage.push_back(tool->readvalue(fin, "|", 3));
            }
            ROS_INFO("End_LoadSector");
        }
        catch(exception e)
        {
        }
        int M12 = 2048 - (StandPackage[47] + StandPackage[48] * 256);
        int M13 = 2048 - (StandPackage[51] + StandPackage[52] * 256);
        int M14 = 2048 - (StandPackage[55] + StandPackage[56] * 256);
        int M18 = 2048 - (StandPackage[71] + StandPackage[72] * 256);
        int M19 = 2048 - (StandPackage[75] + StandPackage[76] * 256);
        int M20 = 2048 - (StandPackage[79] + StandPackage[80] * 256);

        double theta1 = (abs(M14) + abs(M20)) * MOTORDEG / 2;
        double theta2 = (abs(M13) + abs(M19)) * MOTORDEG / 2;
        double theta3 = (abs(M12) + abs(M18)) * MOTORDEG / 2;

        double Robot_Height_1 = L_Calf * cos(theta1 * DEG2RAD);
        double Robot_Height_2 = L_Thigh * cos((theta2 - theta1) * DEG2RAD);
        double Robot_Height_3 = L_Body * cos((theta3 + L_BodyError - theta2 + theta1) * DEG2RAD);
        double Robot_Width_1 = L_Calf * sin(theta1 * DEG2RAD);
        double Robot_Width_2 = L_Thigh * sin((theta2 - theta1) * DEG2RAD);
        double Robot_Width_3 = L_Body * sin((theta3 + L_BodyError - theta2 + theta1) * DEG2RAD);

        camera_angle_offest = theta1 + theta3 - theta2;
        ROS_INFO("theta1 = %f, theta2 = %f, theta3 = %f", theta1, theta2, theta3);
        ROS_INFO("camera_angle_offest = %f",camera_angle_offest);
        ROS_INFO("Robot_Height_1 = %f, Robot_Height_2 = %f, Robot_Height_3 = %f", Robot_Height_1, Robot_Height_2, Robot_Height_3);
        RobotHeight = L_Shoes + L_FOOT + Robot_Height_1 + Robot_Height_2 + Robot_Height_3;
        RobotWidth = Robot_Width_1 - Robot_Width_2 + Robot_Width_3;
        ROS_INFO("Robot_Width_1 = %f, Robot_Width_2 = %f, Robot_Width_3 = %f", Robot_Width_1, Robot_Width_2, Robot_Width_3);
        ROS_INFO("RobotHeight = %f, RobotWidth = %f",RobotHeight, RobotWidth);
        fin.close();
    }
    StandPackage.clear();
}