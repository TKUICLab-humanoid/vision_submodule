#include <FeatureDistance/FeatureDistance.h>

FeatureDistance::FeatureDistance()
{  
    tool = ToolInstance::getInstance();
    //initialization  (the angle of head is 1350)                                                
    camera_height = 56.883;                 //攝影機高度

    camera2robot_dis = 0.0;                 //攝影機到機器人距離

    image_bottom_angle = 2.617;               //畫面最底與機器人之間夾角
    half_VFOV_angle = VFOV / 2;      //垂直視角一半
    half_HFOV_angle = HFOV / 2;    //水平視角一半
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

Distance FeatureDistance::measureLine(int Feature_x, int Feature_y) //LightLight
{
    // ROS_INFO("measure");
    // ROS_INFO("xyz_dis: %lf", xyz_dis);
    // ROS_INFO("x_dis: %lf", distance.x_dis);
    // ROS_INFO("y_dis: %lf", distance.y_dis);
    // ROS_INFO("dis: %lf\n\n", distance.dis);

    float width_cnt = (float)Feature_x;
    float height_cnt = (float)Feature_y;
    float error_y;
    float error_x;
    Distance distance;
    
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
            distance.y_dis = camera_height * tan(vertical_angle * DEG2RAD) + camera2robot_dis;
        }
        else
        {
            error_y = 240.0 - height_cnt;
            vertical_angle = image_bottom_angle + half_VFOV_angle + atan2(error_y , 640) * 180 / PI;
            distance.y_dis = camera_height * tan(vertical_angle * DEG2RAD) + camera2robot_dis;
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
    if(Horizontal_Head_Angle != 0)
    {
        if(Horizontal_Head_Angle < 0)
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

    return distance;
}

Distance FeatureDistance::measureObject(int Feature_x, int Feature_y)// Lee
{
    double width_cnt = (double)Feature_x;
    double height_cnt = (double)Feature_y;
    double error_x;
    double error_y;
    double x_ratio;
    double y_ratio;
    double ratio_angle;
    double xyz_dis;
    double x_dis;
    double y_dis;
    double xy_dis;
    double object_angle;
    Distance distance;

    if(Feature_x == -1 && Feature_y == -1)
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
            vertical_angle = image_bottom_angle + half_VFOV_angle - atan2(error_y , 240) * 180 / PI;
            y_ratio = camera_height * tan(vertical_angle * DEG2RAD);
        }
        else
        {
            error_y = 240.0 - height_cnt;
            vertical_angle = image_bottom_angle + half_VFOV_angle + atan2(error_y , 240) * 180 / PI;
            y_ratio = camera_height * tan(vertical_angle * DEG2RAD);
        }
        if(width_cnt > 320)
        {
            error_x = width_cnt - 320.0;
            horizontal_angle = atan2(error_x, image_center_horizontal_length) * 180 / PI;
            x_ratio = (camera_height / cos(vertical_angle * DEG2RAD)) * tan(horizontal_angle * DEG2RAD);
        }
        else
        {
            error_x = 320.0 - width_cnt;
            horizontal_angle = atan2(error_x, image_center_horizontal_length) * 180 / PI;
            x_ratio = -((camera_height / cos(vertical_angle * DEG2RAD)) * tan(horizontal_angle * DEG2RAD));
        }

        ROS_INFO("x_ratio: %lf", x_ratio);
        ROS_INFO("y_ratio: %lf\n", y_ratio);
        ratio_angle = atan2(x_ratio, y_ratio) * 180 / PI;
        if(!depth_buffer.empty())
        {
            ROS_INFO("start depth dis");
            ROS_INFO("Feature_x: %d", Feature_x);
            ROS_INFO("Feature_y: %d", Feature_y);
            xyz_dis = (depth_buffer.at<uint16_t>(Feature_y, Feature_x))*0.1 + 1.0 + 6.5 ;//獲取圖像座標Feature_y,Feature_x的深度值,單位是公分
            ROS_INFO("xyz_dis: %lf", xyz_dis);
            ROS_INFO("end depth dis\n");
        }
        x_dis = sqrt(pow(xyz_dis,2)-pow(camera_height-9.5,2)) * sin(ratio_angle * DEG2RAD);
        y_dis = sqrt(pow(xyz_dis,2)-pow(camera_height-9.5,2)) * cos(ratio_angle * DEG2RAD) + camera2robot_dis;
        xy_dis = sqrt(pow(x_dis,2)+pow(y_dis,2));
        object_angle = atan2(x_dis, y_dis) * 180 / PI;

        ROS_INFO("camera_height-9.5: %lf", camera_height-9.5);
        ROS_INFO("x_dis: %lf", x_dis);
        ROS_INFO("y_dis: %lf", y_dis);
        ROS_INFO("xy_dis: %lf\n", xy_dis);

        if(Horizontal_Head_Angle != 0)
        {
            y_dis = xy_dis * cos((-Horizontal_Head_Angle + object_angle) * DEG2RAD);
            x_dis = xy_dis * sin((-Horizontal_Head_Angle + object_angle) * DEG2RAD);
            ROS_INFO("Horizontal_Head_Angle: %lf", -Horizontal_Head_Angle);
            ROS_INFO("object_angle: %lf", object_angle);
            ROS_INFO("Angle: %lf\n", -Horizontal_Head_Angle + object_angle);
        }

        distance.x_dis = x_dis;
        distance.y_dis = y_dis;
        distance.dis = xy_dis;
    }

    return distance;
}

void FeatureDistance::calcImageAngle(motordata Horizontal_Head,motordata Vertical_Head)
{
    calcMotorAngle(Horizontal_Head.pos,Vertical_Head.pos);

    // Lee
    float Moving_angle_error = camera_angle_offest + (Robot_Pitch - Pitch_init);
    // ROS_INFO("Moving_angle_error = %f", Moving_angle_error);
    float foot2robot_dis;
    float foot2robot_angle;
    if((Robot_Pitch - Pitch_init) >= 0)
    {
        foot2robot_dis = sqrt(pow(RobotHeight,2)+pow(RobotWidth-7.5,2)); // front shoe
        foot2robot_angle = atan2(RobotHeight, RobotWidth-7.5) * 180 / PI - (Robot_Pitch - Pitch_init);
    }
    else
    {
        foot2robot_dis = sqrt(pow(RobotHeight,2)+pow(RobotWidth+6.5,2)); // behind shoe
        foot2robot_angle = atan2(RobotHeight, RobotWidth+6.5) * 180 / PI - (Robot_Pitch - Pitch_init);
    }
    camera_height = foot2robot_dis * sin(foot2robot_angle * DEG2RAD) + L_CAMERA * sin((Vertical_Head_Angle - Moving_angle_error) * DEG2RAD);
    // ROS_INFO("foot2robot_dis = %f", foot2robot_dis * sin(foot2robot_angle * DEG2RAD));
    // ROS_INFO("L_CAMERA = %f", L_CAMERA * sin((Vertical_Head_Angle - Moving_angle_error) * DEG2RAD));
    // ROS_INFO("camera_height = %f", camera_height);
    camera2robot_dis = RobotHeight * sin((Robot_Pitch - Pitch_init) * DEG2RAD) + L_CAMERA * cos((Vertical_Head_Angle - Moving_angle_error) * DEG2RAD);

    image_bottom_angle = Vertical_Head_Angle - Moving_angle_error - half_VFOV_angle - AVGERRORANGLE;
    image_top_length = camera2robot_dis + camera_height * tan((VFOV + image_bottom_angle) * DEG2RAD);
    image_bottom_length = camera2robot_dis + camera_height * tan(image_bottom_angle * DEG2RAD);
    image_top_width_length = (camera_height / cos((VFOV + image_bottom_angle) * DEG2RAD)) * tan(half_HFOV_angle * DEG2RAD);
    image_bottom_width_length = (camera_height / cos(image_bottom_angle * DEG2RAD)) * tan(half_HFOV_angle * DEG2RAD);

    /* LightLight
    float Moving_angle_error = -(camera_angle_offest) + Robot_Pitch - Pitch_init;
    camera_height = RobotHeight * cos((Robot_Pitch - Pitch_init) * DEG2RAD) + L_CAMERA * sin((Vertical_Head_Angle - Moving_angle_error) * DEG2RAD);
    // double aa = cos((Vertical_Head_Angle - Robot_Pitch + Pitch_init) * DEG2RAD) * CAMERA_HEIGHT;
    camera2robot_dis = RobotHeight * sin(Moving_angle_error * DEG2RAD) + L_CAMERA * cos((Vertical_Head_Angle - Moving_angle_error) * DEG2RAD) - 3;
    image_bottom_angle = Vertical_Head_Angle - half_VFOV_angle - AVGERRORANGLE - Moving_angle_error;

    image_top_length = camera2robot_dis + camera_height * tan((VFOV + image_bottom_angle) * DEG2RAD);
    image_bottom_length = camera2robot_dis + camera_height * tan(image_bottom_angle * DEG2RAD);
    image_top_width_length = (camera_height / cos((VFOV + image_bottom_angle) * DEG2RAD)) * tan(half_HFOV_angle * DEG2RAD);
    image_bottom_width_length = (camera_height / cos(image_bottom_angle * DEG2RAD)) * tan(half_HFOV_angle * DEG2RAD);
    */

    /*ROS_INFO("camera_height = %f",camera_height);
    ROS_INFO("camera2robot_dis = %f",camera2robot_dis);
    ROS_INFO("image_bottom_angle = %f",image_bottom_angle);
    ROS_INFO("RobotHeight = %f",RobotHeight);*/
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
    int length;
    int packagecnt;
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

        ROS_INFO("camera_angle_offest = %f",camera_angle_offest);

        RobotHeight = L_Shoes + L_FOOT + Robot_Height_1 + Robot_Height_2 + Robot_Height_3;
        RobotWidth = Robot_Width_1 - Robot_Width_2 + Robot_Width_3;
        ROS_INFO("RobotHeight = %f",RobotHeight);
        fin.close();
    }
    StandPackage.clear();
}
