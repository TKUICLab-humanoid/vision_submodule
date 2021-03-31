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

Distance FeatureDistance::measure(int Feature_x, int Feature_y,CameraType cameratype)
{
    float width_cnt = (float)Feature_x;
    float height_cnt = (float)Feature_y;
    float error_y;
    float error_x;
    float depth_scale = 0.1;//0.1為cm 1為mm
    float distance_sum = 0;
    int effective_pixel = 0;
    float effective_distance = 0;
    Size range = Size(4,4);
    Rect RectRange(Feature_x-range.width/2,Feature_y-range.height/2,range.width,range.height);
    Distance distance;
    if(RectRange.y < 0)
    {
        RectRange.y = 0;
    }
    if(RectRange.x < 0)
    {
        RectRange.x = 0;
    }
    if(RectRange.y >479)
    {
        RectRange.y = 473;
    }
    if(RectRange.x > 639)
    {       
        RectRange.y = 633;
    }
    //ROS_INFO("*************distances.x_dis = %d",distance.x_dis);
    //ROS_INFO("*************distances.y_dis = %d",distance.y_dis);
    //ROS_INFO("*************distances.dis = %d",distance.dis);  
    switch(cameratype)
    {
        case CameraType::stereo:
            //ROS_INFO("------------distances.x_dis = %d",distance.x_dis);
            //ROS_INFO("------------distances.y_dis = %d",distance.y_dis);
            //ROS_INFO("------------distances.dis = %d",distance.dis);  
            //ROS_INFO("stereo");
            
            if(!depth_buffer.empty())
            {
                for(int y=RectRange.y;y<RectRange.y+RectRange.height;y++){
                    for(int x=RectRange.x;x<RectRange.x+RectRange.width;x++){
                        //如果深度图下該點像素不為0，表示有距離信息
                        if(depth_buffer.at<uint16_t>(y,x)){
                            distance_sum+=depth_scale*depth_buffer.at<uint16_t>(y,x);
                            effective_pixel++;
                        }
                    }
                }         
                effective_distance = (distance_sum/float(effective_pixel));
                //ROS_INFO("distance_sum = %f , effective_pixel = %d ",distance_sum,effective_pixel);
                if(Feature_x == 320 && Feature_y ==240)
                {
                    // cout<<"effective_distance :"<<effective_distance<<" cm"<<endl;
                    // ROS_INFO("camera_height = %f",camera_height);
                    // ROS_INFO("RobotHeight = %f",RobotHeight);
                }
                //distance.dis = int(effective_distance);

                if(effective_distance > camera_height && effective_distance < 500)
                {
                    float camera_angle = acos(camera_height / effective_distance) * RAD2DEG - (round(AVGERRORANGLE) + (Vertical_Head_Angle-28.65)/8.785) ;
                    if(Feature_x == 320 && Feature_y ==240)
                    {
                        // ROS_INFO("AVGERRORANGLE + (Vertical_Head_Angle-28.65)/8.785 = %f, AVGERRORANGLE * (Vertical_Head_Angle/0.08789 - 1300)/100 = %f,Moving_angle_error = %f ,acos(camera_height / effective_distance) * RAD2DEG = %f",AVGERRORANGLE + (Vertical_Head_Angle-28.65)/8.785,AVGERRORANGLE * (Vertical_Head_Angle/0.08789 - 1300)/100,Moving_angle_error,acos(camera_height / effective_distance) * RAD2DEG); 
                        // ROS_INFO("camera_angle = %f, camera_angle_offest = %f , camera2robot_dis = %f",camera_angle,camera_angle_offest,+ camera2robot_dis); 
                    }
                    float dist = effective_distance * sin(camera_angle* DEG2RAD);
                    distance.x_dis = int(round(dist * sin(Horizontal_Head_Angle * DEG2RAD)));
                    distance.y_dis = int(round(dist * cos(Horizontal_Head_Angle * DEG2RAD)));
                    distance.dis = int(round(sqrt(pow(distance.x_dis,2)+pow(distance.y_dis,2))));
                    //ROS_INFO("effective_distance = %f , dist = %f ,camera_angle = %f, camera_height = %f",effective_distance,dist,camera_angle,camera_height);
                    //ROS_INFO("break");
                    //ROS_INFO("++++++++++++++distances.x_dis = %d",distance.x_dis);
                    //ROS_INFO("++++++++++++++distances.y_dis = %d",distance.y_dis);
                    //ROS_INFO("++++++++++++++distances.dis = %d",distance.dis);       
                }
                // ROS_INFO("stereo: x = %d, y = %d, dis = %d",distance.x_dis,distance.y_dis,distance.dis);
                break;
            }
 
        case CameraType::Monocular:

            //ROS_INFO("Monocular");
            //if(width_cnt == -1 && height_cnt == -1)
            //{

            //}
            //else
            //{
                if(height_cnt > 240)
                {
                    error_y = height_cnt - 240.0;
                    vertical_angle = image_bottom_angle + half_VFOV_angle - atan2(error_y , 640) * 180 / PI;
                   // ROS_INFO("error_y = %f, vertical_angle = %f, image_bottom_angle = %f, half_VFOV_angle = %f ",error_y,vertical_angle,image_bottom_angle,half_VFOV_angle);
                    distance.y_dis = int(round(camera_height * tan(vertical_angle * DEG2RAD) + camera2robot_dis));
                }
                else
                {
                    error_y = 240.0 - height_cnt;
                    vertical_angle = image_bottom_angle + half_VFOV_angle + atan2(error_y , 640) * 180 / PI;
                    if(Feature_x == 320 && Feature_y ==240)
                    {
                        // ROS_INFO("error_y = %f, vertical_angle = %f, image_bottom_angle = %f, half_VFOV_angle = %f,camera2robot_dis = %f ",error_y,vertical_angle,image_bottom_angle,half_VFOV_angle,camera2robot_dis);
                    }
                    
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
            //}
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
    float Moveing_angle_error = -(camera_angle_offest) + Robot_Pitch - Pitch_init;
    camera_height = RobotHeight * cos((Robot_Pitch - Pitch_init) * DEG2RAD) + sin((Vertical_Head_Angle - Moveing_angle_error) * DEG2RAD) * CAMERA_HEIGHT;
    //double aa = cos((Vertical_Head_Angle - Robot_Pitch + Pitch_init) * DEG2RAD) * CAMERA_HEIGHT;
    camera2robot_dis = RobotHeight * sin(Moveing_angle_error * DEG2RAD) + cos((Vertical_Head_Angle - Moveing_angle_error) * DEG2RAD) * CAMERA_HEIGHT - 3;
    image_bottom_angle = Vertical_Head_Angle - half_VFOV_angle - AVGERRORANGLE - Moveing_angle_error;

    image_top_length = camera2robot_dis + camera_height * tan((VFOV + image_bottom_angle) * DEG2RAD);
    image_bottom_length = camera2robot_dis + camera_height * tan(image_bottom_angle * DEG2RAD);
    image_top_width_length = (camera_height / cos((VFOV + image_bottom_angle) * DEG2RAD)) * tan(half_HFOV_angle * DEG2RAD);
    image_bottom_width_length = (camera_height / cos(image_bottom_angle * DEG2RAD)) * tan(half_HFOV_angle * DEG2RAD);

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
    char pathend[20] = "/sector/";
    char pathend2[20] = "29.ini";
    char path[200];
    int packagecnt;
    int cnt = 3;
    strcpy(path, tool->standPath);
    // ROS_INFO("tool->standPath = %s",tool->standPath);
    strcat(path, pathend);
    strcat(path, pathend2);
    fstream fin;
    fin.open(path, ios::in);
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
        M12 = abs(M12);
        int M13 = 2048 - (StandPackage[51] + StandPackage[52] * 256);
	    M13 = abs(M13);
        int M14 = 2048 - (StandPackage[55] + StandPackage[56] * 256);
	    M14 = abs(M14);
        int M18 = 2048 - (StandPackage[71] + StandPackage[72] * 256);
	    M18 = abs(M18);
        int M19 = 2048 - (StandPackage[75] + StandPackage[76] * 256);
        M19 = abs(M19);
        int M20 = 2048 - (StandPackage[79] + StandPackage[80] * 256);
        M20 = abs(M20);

        double theta1 = (M14 + M20) / 2 * MOTORDEG;
        double theta2 = (M13 + M19) / 2 * MOTORDEG;
        double theta3 = (M12 + M18) / 2 * MOTORDEG;

        double Robot_Height_1 = cos(theta1 * DEG2RAD) * SLEG_L;     //L * cos(theta1)
        double Robot_Height_2 = cos((theta1 - theta2) * DEG2RAD) * BLEG_L;      //L *cos(theta2-theta1)
        double Robot_Height_3 = cos((theta2 - theta1 - theta3) * DEG2RAD) * BODY_L;     //L *cos(theta2-theta1 - theta3)

        camera_angle_offest = theta2 - theta1 - theta3;

        ROS_INFO("camera_angle_offest = %f",camera_angle_offest);

        RobotHeight = Robot_Height_1 + Robot_Height_2 + Robot_Height_3 + FOOT_H - 1.0;
        ROS_INFO("RobotHeight = %f",RobotHeight);
    }
    StandPackage.clear();
}
