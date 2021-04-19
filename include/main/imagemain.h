#ifndef IMAGEMAIN_H
#define IMAGEMAIN_H

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <image_transport/image_transport.h>

#include "sensor_msgs/Image.h"
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
//-----------tku_msgs--------------
#include "tku_msgs/HeadPackage.h"
#include "tku_msgs/ObservationData.h"
#include "tku_msgs/FeaturePoint.h"
#include "tku_msgs/Distance.h"
#include "tku_msgs/ImageLengthData.h"
#include "tku_msgs/SensorSet.h"
#include "tku_msgs/SensorPackage.h"
#include "tku_msgs/SoccerData.h"
#include "tku_msgs/SoccerDataList.h"

#include "tku_msgs/ButtonColorForm.h"
#include "tku_msgs/HSVInfo.h"
#include "tku_msgs/HSVValue.h"
#include "tku_msgs/BuildModel.h"
#include "tku_msgs/SaveHSV.h"
#include "tku_msgs/ColorData.h"
#include "tku_msgs/ColorArray.h"
#include "tku_msgs/ObjectList.h"

#include "tku_libs/RosCommunication.h"
#include "LineDetected/LineDetected.h"

#define PI 3.14159265
#define DEG2RAD  M_PI/180

using namespace std;
using namespace cv;


class Vision_main : public LineDetected
{
    public:
        Vision_main(ros::NodeHandle &nh);
        ~Vision_main();
        void strategy_main();
        void strategy_init();
    public:
        void DepthCallback(const sensor_msgs::ImageConstPtr& depth_img);
        void GetImagesourceFunction(const sensor_msgs::ImageConstPtr& msg);
        void ModelingFunction(const tku_msgs::ButtonColorForm& msg);
        void ChangeHSVValue(const tku_msgs::HSVValue& msg);
        bool LoadHSVInfo(tku_msgs::HSVInfo::Request &HSVreq, tku_msgs::HSVInfo::Response &HSVres);
        bool CallBuildFunction(tku_msgs::BuildModel::Request &req, tku_msgs::BuildModel::Response &res);
        bool CallSaveHSVFunction(tku_msgs::SaveHSV::Request &req, tku_msgs::SaveHSV::Response &res);
        void GetIMUDataFunction(const tku_msgs::SensorPackage &msg);
        void HeadAngleFunction(const tku_msgs::HeadPackage &msg);
    private:
        RosCommunicationInstance *ros_com;

        motordata Horizontal_Head;
        motordata Vertical_Head;

        sensor_msgs::ImagePtr msg_object;
        sensor_msgs::ImagePtr msg_monitor;
        sensor_msgs::ImagePtr msg_measure;

        tku_msgs::ObservationData Observation_Data;
        tku_msgs::ImageLengthData ImageLengthData;
        tku_msgs::SensorSet IMUDataRequest;
        tku_msgs::SoccerDataList Soccer;

        ros::NodeHandle *nh;
        ros::NodeHandle color_nh;
        ros::NodeHandle depth_nh;
        ros::CallbackQueue color_queue;
        ros::CallbackQueue depth_queue;
        ros::AsyncSpinner *color_spinner;
        ros::AsyncSpinner *depth_spinner;

        ros::Subscriber Depthimage_subscriber;
        ros::Subscriber Imagesource_subscriber;
        ros::Subscriber HeadAngle_subscriber;
        ros::Subscriber IMUData_Subscriber;
        //--------------HSV---------------
        ros::Subscriber ModelingButton_subscriber;
        ros::Subscriber HSVValue_subscriber;
        ros::ServiceServer HSV_service;
        ros::ServiceServer Build_service;
        ros::ServiceServer Save_service;
        //--------------------------------
        ros::Publisher Distance_Publisher;

        ros::Publisher ObservationData_Publisher;
        ros::Publisher ImageLengthData_Publisher;
        ros::Publisher IMUDataRequest_Publisher; 
        ros::Publisher SoccerData_Publisher;

        image_transport::Publisher Object_Frame_Publisher;
        image_transport::Publisher Monitor_Frame_Publisher;
        image_transport::Publisher Measure_Frame_Publisher;
    private:
        bool processVisionFlag;
        float pitch_pre;
        float roll_pre;
};

#endif // IMAGEMAIN_H
