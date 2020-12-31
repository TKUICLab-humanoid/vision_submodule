#ifndef IMAGEMAIN_H
#define IMAGEMAIN_H

#include <ros/ros.h>
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
#include "tku_msgs/BGRValue.h"
#include "tku_msgs/HoughValue.h"
#include "tku_msgs/HoughInfo.h"
#include "tku_msgs/BuildModel.h"
#include "tku_msgs/SaveHSV.h"
#include "tku_msgs/ColorData.h"
#include "tku_msgs/ColorArray.h"
#include "tku_msgs/ObjectList.h"

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
        void GetImagesourceFunction(const sensor_msgs::ImageConstPtr& msg);
        void DepthCallback(const sensor_msgs::ImageConstPtr& depth_img);
        void ModelingFunction(const tku_msgs::ButtonColorForm& msg);
        void ChangeHSVValue(const tku_msgs::HSVValue& msg);
        bool LoadHSVInfo(tku_msgs::HSVInfo::Request &HSVreq, tku_msgs::HSVInfo::Response &HSVres);
        bool CallBuildFunction(tku_msgs::BuildModel::Request &req, tku_msgs::BuildModel::Response &res);
        bool CallSaveHSVFunction(tku_msgs::SaveHSV::Request &req, tku_msgs::SaveHSV::Response &res);
        void GetIMUDataFunction(const tku_msgs::SensorPackage &msg);
        void HeadAngleFunction(const tku_msgs::HeadPackage &msg);
        void ChangeBGRValue(const tku_msgs::BGRValue& msg);
        bool LoadBGRInfo(tku_msgs::BGRInfo::Request &req, tku_msgs::BGRInfo::Response &res);
        void LoadBGRValue();
        void ChangeHoughValue(const tku_msgs::HoughValue& msg);
        void LoadHoughValue();
        bool LoadHoughInfo(tku_msgs::HoughInfo::Request &req, tku_msgs::HoughInfo::Response &res);

        float B_,G_,R_;
        float threshold_,minLineLength_,maxLineGap_;
    private:
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

        ros::Subscriber Imagesource_subscriber;
        ros::Subscriber Depthimage_subscriber;
        ros::Subscriber HeadAngle_subscriber;
        ros::Subscriber IMUData_Subscriber;
        //--------------HSV---------------
        ros::Subscriber ModelingButton_subscriber;
        ros::Subscriber HSVValue_subscriber;
        ros::ServiceServer HSV_service;
        ros::ServiceServer Build_service;
        ros::ServiceServer Save_service;
        //--------------------------------

        //---------------BGR--------------
        ros::Subscriber BGRValue_subscriber;
        ros::ServiceServer BGR_service;
        
        //--------------Hough-------------
        ros::Subscriber HoughValue_subscriber;
        ros::ServiceServer Hough_service;
       
       
        ros::Publisher Distance_Publisher;
        ros::Publisher ObservationData_Publisher;
        ros::Publisher ImageLengthData_Publisher;
        ros::Publisher IMUDataRequest_Publisher; 
        ros::Publisher SoccerData_Publisher;


        // sensor_msgs::ImagePtr msg_blur;
        sensor_msgs::ImagePtr msg_imageGamma;
        // sensor_msgs::ImagePtr msg_nobackgroud;
        // sensor_msgs::ImagePtr msg_morphologyEx;
        // sensor_msgs::ImagePtr msg_edge;
        sensor_msgs::ImagePtr msg_mask;
        // sensor_msgs::ImagePtr msg_orihough;
        sensor_msgs::ImagePtr msg_hough;
        // image_transport::Publisher blur_Frame_Publisher;
        image_transport::Publisher Gamma_Frame_Publisher;
        image_transport::Publisher nobackgroud_Frame_Publisher;
        // image_transport::Publisher morphologyEx_Frame_Publisher;
        // image_transport::Publisher edge_Frame_Publisher;
        image_transport::Publisher Object_Frame_Publisher;
        image_transport::Publisher Monitor_Frame_Publisher;
        image_transport::Publisher Measure_Frame_Publisher;
        image_transport::Publisher mask_Frame_Publisher;
        // image_transport::Publisher OriHough_Publisher;
        image_transport::Publisher MerHough_Publisher;

    private:
        float pitch_pre;
        float roll_pre;
};

#endif // IMAGEMAIN_H
