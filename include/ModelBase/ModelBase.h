#ifndef MODELBASE_H
#define MODELBASE_H

#define IMAGEHEIGHT 240
#define IMAGEWIDTH 320
#define DefLabelMarkSize 8
#define HueScrollBarMax 360
#define SaturationScrollBarMax 100
#define BrightnessScrollBarMax 100

#define ColorDeep 256
//----------------msgs----------------
#include "tku_msgs/LabelModelObjectList.h"
#include "tku_msgs/BGRValue.h"
#include "tku_msgs/BGRInfo.h"
//----------------libs----------------
#include "tku_libs/strategy_info.h"
#include "tku_libs/TKU_tool.h"
#include "tku_libs/RosCommunication.h"

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include "FeatureDistance/FeatureDistance.h"

using namespace std;
using namespace cv;
struct ColorRange
{
    float HueMin;
    float HueMax;
    float SaturationMin;
    float SaturationMax;
    float BrightnessMin;
    float BrightnessMax;

    string LabelName;
};

struct BGRRange
{
    int BuValue;
    int GrValue;
    int ReValue;
    string ParameterName;
};

class ModelBase : public FeatureDistance
{
    private:
        int ImageSize;
        unsigned char *bmpSample;
    //-----TColorFunctionUnit----------
        float RtmpValue, GtmpValue, BtmpValue;
        float HtmpValue, StmpValue, VtmpValue;
        float maxHSV, minHSV;
        void ExecuteMaxMin(float &rValue, float &gValue, float &bValue);
        void HSVtoRGB(float hValue, float sValue, float vValue);
    //---------------------------------
    public:
        ModelBase();
        ~ModelBase();

		ToolInstance *tool;
    
    //----------TModelUnit------------------
        ColorRange* hsvColorRange;
        unsigned char *TableBMPtoHSV;
        unsigned char *Label_Model;
        bool BuiltTable;
        bool isBuildModel;
        bool ViewColorModel;
        int ColorSelected;

        void HSV_BuildingTable();
        void HSV_BuildingColorModel();
        void ColorModeling(cv::Mat colorimage);         //while color modeling
        void ChangeToColorModel(cv::Mat pImage);        //before Erosion and Dilation
        void ChangeToColorModel_ImageProcess(cv::Mat pImage);   //after Erosion and Dilation
        bool HSV_hsvCheckRange( ColorRange *hsvRange, float &hValue, float &sValue, float &vValue );

        tku_msgs::LabelModelObjectList labelmodel;
    //--------------------------------------
    
    //--------TColorFunctionUnit------------
        //------HSV to RGB------//
        float RofHSVtoRGB(float hValue, float sValue, float vValue);
        float GofHSVtoRGB(float hValue, float sValue, float vValue);
        float BofHSVtoRGB(float hValue, float sValue, float vValue);
        //------RGB to HSV------//
        float HofRGBtoHSV(float rValue, float gValue, float bValue);
        float SofRGBtoHSV(float rValue, float gValue, float bValue);
        float VofRGBtoHSV(float rValue, float gValue, float bValue);
    //---------------------------------------

    //---------TdataUnit---------------------
        int strategyname;
        void SaveColorRangeFile();
        void LoadColorRangeFile();
        ColorRange** HSVColorRange;
    // ---------------------------------------
//    std_msgs::Int16MultiArray labelmodel;
    
    //---------BGR Test-------------
        void SaveBGRFile();
        void LoadBGRFile();
        BGRRange* BGRColorRange;
};
extern ModelBase* Model_Base;
#endif // TMODELUNIT_H