#include <ModelBase/ModelBase.h>

ModelBase* Model_Base = new ModelBase();

ModelBase::ModelBase()
{
//     ImageSize = IMAGEHEIGHT * IMAGEWIDTH * 3;
//     this->bmpSample = new unsigned char [ ColorDeep * ColorDeep * ColorDeep ];
//     this->TableBMPtoHSV = new unsigned char[ ColorDeep * ColorDeep * ColorDeep * 3];
//     this->Label_Model = new unsigned char [ IMAGEHEIGHT * IMAGEWIDTH];
//     this->BuiltTable = false;
//     this->ViewColorModel = true;
//     this->isBuildModel = false;
//     this->ColorSelected = 0;
// //-------------TdataUnit-----------------
//     this->strategyname = 8;
//     this->HSVColorRange = new ColorRange* [DefLabelMarkSize];
//     for(int ColorRangeCnt = 0; ColorRangeCnt < DefLabelMarkSize; ColorRangeCnt++)
//     {
//         this->HSVColorRange[ColorRangeCnt] = new ColorRange;
//         this->HSVColorRange[ColorRangeCnt]->HueMax = 1.0;
//         this->HSVColorRange[ColorRangeCnt]->HueMin = 0.0;
//         this->HSVColorRange[ColorRangeCnt]->SaturationMax = 1.0;
//         this->HSVColorRange[ColorRangeCnt]->SaturationMin = 0.0;
//         this->HSVColorRange[ColorRangeCnt]->BrightnessMax = 1.0;
//         this->HSVColorRange[ColorRangeCnt]->BrightnessMin = 0.0;
//         switch (ColorRangeCnt) {
//         case 0:
//             this->HSVColorRange[ColorRangeCnt]->LabelName = "[Orange Range]";
//             break;
//         case 1:
//             this->HSVColorRange[ColorRangeCnt]->LabelName = "[Yellow Range]";
//             break;
//         case 2:
//             this->HSVColorRange[ColorRangeCnt]->LabelName = "[Blue Range]";
//             break;
//         case 3:
//             this->HSVColorRange[ColorRangeCnt]->LabelName = "[Green Range]";
//             break;
//         case 4:
//             this->HSVColorRange[ColorRangeCnt]->LabelName = "[Black Range]";
//             break;
//         case 5:
//             this->HSVColorRange[ColorRangeCnt]->LabelName = "[Red Range]";
//             break;
//         case 6:
//             this->HSVColorRange[ColorRangeCnt]->LabelName = "[White Range]";
//             break;
//         case 7:
//             this->HSVColorRange[ColorRangeCnt]->LabelName = "[Other Range]";
//             break;
//         default:
//             break;
//         }
//     }
//---------------------------------------

//----------TColorFunctionUnit-----------
    // this->maxHSV = 0.0;
    // this->minHSV = 0.0;

    // this->RtmpValue = 0.0;
    // this->GtmpValue = 0.0;
    // this->BtmpValue = 0.0;

    // this->HtmpValue = 0.0;
    // this->StmpValue = 0.0;
    // this->VtmpValue = 0.0;

//-----------BGR threshold------------------
    BGRColorRange = new BGRRange;
    BGRColorRange->BuValue = 0;
    BGRColorRange->GrValue = 0;
    BGRColorRange->ReValue = 0;
    BGRColorRange->ParameterName = "[BGRColorRange]";
//-----------BGR threshold------------------
    houghrange = new HoughRange;
    houghrange->hough_threshold = 0;
    houghrange->hough_minLineLength = 0;
    houghrange->hough_maxLineGap = 0;
    houghrange->ParameterName = "[HoughRange]";
    tool = ToolInstance::getInstance();

}
ModelBase::~ModelBase()
{
    delete [] this->bmpSample;
    delete [] this->Label_Model;

    for(int ColorRangeCnt = 0; ColorRangeCnt < DefLabelMarkSize; ColorRangeCnt++)
    {
        delete [] this->HSVColorRange[ColorRangeCnt];
    }
    delete [] this->HSVColorRange;
    delete BGRColorRange;
    delete houghrange;
}

// void ModelBase::HSV_BuildingTable()
// {
//     unsigned char *tmpHere;
//     float rValue, gValue, bValue;

//     tmpHere = this->TableBMPtoHSV;

//     for(int rValue_tmp = 0; rValue_tmp < ColorDeep; rValue_tmp++){
//         for( int gValue_tmp = 0; gValue_tmp < ColorDeep; gValue_tmp++ ){
//             for( int bValue_tmp = 0; bValue_tmp < ColorDeep; bValue_tmp++ ){
//                 rValue = float( rValue_tmp / 255.0 );
//                 gValue = float( gValue_tmp / 255.0 );
//                 bValue = float( bValue_tmp / 255.0 );

//                 *tmpHere++ = (unsigned char)( Model_Base->HofRGBtoHSV( float( rValue ), float( gValue ), float( bValue ) ) / 360.0 * 255.0 );
//                 *tmpHere++ = (unsigned char)( Model_Base->SofRGBtoHSV( float( rValue ), float( gValue ), float( bValue ) ) * 255.0 );
//                 *tmpHere++ = (unsigned char)( Model_Base->VofRGBtoHSV( float( rValue ), float( gValue ), float( bValue ) ) * 255.0 );
//             }
//         }
//     }
//     this->BuiltTable = true;
//     ROS_INFO("BuildingTable");
// }

// void ModelBase::HSV_BuildingColorModel()
// {
//     unsigned char *tmpHere;
//     float hValue, sValue, vValue;
//     float rValue, gValue, bValue;

//     tmpHere = this->bmpSample;
//     for( int rValue_tmp = 0; rValue_tmp < ColorDeep; rValue_tmp++ ){
//         for( int gValue_tmp = 0; gValue_tmp < ColorDeep; gValue_tmp++ ){
//             for( int bValue_tmp = 0; bValue_tmp < ColorDeep; bValue_tmp++ ){
//                 rValue = float( rValue_tmp / 255.0 );
//                 gValue = float( gValue_tmp / 255.0 );
//                 bValue = float( bValue_tmp / 255.0 );

//                 hValue = float( Model_Base->HofRGBtoHSV(float( rValue ), float( gValue ), float( bValue ) ) / 360.0 );
//                 sValue = Model_Base->SofRGBtoHSV(float(rValue), float(gValue), float(bValue));
//                 vValue = Model_Base->VofRGBtoHSV(float(rValue), float(gValue), float(bValue));

//                 if( this->HSV_hsvCheckRange( Model_Base->HSVColorRange[ (int)LabelModel::Yellow], hValue, sValue, vValue ) )
//                     *tmpHere++ = (int)LabelMark::YellowLabel;
//                 else if( this->HSV_hsvCheckRange( Model_Base->HSVColorRange[ (int)LabelModel::Blue], hValue, sValue, vValue ) )
//                     *tmpHere++ = (int)LabelMark::BlueLabel;
//                 else if( this->HSV_hsvCheckRange( Model_Base->HSVColorRange[ (int)LabelModel::Orange], hValue, sValue, vValue ) )
//                     *tmpHere++ = (int)LabelMark::OrangeLabel;
//                 else if( this->HSV_hsvCheckRange( Model_Base->HSVColorRange[ (int)LabelModel::Green], hValue, sValue, vValue ) )
//                     *tmpHere++ = (int)LabelMark::GreenLabel;
//                 else if( this->HSV_hsvCheckRange( Model_Base->HSVColorRange[ (int)LabelModel::White], hValue, sValue, vValue ) )
//                     *tmpHere++ = (int)LabelMark::WhiteLabel;
//                 else if( this->HSV_hsvCheckRange( Model_Base->HSVColorRange[ (int)LabelModel::Black], hValue, sValue, vValue ) )
//                     *tmpHere++ = (int)LabelMark::BlackLabel;
//                 else if( this->HSV_hsvCheckRange( Model_Base->HSVColorRange[ (int)LabelModel::Red], hValue, sValue, vValue ) )
//                     *tmpHere++ = (int)LabelMark::RedLabel;
//                 else if( this->HSV_hsvCheckRange( Model_Base->HSVColorRange[ (int)LabelModel::Other], hValue, sValue, vValue ) )
//                     *tmpHere++ = (int)LabelMark::OthersLabel;
//                 else
//                     *tmpHere++ = 0x00;
//             }
//         }
//     }
//     *this->bmpSample = 0x00;
//     ROS_INFO("BuildingColorModel");
// }

// void ModelBase::ColorModeling(cv::Mat pImage)
// {
//     unsigned char rValue, gValue, bValue;
//     unsigned char rMask, gMask, bMask;
//     float hValue, sValue, vValue;
//     unsigned char *tmpTable;

//     switch (this->ColorSelected)
//     {
//     case  (int)LabelModel::Black:
//         rMask = 255;
//         gMask = 0;
//         bMask = 255;
//         break;
//     case  (int)LabelModel::Blue:
//         rMask = 128;
//         gMask = 0;
//         bMask = 128;
//         break;
//     case  (int)LabelModel::Green:
//         rMask = 128;
//         gMask = 0;
//         bMask = 0;
//         break;
//     case  (int)LabelModel::Orange:
//         rMask = 0;
//         gMask = 0;
//         bMask = 128;
//         break;
//     case  (int)LabelModel::Red:
//         rMask = 0;
//         gMask = 255;
//         bMask = 255;
//         break;
//     case  (int)LabelModel::Yellow:
//         rMask = 0;
//         gMask = 128;
//         bMask = 128;
//         break;
//     case  (int)LabelModel::White:
//         rMask = 255;
//         gMask = 255;
//         bMask = 0;
//         break;
//     case  (int)LabelModel::Other:
//         rMask = 128;
//         gMask = 0;
//         bMask = 255;
//         break;
//     default:
//         break;
//     }
//     for( int HeightCnt  = 0; HeightCnt < IMAGEHEIGHT; HeightCnt++ )
//     {
//         for( int WidthCnt = 0; WidthCnt < IMAGEWIDTH; WidthCnt++ )
//         {
//             bValue = *(pImage.data + ((HeightCnt*IMAGEWIDTH + WidthCnt) * 3 + 0));
//             gValue = *(pImage.data + ((HeightCnt*IMAGEWIDTH + WidthCnt) * 3 + 1));
//             rValue = *(pImage.data + ((HeightCnt*IMAGEWIDTH + WidthCnt) * 3 + 2));
//             tmpTable = this->TableBMPtoHSV + 3 * (rValue * ColorDeep * ColorDeep + gValue * ColorDeep + bValue);

//             hValue = ((float)(*tmpTable++) / 255);
//             sValue = ((float)(*tmpTable++) / 255);
//             vValue = ((float)(*tmpTable++) / 255);

//             if(this->HSV_hsvCheckRange(this->hsvColorRange, hValue, sValue, vValue))
//             {
//                 *(pImage.data + ((HeightCnt*IMAGEWIDTH + WidthCnt) * 3 + 0)) = bMask;
//                 *(pImage.data + ((HeightCnt*IMAGEWIDTH + WidthCnt) * 3 + 1)) = gMask;
//                 *(pImage.data + ((HeightCnt*IMAGEWIDTH + WidthCnt) * 3 + 2)) = rMask;
//             }
//         }
//     }
// }

// void ModelBase::ChangeToColorModel(cv::Mat pImage)
// {
//     unsigned char *rValue, *gValue, *bValue;
//     unsigned char *tmpValue;
//     for( int HeightCnt  = 0; HeightCnt < IMAGEHEIGHT; HeightCnt++ )
//     {
//         for( int WidthCnt = 0; WidthCnt < IMAGEWIDTH; WidthCnt++ )
//         {
//             bValue = (pImage.data + ((HeightCnt*IMAGEWIDTH + WidthCnt) * 3 + 0));
//             gValue = (pImage.data + ((HeightCnt*IMAGEWIDTH + WidthCnt) * 3 + 1));
//             rValue = (pImage.data + ((HeightCnt*IMAGEWIDTH + WidthCnt) * 3 + 2));

//             tmpValue = this->bmpSample + (*rValue * ColorDeep * ColorDeep + *gValue * ColorDeep + *bValue);
//             if(*tmpValue == (int)LabelMark::BlackLabel)
//             {
//                 *bValue = 255;
//                 *gValue = 0;
//                 *rValue = 255;
//                 *((this->Label_Model)+(HeightCnt*IMAGEWIDTH + WidthCnt)) = (int)LabelMark::BlackLabel;
//             }else if(*tmpValue == (int)LabelMark::BlueLabel){
//                 *bValue = 128;
//                 *gValue = 0;
//                 *rValue = 128;
//                 *((this->Label_Model)+(HeightCnt*IMAGEWIDTH + WidthCnt)) = (int)LabelMark::BlueLabel;
//             }else if(*tmpValue == (int)LabelMark::GreenLabel){
//                 *bValue = 0;
//                 *gValue = 0;
//                 *rValue = 128;
//                 *((this->Label_Model)+(HeightCnt*IMAGEWIDTH + WidthCnt)) = (int)LabelMark::GreenLabel;
//             }else if(*tmpValue == (int)LabelMark::OrangeLabel){
//                 *bValue = 128;
//                 *gValue = 0;
//                 *rValue = 0;
//                 *((this->Label_Model)+(HeightCnt*IMAGEWIDTH + WidthCnt)) = (int)LabelMark::OrangeLabel;
//             }else if(*tmpValue == (int)LabelMark::RedLabel){
//                 *bValue = 255;
//                 *gValue = 255;
//                 *rValue = 0;
//                 *((this->Label_Model)+(HeightCnt*IMAGEWIDTH + WidthCnt)) = (int)LabelMark::RedLabel;
//             }else if(*tmpValue == (int)LabelMark::YellowLabel){
//                 *bValue = 128;
//                 *gValue = 128;
//                 *rValue = 0;
//                 *((this->Label_Model)+(HeightCnt*IMAGEWIDTH + WidthCnt)) = (int)LabelMark::YellowLabel;
//             }else if(*tmpValue == (int)LabelMark::WhiteLabel){
//                 *bValue = 0;
//                 *gValue = 255;
//                 *rValue = 255;
//                 *((this->Label_Model)+(HeightCnt*IMAGEWIDTH + WidthCnt)) = (int)LabelMark::WhiteLabel;
//             }else if(*tmpValue == (int)LabelMark::OthersLabel){
//                 *bValue = 255;
//                 *gValue = 0;
//                 *rValue = 128;
//                 *((this->Label_Model)+(HeightCnt*IMAGEWIDTH + WidthCnt)) = (int)LabelMark::OthersLabel;
//             }else{
//                 *bValue = 0;
//                 *gValue = 0;
//                 *rValue = 0;
//                 *((this->Label_Model)+(HeightCnt*IMAGEWIDTH + WidthCnt)) = 0x00;
//             }
// //            this->labelmodel.LabelModel.push_back(*((this->Label_Model)+(HeightCnt*IMAGEWIDTH + WidthCnt)));
//         }
//     }
// }

// void ModelBase::ChangeToColorModel_ImageProcess(cv::Mat pImage)
// {
//     unsigned char *rValue, *gValue, *bValue;
//     unsigned char *tmpValue;
//     for( int HeightCnt  = 0; HeightCnt < IMAGEHEIGHT; HeightCnt++ )
//     {
//         for( int WidthCnt = 0; WidthCnt < IMAGEWIDTH; WidthCnt++ )
//         {
//             bValue = (pImage.data + ((HeightCnt*IMAGEWIDTH + WidthCnt) * 3 + 0));
//             gValue = (pImage.data + ((HeightCnt*IMAGEWIDTH + WidthCnt) * 3 + 1));
//             rValue = (pImage.data + ((HeightCnt*IMAGEWIDTH + WidthCnt) * 3 + 2));
//             tmpValue = this->Label_Model + HeightCnt * IMAGEWIDTH + WidthCnt;
//             this->labelmodel.LabelModel.push_back(*tmpValue);
//             if(*tmpValue == (int)LabelMark::BlackLabel)
//             {
//                 *bValue = 255;
//                 *gValue = 0;
//                 *rValue = 255;
//             }else if(*tmpValue == (int)LabelMark::BlueLabel){
//                 *bValue = 128;
//                 *gValue = 0;
//                 *rValue = 128;
//             }else if(*tmpValue == (int)LabelMark::GreenLabel){
//                 *bValue = 0;
//                 *gValue = 0;
//                 *rValue = 128;
//             }else if(*tmpValue == (int)LabelMark::OrangeLabel){
//                 *bValue = 128;
//                 *gValue = 0;
//                 *rValue = 0;
//             }else if(*tmpValue == (int)LabelMark::RedLabel){
//                 *bValue = 255;
//                 *gValue = 255;
//                 *rValue = 0;
//             }else if(*tmpValue == (int)LabelMark::YellowLabel){
//                 *bValue = 128;
//                 *gValue = 128;
//                 *rValue = 0;
//             }else if(*tmpValue == (int)LabelMark::WhiteLabel){
//                 *bValue = 0;
//                 *gValue = 255;
//                 *rValue = 255;
//             }else if(*tmpValue == (int)LabelMark::OthersLabel){
//                 *bValue = 255;
//                 *gValue = 0;
//                 *rValue = 128;
//             }
//         }
//     }
// }

// bool ModelBase::HSV_hsvCheckRange(ColorRange *hsvRange, float &hValue, float &sValue, float &vValue)
// {

//     if( hsvRange->HueMax >= hsvRange->HueMin ){
//         if( hsvRange->HueMax >= hValue && hsvRange->HueMin <= hValue
//                 && hsvRange->SaturationMax >= sValue && hsvRange->SaturationMin <= sValue
//                 && hsvRange->BrightnessMax >= vValue && hsvRange->BrightnessMin <= vValue )
//             return true;
//     }else{
//         if( ( hsvRange->HueMax >= hValue || hsvRange->HueMin <= hValue )
//                 && hsvRange->SaturationMax >= sValue && hsvRange->SaturationMin <= sValue
//                 && hsvRange->BrightnessMax >= vValue && hsvRange->BrightnessMin <= vValue )
//             return true;
//     }

//     return false;
// }

// void ModelBase::SaveColorRangeFile()
// {
//     char path[200];
//     strcpy(path, tool->getPackagePath("strategy").c_str());
//     strcat(path, "/ColorModelData.ini");
//     try
//     {
// //        ofstream OutFile(sFileName.c_str());
//         ofstream OutFile(path);
//         for(int ColorRangeCnt = 0; ColorRangeCnt < DefLabelMarkSize; ColorRangeCnt++)
//         {
//             OutFile << this->HSVColorRange[ColorRangeCnt]->LabelName;
//             OutFile << "\n";
//             OutFile << "Hue_Max = ";
//             OutFile << this->HSVColorRange[ColorRangeCnt]->HueMax;
//             OutFile << "\n";
//             OutFile << "Hue_Min = ";
//             OutFile << this->HSVColorRange[ColorRangeCnt]->HueMin;
//             OutFile << "\n";
//             OutFile << "Saturation_Max = ";
//             OutFile << this->HSVColorRange[ColorRangeCnt]->SaturationMax;
//             OutFile << "\n";
//             OutFile << "Saturation_Min = ";
//             OutFile << this->HSVColorRange[ColorRangeCnt]->SaturationMin;
//             OutFile << "\n";
//             OutFile << "Brightness_Max = ";
//             OutFile << this->HSVColorRange[ColorRangeCnt]->BrightnessMax;
//             OutFile << "\n";
//             OutFile << "Brightness_Min = ";
//             OutFile << this->HSVColorRange[ColorRangeCnt]->BrightnessMin;
//             OutFile << "\n";
//         }
//         OutFile.close();
//     }
//     catch( exception e )
//     {
//     }
// }

// void ModelBase::LoadColorRangeFile()
// {
//     fstream fin;
//     char line[100]; 
//     char path[200];
//     strcpy(path, tool->getPackagePath("strategy").c_str());
//     strcat(path, "/ColorModelData.ini");

//     fin.open(path, ios::in);
//     //fin.open(("../../Parameter/Color_Model_Data/ColorModelData.ini"), ios::in);
//     try
//     {
//         for(int ColorCnt = 0; ColorCnt < 8; ColorCnt++)
//         {
//             fin.getline(line,sizeof(line),'\n');
//             HSVColorRange[ColorCnt]->HueMax = tool->readvalue(fin, "Hue_Max", 1);
//             HSVColorRange[ColorCnt]->HueMin = tool->readvalue(fin, "Hue_Min", 1);
//             HSVColorRange[ColorCnt]->SaturationMax = tool->readvalue(fin, "Saturation_Max", 1);
//             HSVColorRange[ColorCnt]->SaturationMin = tool->readvalue(fin, "Saturation_Min", 1);
//             HSVColorRange[ColorCnt]->BrightnessMax = tool->readvalue(fin, "Brightness_Max", 1);
//             HSVColorRange[ColorCnt]->BrightnessMin = tool->readvalue(fin, "Brightness_Min", 1);
//         }
//         fin.close();
//     }
//     catch(exception e)
//     {
//     }
// }

// void ModelBase::ExecuteMaxMin(float &rValue, float &gValue, float &bValue)
// {
//     if(rValue > gValue)                     //R > G
//     {
//         if(rValue > bValue)                 //R > G && R > B
//         {
//             this->maxHSV = rValue;
//             if(gValue > bValue)
//                 this->minHSV = bValue;      //R > G > B
//             else
//                 this->minHSV = gValue;      //R > B > G
//         }
//         else                                //B > R > G
//         {
//             this->maxHSV = bValue;
//             this-> minHSV = gValue;
//         }
//     }
//     else                                    //G > R
//     {
//         if(gValue > bValue)                 //G > R && G > B
//         {
//             this->maxHSV = gValue;
//             if(rValue > bValue)
//                 this->minHSV = bValue;      //G > R > B
//             else
//                 this->minHSV = rValue;      //G > B > R
//         }
//         else                                //B > G > R
//         {
//             this->maxHSV = bValue;
//             this->minHSV = rValue;
//         }
//     }
// }
// //-----------------------HSVtoRGB-------------------
// void ModelBase::HSVtoRGB(float hValue, float sValue, float vValue)
// {
//     if(sValue == 0.0)
//     {
//         this->RtmpValue = vValue;
//         this->GtmpValue = vValue;
//         this->BtmpValue = vValue;
//     }
//     else
//     {
//         int HueInt;
//         float Huefloat;
//         float pValue, qValue, tValue;

//         HueInt = ((int)hValue / 60) % 6;
//         Huefloat = (float)((hValue / 60.0) - (float)HueInt);
//         pValue = (float)(vValue * (1.0 - sValue));
//         qValue = (float)(vValue * (1.0 - Huefloat * sValue));
//         tValue = (float)(vValue * (1.0 - (1.0 - Huefloat) * sValue));

//         switch(HueInt)
//         {
//             case 0:
//             {
//                 this->RtmpValue = vValue;
//                 this->GtmpValue = tValue;
//                 this->BtmpValue = pValue;
//                 break;
//             }
//             case 1:
//             {
//                 this->RtmpValue = qValue;
//                 this->GtmpValue = vValue;
//                 this->BtmpValue = pValue;
//                 break;
//             }
//             case 2:
//             {
//                 this->RtmpValue = pValue;
//                 this->GtmpValue = vValue;
//                 this->BtmpValue = tValue;
//                 break;
//             }
//             case 3:
//             {
//                 this->RtmpValue = pValue;
//                 this->GtmpValue = qValue;
//                 this->BtmpValue = vValue;
//                 break;
//             }
//             case 4:
//             {
//                 this->RtmpValue = tValue;
//                 this->GtmpValue = pValue;
//                 this->BtmpValue = vValue;
//                 break;
//             }
//             case 5:
//             {
//                 this->RtmpValue = vValue;
//                 this->GtmpValue = pValue;
//                 this->BtmpValue = qValue;
//                 break;
//             }
//         }
//     }
//     this->HtmpValue = hValue;
//     this->StmpValue = sValue;
//     this->VtmpValue = vValue;
// }
// //----------(R,G,B) of HSV to RGB-------------
// float ModelBase::RofHSVtoRGB(float hValue, float sValue, float vValue)
// {
//     if(hValue == this->HtmpValue && sValue == this->StmpValue && vValue == this->VtmpValue)
//         return this->RtmpValue;
//     this->HSVtoRGB(hValue, sValue, vValue);
//     return this->RtmpValue;
// }

// float ModelBase::GofHSVtoRGB(float hValue, float sValue, float vValue)
// {
//     if(hValue == this->HtmpValue && sValue == this->StmpValue && vValue == this->VtmpValue)
//         return this->GtmpValue;
//     this->HSVtoRGB(hValue, sValue, vValue);
//     return this->GtmpValue;
// }

// float ModelBase::BofHSVtoRGB(float hValue, float sValue, float vValue)
// {
//     if(hValue == this->HtmpValue && sValue == this->StmpValue && vValue == this->VtmpValue)
//         return this->BtmpValue;
//     this->HSVtoRGB(hValue, sValue, vValue);
//     return this->BtmpValue;
// }
// //--------------------------------------------
// //-----------(H,S,V) of RGB to HSV------------
// float ModelBase::HofRGBtoHSV(float rValue, float gValue, float bValue)
// {
//     this->ExecuteMaxMin(rValue, gValue, bValue);
//     float HueValue;

//     if(this->maxHSV == this->minHSV)
//         return 0.0;
//     if(rValue == this->maxHSV)
//     {
//         HueValue = 60.0 * ((gValue - bValue) / (this->maxHSV - this->minHSV));
//         if(gValue < bValue)
//             HueValue += 360.0;
//     }
//     else if(gValue == this->maxHSV)
//         HueValue = (float)(60.0 * ((bValue - rValue) / (this->maxHSV - this->minHSV)) + 120.0);
//     else
//         HueValue = (float)(60.0 * ((rValue - gValue) / (this->maxHSV - this->minHSV)) + 240.0);
//     if(HueValue >= 360.0)
//         HueValue -= 360.0;
//     if(HueValue < 0.0)
//         HueValue += 360.0;
//     return HueValue;
// }

// float ModelBase::SofRGBtoHSV(float rValue, float gValue, float bValue)
// {
//     this->ExecuteMaxMin(rValue, gValue, bValue);
//     return ((this->maxHSV == 0.0) || (this->maxHSV == this->minHSV)) ? 0:((this->maxHSV - this->minHSV) / this->maxHSV);
// }

// float ModelBase::VofRGBtoHSV(float rValue, float gValue, float bValue)
// {
//     this->ExecuteMaxMin(rValue, gValue, bValue);
//     return this->maxHSV;
// }

//-----------BGR threshold-------------------

void ModelBase::LoadBGRFile()
{
    // BGRColorRange->RValue = 0.0;
    // BGRColorRange->GValue = 0.0;
    // BGRColorRange->BValue = 0.0;
    fstream fin;
    char line[100]; 
    char path[200];
    std::string PATH = tool->getPackagePath("strategy");
    strcpy(path, PATH.c_str());
    strcat(path, "/BGR_Value.ini");
    
    fin.open(path, ios::in);
    //fin.open(("../../Parameter/Color_Model_Data/ColorModelData.ini"), ios::in);
    try
    {
        // ROS_INFO("2 path = %s",path);
        fin.getline(line,sizeof(line),'\n');
        BGRColorRange->BuValue = tool->readvalue(fin, "BValue", 0);
        BGRColorRange->GrValue = tool->readvalue(fin, "GValue", 0);
        BGRColorRange->ReValue = tool->readvalue(fin, "RValue", 0);
        // ROS_INFO("B = %f G = %f R = %f",BGRColorRange->BuValue,BGRColorRange->GrValue,BGRColorRange->ReValue);

        fin.close();
    }
    catch(exception e)
    {
    }
}
void ModelBase::SaveBGRFile()
{
    char path[200];
    printf("%s",path);
    std::string PATH = tool->getPackagePath("strategy");
    strcpy(path, PATH.c_str());
    strcat(path, "/BGR_Value.ini");
    try
    {
//       ofstream OutFile(sFileName.c_str());
        ofstream OutFile(path);
        OutFile << BGRColorRange->ParameterName;
        OutFile << "\n";
        OutFile << "BValue = ";
        OutFile << BGRColorRange->BuValue;
        OutFile << "\n";
        OutFile << "GValue = ";
        OutFile << BGRColorRange->GrValue;
        OutFile << "\n";
        OutFile << "RValue = ";
        OutFile << BGRColorRange->ReValue;
        OutFile << "\n";
        OutFile.close();
    }
    catch( exception e )
    {
    }
}

//-----------Hough threshold-------------------

void ModelBase::LoadHoughFile()
{
    fstream fin;
    char line[100]; 
    char path[200];
    std::string PATH = tool->getPackagePath("strategy");
    strcpy(path, PATH.c_str());
    strcat(path, "/Hough_Value.ini");
    
    fin.open(path, ios::in);
    try
    {
        // ROS_INFO("1 path = %s",path);
        fin.getline(line,sizeof(line),'\n');
        houghrange->hough_threshold = tool->readvalue(fin, "Hough_threshold", 0);
        houghrange->hough_minLineLength = tool->readvalue(fin, "Hough_minLineLength", 0);
        houghrange->hough_maxLineGap = tool->readvalue(fin, "Hough_maxLineGap", 0);
        // ROS_INFO("hough_threshold = %d hough_minLineLength = %d hough_maxLineGap = %d",houghrange->hough_threshold,houghrange->hough_minLineLength,houghrange->hough_maxLineGap);
        fin.close();
    }
    catch(exception e)
    {
    }
}
void ModelBase::SaveHoughFile()
{
    char path[200];
    printf("%s",path);
    std::string PATH = tool->getPackagePath("strategy");
    strcpy(path, PATH.c_str());
    strcat(path, "/Hough_Value.ini");
    try
    {
        ofstream OutFile(path);
        OutFile << houghrange->ParameterName;
        OutFile << "\n";
        OutFile << "Hough_threshold = ";
        OutFile << houghrange->hough_threshold;
        OutFile << "\n";
        OutFile << "Hough_minLineLength = ";
        OutFile << houghrange->hough_minLineLength;
        OutFile << "\n";
        OutFile << "Hough_maxLineGap = ";
        OutFile << houghrange->hough_maxLineGap;
        OutFile << "\n";
        OutFile.close();
    }
    catch( exception e )
    {
    }
}

