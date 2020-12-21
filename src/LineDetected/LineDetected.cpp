#include <LineDetected/LineDetected.h>

LineDetected::LineDetected()
{
    mX = 0;
    mY = 0;

    // Model_Base->BGRColorRange->ReValue = 0;
    // Model_Base->BGRColorRange->GrValue = 0;
    // Model_Base->BGRColorRange->BuValue = 0;
    hough_threshold = 100;
    hough_minLineLength = 60.0;
    hough_maxLineGap = 40.0;

}
LineDetected::~LineDetected()
{
    
}

Mat LineDetected::ImagePreprocessing(const Mat iframe)
{
    orign = iframe.clone();
    resize(orign, orign, cv::Size(320, 240));
    blur(orign,orign,Size(3,3));
    int frame_rows = iframe.rows;
    int frame_cols = iframe.cols;
    
    //濾除非場地部份(laplace)
    Mat mask = Mat::zeros(orign.rows,orign.cols, CV_8UC3); 
    Mat Kernel = (Mat_<float>(3, 3) << 0, -1, 0, 0, 6, 0, 0, -1, 0);
    Mat imageEnhance;
    filter2D(orign, imageEnhance, CV_8UC3, Kernel);
    //imshow("imageEnhance",imageEnhance);
    imageGamma = Mat::zeros(imageEnhance.rows,imageEnhance.cols, CV_32FC3); 
    for(int row = 0; row < imageEnhance.rows; row++)
    {
        for(int col = 0; col < imageEnhance.cols; col++)
        {
            imageGamma.at<Vec3f>(row, col)[0] = (imageEnhance.at<Vec3b>(row, col)[0]) * (imageEnhance.at<Vec3b>(row, col)[0]) * (imageEnhance.at<Vec3b>(row, col)[0]);
            imageGamma.at<Vec3f>(row, col)[1] = (imageEnhance.at<Vec3b>(row, col)[1]) * (imageEnhance.at<Vec3b>(row, col)[1]) * (imageEnhance.at<Vec3b>(row, col)[1]);
            imageGamma.at<Vec3f>(row, col)[2] = (imageEnhance.at<Vec3b>(row, col)[2]) * (imageEnhance.at<Vec3b>(row, col)[2]) * (imageEnhance.at<Vec3b>(row, col)[2]);
        }
    }
    normalize(imageGamma, imageGamma, 0, 255, NORM_MINMAX);
    convertScaleAbs(imageGamma, imageGamma);
    
    //imshow("imageGamma",imageGamma);
    R_value = Model_Base->BGRColorRange->ReValue;
    G_value = Model_Base->BGRColorRange->GrValue;
    B_value = Model_Base->BGRColorRange->BuValue;
    
    for(int col = 0; col < imageGamma.cols;col++)
    {
        for(int row = imageGamma.rows -1 ; row > 0 ;row--)
        {
            int B = imageGamma.at<Vec3b>(row, col)[0];
            int G = imageGamma.at<Vec3b>(row, col)[1];
            int R = imageGamma.at<Vec3b>(row, col)[2];
            if( G >= G_value && B <= B_value)
            {
                mask.at<Vec3b>(row, col)[0] = 255;
                mask.at<Vec3b>(row, col)[1] = 255;
                mask.at<Vec3b>(row, col)[2] = 255;
            }
        }
    }
    
    Mat mask_element = getStructuringElement(MORPH_RECT, Size(7, 7)); 
    dilate(mask,mask,mask_element);
    //imshow("mask",mask);
    
    cvtColor(mask,mask,COLOR_BGR2GRAY);
    threshold(mask,mask,200,255,THRESH_BINARY);
    morphologyEx(mask, morph, CV_MOP_OPEN, mask_element); 
    
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours( morph, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

    Mat green_mask( orign.size(), CV_8U, Scalar(0));

    // Find the convex hull object for each contour
    vector<vector<Point> >hull(contours.size());
    vector<vector<Point> >greenhull;
    ROS_INFO("----------------contours = %d",contours.size());
    int j = 0;
    // Mat drawing;   
    Vec4i fieldline;
    for( int i = 0; i < contours.size(); i++ )
    {     
        convexHull( Mat(contours[i]), hull[i], false );
        double contoursArea = contourArea(contours[i]);
        if(contoursArea>4500)
        {
            greenhull.push_back(contours[i]);
            //fitLineRANSAC(greenhull,fieldline);
        }
    }
    ROS_INFO("----------------hull = %d",hull.size());  
    vector<vector<Point> >contours_poly(greenhull.size());
    //ROS_INFO("x1 = %d, y1 = %d, x2 = %d, y2 = %d",fieldline[0], fieldline[1],fieldline[2], fieldline[3]);
    //line( green_mask, Point(fieldline[0], fieldline[1]), Point(fieldline[2], fieldline[3]), Scalar(0,0,255), 2, CV_AA);
    for( int i = 0; i< greenhull.size(); i++ )
    {
        
        //approxPolyDP(Mat(greenhull[i]), contours_poly[i], 100, true);//待修改
        drawContours( green_mask, greenhull, i, Scalar(255), -1);
        /*for( int j = 0; j< greenhull[i].size(); j++ )
        {
            circle(green_mask,greenhull[i][j],2,Scalar(0,0,255),CV_FILLED,-1);
        }*/
    }
    imageGamma.copyTo(nobackgroud_image,green_mask);
    //imshow("nobackgroud_image",nobackgroud_image);

    for(int col = 0; col < nobackgroud_image.cols;col++)
    {
        for(int row = nobackgroud_image.rows -1 ; row > 0 ;row--)
        {
            int B = nobackgroud_image.at<Vec3b>(row, col)[0];
            int G = nobackgroud_image.at<Vec3b>(row, col)[1];
            int R = nobackgroud_image.at<Vec3b>(row, col)[2];
            if( B >= B_value && G >= G_value && R >= R_value)
            {
                nobackgroud_image.at<Vec3b>(row, col)[0] = 255;
                nobackgroud_image.at<Vec3b>(row, col)[1] = 255;
                nobackgroud_image.at<Vec3b>(row, col)[2] = 255;
            }else if( B <= 10 && G <= 10 && R<=10)
            {
                nobackgroud_image.at<Vec3b>(row, col)[0] = 0;
                nobackgroud_image.at<Vec3b>(row, col)[1] = 0;
                nobackgroud_image.at<Vec3b>(row, col)[2] = 0;
            }else{
                nobackgroud_image.at<Vec3b>(row, col)[0] = 0;
                nobackgroud_image.at<Vec3b>(row, col)[1] = 255;
                nobackgroud_image.at<Vec3b>(row, col)[2] = 0;
            }
        }
    }
    Mat greenmask_element = getStructuringElement(MORPH_RECT, Size(8, 8)); 
    dilate(nobackgroud_image,nobackgroud_image,greenmask_element);
    //imshow("nobackgroud_image1",nobackgroud_image);
    return nobackgroud_image;
    
    // orign = iframe.clone();
    // blur(orign,orign,Size(10,10));
    // resize(orign, orign, cv::Size(320, 240));
    // //濾除非場地部份
    // Mat mask = Mat::zeros(orign.rows,orign.cols, CV_8UC3); 
    // Mat Kernel = (Mat_<float>(3, 3) << 0, -1, 0, -1, 6, -1, 0, -1, 0);
    // Mat imageEnhance;
    // filter2D(orign, imageEnhance, CV_8UC3, Kernel);

    // imageGamma = Mat::zeros(imageEnhance.rows,imageEnhance.cols, CV_32FC3); 
    // for(int row = 0; row < imageEnhance.rows; row++)
    // {
    //     for(int col = 0; col < imageEnhance.cols; col++)
    //     {
    //         imageGamma.at<Vec3f>(row, col)[0] = (imageEnhance.at<Vec3b>(row, col)[0]) * (imageEnhance.at<Vec3b>(row, col)[0]) * (imageEnhance.at<Vec3b>(row, col)[0]);
    //         imageGamma.at<Vec3f>(row, col)[1] = (imageEnhance.at<Vec3b>(row, col)[1]) * (imageEnhance.at<Vec3b>(row, col)[1]) * (imageEnhance.at<Vec3b>(row, col)[1]);
    //         imageGamma.at<Vec3f>(row, col)[2] = (imageEnhance.at<Vec3b>(row, col)[2]) * (imageEnhance.at<Vec3b>(row, col)[2]) * (imageEnhance.at<Vec3b>(row, col)[2]);
    //     }
    // }
    // normalize(imageGamma, imageGamma, 0, 255, NORM_MINMAX);
    // convertScaleAbs(imageGamma, imageGamma);
    // //namedWindow("imageGamma",WINDOW_NORMAL);
    // //imshow("imageGamma",imageGamma);
    
    // R_value = Model_Base->BGRColorRange->ReValue;
    // G_value = Model_Base->BGRColorRange->GrValue;
    // B_value = Model_Base->BGRColorRange->BuValue;


    // if(Vertical_Head_Angle > 50.0)
    // {
    //     for(int col = 0; col < imageGamma.cols; col++)
    //     {
    //         for(int row = imageGamma.rows - 1; row > 0; row--)
    //         {
    //             int b = imageGamma.at<Vec3b>(row, col)[0];
    //             int g = imageGamma.at<Vec3b>(row, col)[1];
    //             int r = imageGamma.at<Vec3b>(row, col)[2];
    //             if( g >= G_value) //0:Y452X61
    //             {
    //                 /*if((b > 230 && g > 230 && r < 150) || (b == 255 && g == 255 && r == 255))
    //                 {
    //                     int b2 = imageGamma.at<Vec3b>(row - 3, col)[0];
    //                     int g2 = imageGamma.at<Vec3b>(row - 3, col)[1];
    //                     int r2 = imageGamma.at<Vec3b>(row - 3, col)[2];
    //                     if(b2 > 230 && g2 > 230 && r2 < 150 || (b2 == 255 && g2 == 255 && r2 == 255))
    //                     {
    //                         int b3 = imageGamma.at<Vec3b>(row - 9, col)[0];
    //                         int g3 = imageGamma.at<Vec3b>(row - 9, col)[1];
    //                         int r3 = imageGamma.at<Vec3b>(row - 9, col)[2];
    //                         if(b3 > 230 && g3 > 230 && r3 < 150 || (b3 == 255 && g3 == 255 && r3 == 255))
    //                         {
    //                             break;
    //                         }
    //                     }
    //                 }*/
    //                 mask.at<Vec3b>(row, col)[0] = 255;
    //                 mask.at<Vec3b>(row, col)[1] = 255;
    //                 mask.at<Vec3b>(row, col)[2] = 255;
    //             }
    //             else
    //             {
    //                 break;
    //             } 
    //         }
    //     }

    //     //Mat mask_element = getStructuringElement(MORPH_RECT, Size(3, 3)); 
    //     //morphologyEx(mask, mask, CV_MOP_OPEN, mask_element); 
    //     imageGamma.copyTo(nobackgroud_image,mask);
    // }
    // else
    // {
    //     nobackgroud_image = imageGamma.clone();
    // }
    
    
    // // namedWindow("nobackgroud_image",WINDOW_NORMAL);
    // // imshow("nobackgroud_image",nobackgroud_image);
  
    // for(int row = 0; row < nobackgroud_image.rows;row++)
    // {
    //     for(int col = 0; col < nobackgroud_image.cols;col++)
    //     {
    //         int b = nobackgroud_image.at<Vec3b>(row, col)[0];
    //         int g = nobackgroud_image.at<Vec3b>(row, col)[1];
    //         int r = nobackgroud_image.at<Vec3b>(row, col)[2];
    //         if(b >= B_value && g >= G_value && r <= R_value)
    //         {
    //             nobackgroud_image.at<Vec3b>(row, col)[0] = 255;
    //             nobackgroud_image.at<Vec3b>(row, col)[1] = 255;
    //             nobackgroud_image.at<Vec3b>(row, col)[2] = 255;
    //         }
    //         else
    //         {
    //             nobackgroud_image.at<Vec3b>(row, col)[0] = 0;
    //             nobackgroud_image.at<Vec3b>(row, col)[1] = 0;
    //             nobackgroud_image.at<Vec3b>(row, col)[2] = 0;
    //         }
    //     }
    // }
    // morph = nobackgroud_image.clone();
    // Mat element = getStructuringElement(MORPH_RECT, Size(15, 15));
    // Mat element_2 = getStructuringElement(MORPH_RECT, Size(5, 5));
    // dilate(morph,element,element);
    // erode(morph,element,element_2);

    // element = getStructuringElement(MORPH_RECT, Size(2,2));
    // element_2 = getStructuringElement(MORPH_RECT, Size(15,15));
    // morphologyEx(morph, morph, CV_MOP_CLOSE, element_2);
    // morphologyEx(morph, morph, CV_MOP_OPEN, element);
    
    // Canny(morph, edge, 50, 150, 3);
    // return  edge;
}

Mat LineDetected::ImageCanny(const Mat iframe)
{     
    edge = iframe.clone() ;
    cvtColor(edge,edge,COLOR_BGR2GRAY);
    threshold(edge,edge,165,255,THRESH_BINARY);
    Canny(edge, edge, 50, 150, 3);
    //imshow("GreenField1",GreenField);
    // imshow("edge",edge);
    return  edge ;
}

double LineDetected::dis2(coordinate a, coordinate b)                //點a、b距離的平方
{
	return (a.X - b.X)*(a.X - b.X) + (a.Y-b.Y)*(a.Y-b.Y);
}

int LineDetected::dir(coordinate A, coordinate B, coordinate P)      //點P與線段AB位置關係
{       
    coordinate AB = { B.X - A.X, B.Y - A.Y };
	coordinate AP = { P.X - A.X, P.Y - A.Y };
    double cross = AB.X*AP.Y - AB.Y*AP.X;
    double   dot = AB.X*AP.X + AB.Y*AP.Y;
   
    if (cross < 0) return -1;       //逆時針
	else if (cross > 0) return 1;   //順時針
	else if (dot < 0) return -2;    //反延長線
	else if (dot >= 0 && dis2(A,B) >= dis2(A,P))
	{
		if (dis2(A, B) < dis2(A, P)) return 2;  //延長線
		return 0;                          //在線上
	}                                      
}

double LineDetected::disMin(coordinate A, coordinate B, coordinate P)//點P到線段AB的最短距離
{
	double r = ((P.X-A.X)*(B.X-A.X) + (P.Y-A.Y)*(B.Y-A.Y)) / dis2(A, B);
	if (r <= 0) return sqrt(dis2(A, P));
	else if (r >= 1) return sqrt(dis2(B, P));
	else
	{
		double AC = r*sqrt(dis2(A,B));
		return sqrt(dis2(A,P)-AC*AC);
	}
}

double LineDetected::MinDistance(Vec4i X,Vec4i Y)
{
    coordinate Xstart = {X[0],X[1]};
    coordinate Xend = {X[2],X[3]};
    coordinate Ystart = {Y[0],Y[1]};
    coordinate Yend = {Y[2],Y[3]};

    coordinate Xline = {Xend.X - Xstart.X , Xend.Y-Xstart.Y};
    coordinate Yline = {Yend.X - Ystart.X , Yend.Y-Ystart.Y};

    if (dir(Xstart, Xend, Ystart) * dir(Xstart, Xend, Yend) <= 0 && 
        dir(Ystart, Yend, Xstart) * dir(Ystart, Yend, Xend) <= 0)  //兩線段相交, 距離為0
	{
        return 0;
    }else{                                                   //如不相交, 則最短距離為每個端點到另一條線段距離的最小值
        double XYMinDistance = min(min(min(disMin(Xstart, Xend, Ystart), disMin(Xstart, Xend, Yend)), disMin(Ystart, Yend, Xstart)),disMin(Ystart,Yend,Xend));
        return XYMinDistance;
    }
}

double LineDetected::AngleDiff(Vec4i X,Vec4i Y)
{
    coordinate Xstart = {X[0],X[1]};
    coordinate Xend = {X[2],X[3]};
    coordinate Ystart = {Y[0],Y[1]};
    coordinate Yend = {Y[2],Y[3]};

    if((Xend.X-Xstart.X) == 0 ) mX = 90;
    else mX = atan2((Xend.Y-Xstart.Y),(Xend.X-Xstart.X))*180/CV_PI;
    if((Yend.X-Ystart.X) == 0)  mY = 90;
    else mY = atan2((Yend.Y-Ystart.Y),(Yend.X-Ystart.X))*180/CV_PI;
    
    if( mX >= 0 && mY >= 0 || mX <= 0 && mY <= 0 ) return abs( mX - mY );
    else if( mX > 0 && mY < 0 ) return mX + abs(mY);
    else if( mX < 0 && mY > 0 ) return abs(mX) + mY;
}
    
coordinate LineDetected::Midpoint(Vec4i line)
{
    coordinate midpoint ;
    midpoint.X = (line[2]+line[0])/2;
    midpoint.Y = (line[3]+line[1])/2;
    return midpoint;
}

double LineDetected::Slope(Vec4i line)
{
    if((line[2]-line[0]) == 0) return 90;
    else {
        double s = atan2((double(line[3])-double(line[1])),(double(line[2])-double(line[0])))*(180.0/CV_PI);
        if(s > 90) return -180+s;
        else if(s < -90) return 180+s;
        else return s;
    }
}

void LineDetected::Merge(Vec4i X,Vec4i Y)
{
    coordinate Xm = Midpoint(X);
    coordinate Ym = Midpoint(Y);
    coordinate Xstart = {X[0],X[1]};
    coordinate Xend = {X[2],X[3]};
    coordinate Ystart = {Y[0],Y[1]};
    coordinate Yend = {Y[2],Y[3]};
    
    XDistance = sqrt(dis2(Xstart,Xend));
    YDistance = sqrt(dis2(Ystart,Yend));
    
    if(XDistance >= YDistance) theta = Slope(X);
    else theta = Slope(Y);

    NewLine = {0,0,0,0};
    if(theta == 0)
    {
        thetaZeroY = (Xstart.Y + Ystart.Y)/2;
        int thetaZeroX[4] =  { Xstart.X,Xend.X,Ystart.X,Yend.X};
        sort(thetaZeroX, thetaZeroX + 4);
        NewLine = {thetaZeroX[0],thetaZeroY,thetaZeroX[3],thetaZeroY};
    }else
    {
        double r = XDistance/(XDistance + YDistance);
        coordinate P = { int((r * Xm.X)+((1-r)*Ym.X) ), int((r * Xm.Y) + ((1 - r) * Ym.Y))};
        double Za = tan ( theta * CV_PI / 180.0);
        double Zb = double(P.Y) - (Za * double(P.X)); 

        double m = -(1/Za);
        double SbXstart = double(Xstart.Y)-(m*double(Xstart.X));
        double SbXend = double(Xend.Y)-(m*double(Xend.X));
        double SbYstart = double(Ystart.Y)-(m*double(Ystart.X));
        double SbYend = double(Yend.Y)-(m*double(Yend.X));

        //printf("Za = %f Zb =%f m = %f SbXstart = %f SbXend = %f SbYstart = %f SbYend = %f\n",Za,Zb,m,SbXstart,SbXend,SbYstart,SbYend);
        int x1 = -(Zb-SbXstart)/(Za-m);
        int y1 = ((Za+m)*x1+(Zb+SbXstart))/2;
        coordinate projXs={x1,y1};
        //printf("x1 = %d y1 =%d\n",x1,y1);
        int x2 = -(Zb-SbXend)/(Za-m);
        int y2 = ((Za+m)*x2+(Zb+SbXend))/2;
        coordinate projXe={x2,y2};
        //printf("x2 = %d y2 =%d\n",x2,y2);
        int x3 = -(Zb-SbYstart)/(Za-m);
        int y3 = ((Za+m)*x3+(Zb+SbYstart))/2;
        coordinate projYs={x3,y3};
        //printf("x3 = %d y3 =%d\n",x3,y3);
        int x4 = -(Zb-SbYend)/(Za-m);
        int y4 = ((Za+m)*x4+(Zb+SbYend))/2;
        coordinate projYe={x4,y4};
        //printf("x4 = %d y4 =%d\n",x4,y4);

        double projXsToprojXe = sqrt(dis2(projXs,projXe));
        double projXsToprojYs = sqrt(dis2(projXs,projYs));
        double projXsToprojYe = sqrt(dis2(projXs,projYe));
        double projXeToprojYs = sqrt(dis2(projXe,projYs));
        double projXeToprojYe = sqrt(dis2(projXe,projYe));
        double projYsToprojYe = sqrt(dis2(projYs,projYe));

        double distanceset[6] =  { projXsToprojXe,projXsToprojYs,projXsToprojYe,projXeToprojYs,projXeToprojYe,projYsToprojYe};
        sort(distanceset, distanceset + 6);
        
        if(distanceset[5] == projXsToprojXe ) 
        {
            NewLine = {x1,y1,x2,y2};
        }
        if(distanceset[5] == projXsToprojYs )
        {
            NewLine = {x1,y1,x3,y3};
        }
        if(distanceset[5] == projXsToprojYe )
        {
            NewLine = {x1,y1,x4,y4};
        }
        if(distanceset[5] == projXeToprojYs )
        {
            NewLine = {x2,y2,x3,y3};
        }
        if(distanceset[5] == projXeToprojYe )
        {
            NewLine = {x2,y2,x4,y4};
        }
        if(distanceset[5] == projYsToprojYe )
        {
            NewLine = {x3,y3,x4,y4};
        }
    }
    printf("NewLine[0]=%d NewLine[1]=%d NewLine[2]=%d NewLine[3]=%d\n",NewLine[0],NewLine[1],NewLine[2],NewLine[3]);
}

int LineDetected::checkline(const Mat image_Enhance,const Mat canny,Vec4i line)
{ 
    coordinate Xstart ;
    coordinate Xend ;
    
    if( line[0] > line[2] )
    {
        Xstart = {line[2],line[3]};
        Xend = {line[0],line[1]};
    }else{
        Xstart = {line[0],line[1]};
        Xend = {line[2],line[3]};
    }

    int Vgreen = 0;
    int Vwhite = 0;
    int Vedge = 0;
    double m = Slope(line)* CV_PI / 180.0;
    double n = Slope(line);
    //ROS_INFO("---n = %f ,m = %f --\n",n,m);
    double length = sqrt(dis2(Xstart,Xend));
    int length_unitX = 0;
    int length_unitY = 0;
    for(size_t i = 0 ;i < 8;i++)
    {
        if(n != 90)
        {   
            if( (n > 90 && n< 270)|| (n <-90 && n>-270))
            {
                length_unitX = int((length/7)*i)*cos(CV_PI - abs(m));
                length_unitY = int((length/7)*i)*sin(CV_PI - abs(m));
            }else{
                length_unitX = int(((length/7)*i)*cos(m));
                length_unitY = int(((length/7)*i)*sin(m));
            }
        }else{
            if(line[1]>line[3])
            {
                length_unitX = 0;
                length_unitY = -(length/7)*i;
            }else{
                length_unitX = 0;
                length_unitY = (length/7)*i;
            }        
        }
        double diff = 0;
        for(int row = Xstart.Y + length_unitY ; row > 0; row--)
        {
            int b = image_Enhance.at<Vec3b>(row, Xstart.X + length_unitX)[0];
            int g = image_Enhance.at<Vec3b>(row, Xstart.X + length_unitX)[1];
            int r = image_Enhance.at<Vec3b>(row, Xstart.X + length_unitX)[2];
                        
            if(g >= 235 && b <=50 && r <= 50)
            {
                int checkgreenX = Xstart.X + length_unitX;
                int checkgreenY = row;
                Distance checkgreen = measure(checkgreenX,checkgreenY,CameraType::Monocular);
                Distance checkpoint = measure(checkgreenX,Xstart.Y + length_unitY,CameraType::Monocular);
                diff = sqrt(pow((checkgreen.x_dis-checkpoint.x_dis),2)+pow((checkgreen.y_dis-checkpoint.y_dis),2));
            }else{
                break;
            }          
        }
        if( diff <= 10 )
        {
            Vgreen++;
        }
        int b1 = image_Enhance.at<Vec3b>(Xstart.Y + length_unitY, Xstart.X + length_unitX)[0];
        int g1 = image_Enhance.at<Vec3b>(Xstart.Y + length_unitY, Xstart.X + length_unitX)[1];
        int r1 = image_Enhance.at<Vec3b>(Xstart.Y + length_unitY, Xstart.X + length_unitX)[2];
        if(g1 >= 235 && b1 >=235 && r1 >= 235)
        {
            Vwhite++;
        }
    }
    ROS_INFO("Vwhite = %d , Vgreen  = %d",Vwhite , Vgreen );
    if(Vwhite > 6 && Vgreen > 6 ) return 1;
    else return 0;
}

vector<Vec4i> LineDetected::complement(vector<Vec4i> all_line,Vec4i remove) 
{    
    //ROS_INFO("all_line = %d",all_line.size());
    for(size_t i = 0 ; i < all_line.size(); i++)
    {
        Vec4i s = all_line[i];
        if(s[0] == remove[0] && s[1] == remove[1] && s[2] == remove[2] &&s[3] == remove[3]) all_line.erase(all_line.begin() + i);       
    }
    //ROS_INFO("After reduce = %d",all_lines.size());
    return all_line;
}

Mat LineDetected::Merge_similar_line(const Mat iframe,const Mat canny_iframe,const Mat original_frame)
{
    tmp.clear();
    check_lines.clear();
    all_lines.clear();
    reduce_similar_lines.clear();
    merge_similar_lines.clear();

    //ROS_INFO("hough_threshold = %d hough_minLineLength = %f hough_maxLineGap = %f",hough_threshold,hough_minLineLength,hough_maxLineGap);
    HoughLinesP(canny_iframe,all_lines,1,CV_PI/180,hough_threshold,hough_minLineLength,hough_maxLineGap); 
    
    Mat original_frame0=iframe.clone();
    Mat original_frame1=original_frame.clone();
    Mat original_frame2=original_frame.clone();

    int all_lines_Size = all_lines.size();
    merge_similar_lines = all_lines;
    
    //ROS_INFO("all_lines_Size = %d",all_lines_Size);
    ROS_INFO("/---------------start-------------/");
    for(size_t i = 0 ; i < all_lines.size(); i++)
    {
        Vec4i X = all_lines[i];
        //ROS_INFO("all_line(%d)=(x1 = %d ,y1 =%d, x2 =%d ,y2 =%d)  slope = %f",i,X[0],X[1],X[2],X[3],Slope(X));
        merge_similar_lines = complement(merge_similar_lines,X);
        int merge_similar_lines_SIZE = merge_similar_lines.size();
        int check = 0;
        for(size_t j = 0 ; j < merge_similar_lines.size(); j++)
        {
            Vec4i Y = merge_similar_lines[j];
            //ROS_INFO("merge_similar_lines(%d)=(x1 = %d ,y1 =%d, x2 =%d ,y2 =%d)  slope = %f",j,Y[0],Y[1],Y[2],Y[3],Slope(Y));
            //ROS_INFO("MinDistance = %f",MinDistance(X,Y));
            //ROS_INFO("AngleDiff = %f",AngleDiff(X,Y));
            if(MinDistance(X,Y) < 20 && AngleDiff(X,Y) < 5)
            {
                //ROS_INFO("--------Merge--------");
                Merge(X,Y);
                int checklinenum = checkline(original_frame0,canny_iframe,NewLine);
                //ROS_INFO("checkline = %d \n",checklinenum);
                if(checklinenum == 1)
                {
                    check = 1; 
                    if(AngleDiff(X,NewLine) < 1 || AngleDiff(Y,NewLine) < 1)
                    {
                        reduce_similar_lines.push_back(NewLine);
                        merge_similar_lines = complement(merge_similar_lines,Y);
                        all_lines = complement(all_lines,Y);
                        all_lines_Size -= 2;
                        //ROS_INFO("all_lines_Size (After Merge ) '%d' = %d",j,all_lines_Size);
                        j--;
                    }else{
                        break;
                        //ROS_INFO("Merge ERROR");
                    }
                }else{
                    check = 0; 
                    ROS_INFO("Not Line");
                } 
                                
            }else{
                //ROS_INFO("all_lines_Size (NO Merge ) '%d' = %d",j,all_lines_Size);
            }
        }       
        int Maxdis =0;
        //ROS_INFO("reduce_similar_lines = %d",reduce_similar_lines.size());
        if(merge_similar_lines.size()>0)
        {
            if(reduce_similar_lines.size()>1)
            {
                ROS_INFO("--------------------------");
                tmp = reduce_similar_lines;
                for(size_t k=0 ; k < reduce_similar_lines.size() ; k++)
                {
                    Vec4i doublecheck1 = reduce_similar_lines[k];
                    tmp = complement(tmp,doublecheck1);
                    for(size_t l=0 ; l < tmp.size() ; l++)
                    {
                        Vec4i doublecheck2 = tmp[l];
                        Merge(doublecheck1,doublecheck2); 
                        coordinate NewLinestart = {NewLine[0],NewLine[1]};
                        coordinate NewLineend = {NewLine[2],NewLine[3]};
                        int newlineDistance = sqrt(dis2(NewLinestart,NewLineend));
                        if( newlineDistance >=  Maxdis)
                        {
                            MaxLine = NewLine;
                        }                 
                    }
                }
                //ROS_INFO("MaxLine = (x1= %d ,y1 = %d ,x2 = %d ,y2 = %d)",MaxLine[0],MaxLine[1],MaxLine[2],MaxLine[3]);
                check_lines.push_back(MaxLine);
            }
            if(reduce_similar_lines.size() == 1)
            {
                //ROS_INFO("push_back(reduce_similar_lines[0])");
                check_lines.push_back(reduce_similar_lines[0]);
            }
            if(merge_similar_lines.size() == merge_similar_lines_SIZE && check != 0){
                //ROS_INFO("push_back(X)");
                check_lines.push_back(X);
            }
            reduce_similar_lines.clear();
        }else if(reduce_similar_lines.size() == 1)
        {
            //ROS_INFO("push_back(reduce_similar_lines[0])");
            check_lines.push_back(reduce_similar_lines[0]);
        }else if(check != 0){
            //ROS_INFO("push_back(X)");
            check_lines.push_back(X);
        }
        reduce_similar_lines.clear();    
        all_lines_Size ++;

    }
    ROS_INFO("check_lines.size(1) = %d",check_lines.size());

    ROS_INFO("/---------------finish-------------/");
    for(size_t i = 0 ; i < all_lines.size(); i++)
    {
        Vec4i X = all_lines[i];
        line( original_frame1, Point(X[0], X[1]), Point(X[2], X[3]), Scalar(255,0,0), 2, CV_AA);
    }
    for( size_t i = 0; i < check_lines.size(); i++ )
	{
        Vec4i l = check_lines[i]; 
	    LineINF lineinf;
        coordinate midpoint = Midpoint(l);
        Distance start_ = measure(l[0],l[1],CameraType::Monocular);
        Distance end_ = measure(l[2],l[3],CameraType::Monocular);
        double dis =  (start_.x_dis - end_.x_dis)*(start_.x_dis - end_.x_dis) + (start_.y_dis - end_.y_dis)*(start_.y_dis - end_.y_dis);
        lineinf.middlepoint = measure(midpoint.X,midpoint.Y,CameraType::Monocular);
        lineinf.Line_length = sqrt(dis);
        coordinate FOV_Bottom_POS = {image_bottom_width_length/2,0};
        double   AdotB = FOV_Bottom_POS.X*lineinf.middlepoint.x_dis + FOV_Bottom_POS.Y*lineinf.middlepoint.y_dis;
        lineinf.Line_theta = acos((AdotB)/((image_bottom_width_length/2)*(lineinf.middlepoint.dis)))*180/CV_PI;
        /*ROS_INFO("check_lines(%d)=(x1 = %d ,y1 =%d, x2 =%d ,y2 =%d)  slope = %f",i,l[0],l[1],l[2],l[3],Slope(l));
        ROS_INFO("lineinf.middlepoint (%d) = (x_dis = %d ,y_dis = %d ,dis = %d )",i,lineinf.middlepoint.x_dis,lineinf.middlepoint.y_dis,lineinf.middlepoint.dis);
        ROS_INFO("lineinf.Line_length (%d) =  %f ",i,lineinf.Line_length);
        ROS_INFO("lineinf.Line_theta (%d) =  %f ",i,lineinf.Line_theta);*/
        line( original_frame2, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 2, CV_AA);
        circle(original_frame2, Point(midpoint.X, midpoint.Y), 1, Scalar(0, 255, 0), -1);
    }
    //imshow("original_frame1",original_frame1);
    //imshow("original_frame2",original_frame2);
    
    return original_frame2;
}

// int LineDetected::getHistograph(const Mat grayImage)
// {
//     int channels[]={0};
// 	const int histSize[]={256};
// 	float range[]={0,256};
// 	const float* ranges[]={range};
// 	calcHist(&grayImage,1,channels,Mat(),hist,1,histSize,ranges,true,false);
// 	double maxValue=0;
// 	minMaxLoc(hist,0,&maxValue,0,0);
// 	int rows=cvRound(maxValue);
// 	for(int i=0;i<256;i++)
// 	{
// 		int temp=(int)(hist.at<float>(i,0));
//         if (i>=10&&temp_max<=temp)
//         {
//             temp_max=temp;
//             i_min = i;
//         }
//         if (temp > 0)
//         {
//             temp_min=temp;
//             i_max = i;
//         }
// 	}
//     return (i_max-i_min)*0.5+i_min;	
// }
