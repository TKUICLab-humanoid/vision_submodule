#include <LineDetected/LineDetected.h>

LineDetected::LineDetected()
{
    mX = 0;
    mY = 0;

    Model_Base->BGRColorRange->ReValue = 150;
    Model_Base->BGRColorRange->GrValue = 220;
    Model_Base->BGRColorRange->BuValue = 220;
    hough_threshold = 100;
    hough_minLineLength = 60.0;
    hough_maxLineGap = 40.0;

}
LineDetected::~LineDetected()
{
    
}

double timeStart, timeEnd;
timespec time1, time2;

timespec LineDetected::diff(timespec start, timespec end) 
{
  timespec temp;
  if ((end.tv_nsec - start.tv_nsec) < 0) {
    temp.tv_sec = end.tv_sec - start.tv_sec - 1;
    temp.tv_nsec = 1000000000 + end.tv_nsec - start.tv_nsec;
  } else {
    temp.tv_sec = end.tv_sec - start.tv_sec;
    temp.tv_nsec = end.tv_nsec - start.tv_nsec;
  }
  return temp;
}
Mat LineDetected::ImagePreprocessing(const Mat iframe)
{
    // clock_gettime(CLOCK_PROCESS_CPUTIME_ID, & time1);
    orign = iframe.clone();
    // bilateralFilter(iframe, orign, 10, 10, 10);
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
            if( G >= G_value && B >= B_value && R <= R_value)
            {
                mask.at<Vec3b>(row, col)[0] = 255;
                mask.at<Vec3b>(row, col)[1] = 255;
                mask.at<Vec3b>(row, col)[2] = 255;
            }
        }
    }
    
    Mat mask_element = getStructuringElement(MORPH_RECT, Size(8, 8)); 
    dilate(mask,mask,mask_element);
    
    cvtColor(mask,mask,COLOR_BGR2GRAY);
    threshold(mask,mask,200,255,THRESH_BINARY);

    morphologyEx(mask, morph, CV_MOP_OPEN, mask_element); 

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours( morph, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

    Mat green_mask( orign.size(), CV_8U, Scalar(0));
    Mat orign1 = iframe.clone();
    Mat orign2 = iframe.clone();
    Gmask = Mat::zeros(orign.rows,orign.cols, CV_8U); 
    // Find the convex hull object for each contour
    vector<vector<Point> >hull(contours.size());
    vector<vector<Point> >greenhull;
    // ROS_INFO("----------------contours = %d",contours.size());
    int j = 0;
    for( int i = 0; i < contours.size(); i++ )
    {     
        convexHull( Mat(contours[i]), hull[i], false );
        double contoursArea = contourArea(contours[i]);
        if(contoursArea > 30000)
        {
            greenhull.push_back(contours[i]);
        }
    }
    if(greenhull.size() < 1)
    {
        Gmask = fitLineRANSAC(orign1,green_mask,hull);
    }else{
        Gmask = fitLineRANSAC(orign1,green_mask,greenhull);
    }
    // ROS_INFO("----------------hull = %d",hull.size());  
    // ROS_INFO("----------------greenhull = %d",greenhull.size()); 
    // vector<vector<Point> >contours_poly(greenhull.size());
    //ROS_INFO("x1 = %d, y1 = %d, x2 = %d, y2 = %d",fieldline[0], fieldline[1],fieldline[2], fieldline[3]);
    // line( green_mask, Point(fieldline[0], fieldline[1]), Point(fieldline[2], fieldline[3]), Scalar(0,0,255), 2, CV_AA);
    for( int i = 0; i< hull.size(); i++ )
    {
        //approxPolyDP(Mat(greenhull[i]), contours_poly[i], 100, true);//待修改
        drawContours( orign2, hull, i, Scalar(255), -1);
        for( int j = 0; j< hull[i].size(); j++ )
        {
            circle(orign2,hull[i][j],2,Scalar(0,0,255),CV_FILLED,-1);
        }
    }
    orign.copyTo(nobackgroud_image,Gmask);
    // imshow("drawing",drawing);

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
            }else if( B <= 20 && G <= 20 && R <= 20)
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
    Mat greenmask_element = getStructuringElement(MORPH_RECT, Size(4, 4)); 
    dilate(nobackgroud_image,nobackgroud_image,greenmask_element);
    //imshow("nobackgroud_image1",nobackgroud_image);
    // resize(nobackgroud_image, nobackgroud_image, cv::Size(320, 240));
    // clock_gettime(CLOCK_PROCESS_CPUTIME_ID, & time2);
    // cout << "imageprocessing total time (clock_gettime) = " << diff(time1, time2).tv_sec << ":" << diff(time1, time2).tv_nsec << endl;

    return nobackgroud_image;
}

Mat LineDetected::ImageCanny(const Mat iframe)
{     
    edge = iframe.clone() ;
    cvtColor(edge,edge,COLOR_BGR2GRAY);
    threshold(edge,edge,165,255,THRESH_BINARY);
    Canny(edge, edge, 100, 150, 3);
    //imshow("GreenField1",GreenField);
    // imshow("edge",edge);
    return  edge ;
}

Pixel3Dpoint LineDetected::deproject_pixel2point(Coordinate point,float depth)
{
    // clock_gettime(CLOCK_PROCESS_CPUTIME_ID, & time1);
    // ROS_INFO("deproject_pixel2point");
    Pixel3Dpoint pixel3Dpoint = {0,0,0};
    Intrinsicscolor color_Intrinsics={312.2850,249.5522,615.0078,615.1781};
    pixel3Dpoint.x = depth * (point.X - color_Intrinsics.PPX)/color_Intrinsics.Fx;
    pixel3Dpoint.y = depth * (point.Y - color_Intrinsics.PPY)/color_Intrinsics.Fy;
    pixel3Dpoint.z = depth;
    // clock_gettime(CLOCK_PROCESS_CPUTIME_ID, & time2);
    // cout << "deproject_pixel2point total time (clock_gettime) = " << diff(time1, time2).tv_sec << ":" << diff(time1, time2).tv_nsec << endl;
    return pixel3Dpoint;
}
double LineDetected::calculate_3D(Coordinate a, Coordinate b)
{
    // clock_gettime(CLOCK_PROCESS_CPUTIME_ID, & time1);
    // ROS_INFO("calculate_3D");
    float pointA = 0;
    float pointB = 0;
    if(!depth_buffer.empty())
    {
        pointA = (depth_buffer.at<uint16_t>(a.X, a.Y))*0.1 ;
        pointB = (depth_buffer.at<uint16_t>(b.X, b.Y))*0.1 ;
    }
    
    Pixel3Dpoint A = {0,0,0};
    Pixel3Dpoint B = {0,0,0};
    if((pointA && pointB) != 0.0 && std::isfinite(pointA)&&std::isfinite(pointB))
    {
        // ROS_INFO("!= 0");
        A = deproject_pixel2point(a,pointA);
        B = deproject_pixel2point(b,pointB);
    }else{
        // ROS_INFO("== 0");
        A = {float(a.X),float(a.Y),0.0};
        B = {float(b.X),float(b.Y),0.0};
    }
    // ROS_INFO("A.x = %f,A.y = %f,A.z = %f",A.x,A.y,A.z);
    // ROS_INFO("B.x = %f,B.y = %f,B.z = %f",B.x,B.y,B.z);
    double dist = sqrt(pow((A.x-B.x),2)+pow((A.y-B.y),2)+pow((A.z-B.z),2));
    // clock_gettime(CLOCK_PROCESS_CPUTIME_ID, & time2);
    // cout << "calculate_3D total time (clock_gettime) = " << diff(time1, time2).tv_sec << ":" << diff(time1, time2).tv_nsec << endl;
    return dist;
}

double LineDetected::dis2(Coordinate a, Coordinate b)                //點a、b距離的平方
{
	return (a.X - b.X)*(a.X - b.X) + (a.Y-b.Y)*(a.Y-b.Y);
}

int LineDetected::dir(Coordinate A, Coordinate B, Coordinate P)      //點P與線段AB位置關係
{       
    Coordinate AB = { B.X - A.X, B.Y - A.Y };
	Coordinate AP = { P.X - A.X, P.Y - A.Y };
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

double LineDetected::disMin(Coordinate A, Coordinate B, Coordinate P,CameraType cameratype) //點P到線段AB的最短距離
{
    // ROS_INFO("disMin");
    // clock_gettime(CLOCK_PROCESS_CPUTIME_ID, & time1);
    double r = ((P.X-A.X)*(B.X-A.X) + (P.Y-A.Y)*(B.Y-A.Y)) / dis2(A, B);
	double dist = 0;
    double dist1 = 0;
    if(cameratype == CameraType::stereo)
    {
        if (r <= 0) 
        {
            dist = calculate_3D(A, P);
        }
        else if (r >= 1) {
            dist = calculate_3D(B, P);
        }
        else
        {
            double AC = r*calculate_3D(A, B);
            dist1 = calculate_3D(A, P);
            dist = sqrt(pow(dist1,2)-AC*AC);
        }
        if(!std::isfinite(dist))
        {
            if (r <= 0) 
            {
                dist = sqrt(dis2(A, P));
            }
            else if (r >= 1) {
                dist = sqrt(dis2(B, P));
            } 
            else
            {
                double AC = r*sqrt(dis2(A,B));
                dist = sqrt(dis2(A,P)-AC*AC);
            }
        }
    }else{
        if (r <= 0) 
        {
            dist = sqrt(dis2(A, P));
        }
        else if (r >= 1) {
            dist = sqrt(dis2(B, P));
        } 
        else
        {
            double AC = r*sqrt(dis2(A,B));
            dist = sqrt(dis2(A,P)-AC*AC);
        }
    }
    
    // clock_gettime(CLOCK_PROCESS_CPUTIME_ID, & time2);
    // cout << "disMin total time (clock_gettime) = " << diff(time1, time2).tv_sec << ":" << diff(time1, time2).tv_nsec << endl;
    return dist;
}

double LineDetected::MinDistance(Vec4i X,Vec4i Y)
{
    // ROS_INFO("MinDistance");
    Coordinate Xstart = {X[0],X[1]};
    Coordinate Xend = {X[2],X[3]};
    Coordinate Ystart = {Y[0],Y[1]};
    Coordinate Yend = {Y[2],Y[3]};

    if (dir(Xstart, Xend, Ystart) * dir(Xstart, Xend, Yend) <= 0 && 
        dir(Ystart, Yend, Xstart) * dir(Ystart, Yend, Xend) <= 0)  //兩線段相交, 距離為0
	{
        return 0;
    }else{                                                   //如不相交, 則最短距離為每個端點到另一條線段距離的最小值
        double XYMinDistance = min(min(min(disMin(Xstart, Xend, Ystart,CameraType::stereo), disMin(Xstart, Xend, Yend,CameraType::stereo)), disMin(Ystart, Yend, Xstart,CameraType::stereo)),disMin(Ystart,Yend,Xend,CameraType::stereo));
        return XYMinDistance;
    }
}

double LineDetected::AngleDiff(Vec4i X,Vec4i Y)
{
    Coordinate Xstart = {X[0],X[1]};
    Coordinate Xend = {X[2],X[3]};
    Coordinate Ystart = {Y[0],Y[1]};
    Coordinate Yend = {Y[2],Y[3]};

    if((Xend.X-Xstart.X) == 0 ) mX = 90;
    else mX = Slope(X);
    if((Yend.X-Ystart.X) == 0)  mY = 90;
    else mY = Slope(Y);;
    
    if( mX >= 0 && mY >= 0 || mX <= 0 && mY <= 0 ) return abs( mX - mY );
    else if( mX > 0 && mY < 0 ) return mX + abs(mY);
    else if( mX < 0 && mY > 0 ) return abs(mX) + mY;
}
    
Coordinate LineDetected::Midpoint(Vec4i line)
{
    Coordinate midpoint ;
    midpoint.X = (line[2]+line[0])/2;
    midpoint.Y = (line[3]+line[1])/2;
    return midpoint;
}

double LineDetected::Slope(Vec4i line)
{
    Point Xstart ;
    Point Xend ;
    if(line[0]>line[2])
    {
        Xstart.x = line[2];
        Xstart.y = line[3];
        Xend.x = line[0];
        Xend.y = line[1];
    }else{
        Xstart.x = line[0];
        Xstart.y = line[1];
        Xend.x = line[2];
        Xend.y = line[3];
    }

    if((line[2]-line[0]) == 0) return 90.0;
    else {
        double s = atan2((double(Xend.y)-double(Xstart.y)),(double(Xend.x)-double(Xstart.x)))*RAD2DEG;
        if(s > 180.0) return -360.0 + s;
        else if( s < -180.0) return 360.0 + s;
        else if(s < 0.0 && s > -180.0)return 180.0 + s;
        else return 180.0 - s;
    }
}

void LineDetected::Merge(Vec4i X,Vec4i Y)
{
    ROS_INFO("X = %d %d %d %d",X[0],X[1],X[2],X[3]);
    ROS_INFO("Y = %d %d %d %d",Y[0],Y[1],Y[2],Y[3]);
    //ROS_INFO("Merge");
    // clock_gettime(CLOCK_PROCESS_CPUTIME_ID, & time1);
    Coordinate Xm = Midpoint(X);
    Coordinate Ym = Midpoint(Y);
    Coordinate Xstart = {0,0};
    Coordinate Xend = {0,0};
    Coordinate Ystart = {0,0};
    Coordinate Yend = {0,0};
    if(X[0]>X[2])
    {
        Xstart = {X[2],X[3]};
        Xend = {X[0],X[1]};
    }else {
        Xstart = {X[0],X[1]};
        Xend = {X[2],X[3]};
    }
    if(Y[0]>Y[2])
    {
        Ystart = {Y[2],Y[3]};
        Yend = {Y[0],Y[1]};
    }else {
        Ystart = {Y[0],Y[1]};
        Yend = {Y[2],Y[3]};
    }
    
    XDistance = sqrt(dis2(Xstart,Xend));
    YDistance = sqrt(dis2(Ystart,Yend));
    ROS_INFO("XDistance = %f ",XDistance);
    ROS_INFO("YDistance = %f ",YDistance);
    
    if(XDistance >= YDistance) theta = Slope(X);
    else theta = Slope(Y);
    
    NewLine = {0,0,0,0};
    if(theta == 0.0)
    {
        thetaZeroY = round((Xstart.Y + Ystart.Y)/2);
        int thetaZeroX[4] =  { Xstart.X,Xend.X,Ystart.X,Yend.X};
        sort(thetaZeroX, thetaZeroX + 4);
        NewLine = {thetaZeroX[0],thetaZeroY,thetaZeroX[3],thetaZeroY};
    }else if(theta == 90.0){
        thetaZeroX = round((Xstart.X + Ystart.X)/2);
        int thetaZeroY[4] =  { Xstart.Y,Xend.Y,Ystart.Y,Yend.Y};
        sort(thetaZeroY, thetaZeroY + 4);
        NewLine = {thetaZeroX,thetaZeroY[0],thetaZeroX,thetaZeroY[3]};
    }
    else
    {
        double r = XDistance/(XDistance + YDistance);
        Coordinate P = { round((r * Xm.X)+((1-r)*Ym.X) ), round((r * Xm.Y) + ((1 - r) * Ym.Y))};
        double Za = tan ( theta * DEG2RAD);
        double Zb = double(P.Y) - (Za * double(P.X)); 

        double m = -(1.0/Za);
        double SbXstart = double(Xstart.Y)-(m*double(Xstart.X));
        double SbXend = double(Xend.Y)-(m*double(Xend.X));
        double SbYstart = double(Ystart.Y)-(m*double(Ystart.X));
        double SbYend = double(Yend.Y)-(m*double(Yend.X));

        //printf("Za = %f Zb =%f m = %f SbXstart = %f SbXend = %f SbYstart = %f SbYend = %f\n",Za,Zb,m,SbXstart,SbXend,SbYstart,SbYend);
        int x1 = round(-(Zb-SbXstart)/(Za-m));
        int y1 = round(((Za+m)*x1+(Zb+SbXstart))/2);
        Coordinate projXs={x1,y1};
        //printf("x1 = %d y1 =%d\n",x1,y1);
        int x2 = round(-(Zb-SbXend)/(Za-m));
        int y2 = round(((Za+m)*x2+(Zb+SbXend))/2);
        Coordinate projXe={x2,y2};
        //printf("x2 = %d y2 =%d\n",x2,y2);
        int x3 = round(-(Zb-SbYstart)/(Za-m));
        int y3 = round(((Za+m)*x3+(Zb+SbYstart))/2);
        Coordinate projYs={x3,y3};
        //printf("x3 = %d y3 =%d\n",x3,y3);
        int x4 = round(-(Zb-SbYend)/(Za-m));
        int y4 = round(((Za+m)*x4+(Zb+SbYend))/2);
        Coordinate projYe={x4,y4};
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
    // clock_gettime(CLOCK_PROCESS_CPUTIME_ID, & time2);
    // cout << "Merge total time (clock_gettime) = " << diff(time1, time2).tv_sec << ":" << diff(time1, time2).tv_nsec << endl;
    ROS_INFO("theta = %f, XDistance = %f, YDistance = %f NewLine = %f",theta,Slope(X),Slope(Y),Slope(NewLine));

    ROS_INFO("NewLine[0]=%d NewLine[1]=%d NewLine[2]=%d NewLine[3]=%d\n",NewLine[0],NewLine[1],NewLine[2],NewLine[3]);
}

int LineDetected::checkline(const Mat image_Enhance,const Mat canny,Vec4i line)
{ 
    // ROS_INFO("checkline");
    // clock_gettime(CLOCK_PROCESS_CPUTIME_ID, & time1);
    Point Xstart = Point(line[0],line[1]);
    Point Xend = Point(line[2],line[3]);
    int x1 = 0;
    int y1 = 0;
    int x2 = 0;
    int y2 = 0;
    
    if( Xstart.x > Xend.y )
    {
        x1 = Xend.x;
        y1 = Xend.y;
        x2 = Xstart.x;
        y2 = Xstart.y;
    }else{
        x1 = Xstart.x;
        y1 = Xstart.y;
        x2 = Xend.x;
        y2 = Xend.y;
    }
    float para_a = (float)((float)(y1-y2)/(float)(x1-x2));
    float para_b = (float)((float)((x1*y2)-(x2*y1))/(float)(x1-x2));
    float theta = Slope(line);
    ROS_INFO("theta = %f",theta);
    ROS_INFO("para = (%f,%f)",para_a,para_b);
    int Vgreen = 0;
    int Vwhite = 0;
    int Vedge = 0;
    int Verror = 0;

    // ROS_INFO("%d %d %d %d",x1,y1,x2,y2);
    int counter = 0;
    if(abs(x2-x1)>20)
    {
        counter = 5;
    }else if(abs(x2-x1)>10 && abs(x2-x1)<=20){
        counter = 3;
    }else{
        counter = 1;
    }
    int countfor = 0;
    ROS_INFO("counter = %d",counter);
    for(int i = 0; i <= abs(x2-x1); i=i+counter)
    {
        float x = 0.0;
        float y = 0.0;
        int dis = 0.0;
        if(theta == 0.0 || para_a == 0.0)
        {
            // ROS_INFO("theta 0");
            x = x1 + i;
            y = y1;
            for(int j = 0; j < 26; j++)
            {
                // ROS_INFO("j = %d",j);
                float x_1 = 0.0;
                float y_1 = 0.0;
                
                x_1 = x;
                y_1 = y - 13 + j;

                if(x_1>639||x_1<0)
                {
                    ROS_INFO("0 continue");
                    continue;
                }               
                if(y_1>479||y_1<0)
                {
                    ROS_INFO("0 continue");
                    continue;
                }
                // ROS_INFO("para_inv = (%f,%f)",para_a_inv,para_a_inv);
                // ROS_INFO("pos = (%f,%f)",x_1,y_1);
                int b = image_Enhance.at<Vec3b>((int)y_1, (int)x_1)[0];
                int g = image_Enhance.at<Vec3b>((int)y_1, (int)x_1)[1];
                int r = image_Enhance.at<Vec3b>((int)y_1, (int)x_1)[2];
                int edge = canny.at<uchar>((int)y_1, (int)x_1);
                // ROS_INFO("edge = %d",edge);
                double diff = 0;
                if(edge == 255)
                {               
                    Vedge++;
                }
                if(g >= 245 && b >=245 && r >= 245)
                {
                    Vwhite++;
                }
                else if(g >= 245 && b <=10 && r <= 10)
                {
                    Coordinate checkgreen = {x_1,y_1};
                    Coordinate checkpoint = {x,y};
                    diff = calculate_3D(checkgreen,checkpoint);
                    if(diff <= 5)
                    {
                        Vgreen++;
                    }
                }else{
                    Verror++;
                }
            }
        }else if(theta == 90.0 || !std::isfinite(para_a))
        {
            // ROS_INFO("theta 90");
            x = x1 ;
            y = y1 + i;
            for(int j = 0; j < 26; j++)
            {
                // ROS_INFO("j = %d",j);
                float x_1 = 0.0;
                float y_1 = 0.0;
                
                x_1 = x - 13 + j;
                y_1 = y;

                if(x_1>639||x_1<0)
                {
                    ROS_INFO("90 continue");
                    continue;
                }               
                if(y_1>479||y_1<0)
                {
                    ROS_INFO("90 continue");
                    continue;
                }
                // ROS_INFO("para_inv = (%f,%f)",para_a_inv,para_a_inv);
                // ROS_INFO("pos = (%f,%f)",x_1,y_1);
                int b = image_Enhance.at<Vec3b>((int)y_1, (int)x_1)[0];
                int g = image_Enhance.at<Vec3b>((int)y_1, (int)x_1)[1];
                int r = image_Enhance.at<Vec3b>((int)y_1, (int)x_1)[2];
                int edge = canny.at<uchar>((int)y_1, (int)x_1);
                // ROS_INFO("edge = %d",edge);
                double diff = 0;
                if(edge == 255)
                {               
                    Vedge++;
                }
                if(g >= 245 && b >=245 && r >= 245)
                {
                    Vwhite++;
                }
                else if(g >= 245 && b <=10 && r <= 10)
                {
                    Coordinate checkgreen = {x_1,y_1};
                    Coordinate checkpoint = {x,y};
                    diff = calculate_3D(checkgreen,checkpoint);
                    if(diff <= 5)
                    {
                        Vgreen++;
                    }
                }else{
                    Verror++;
                }
            }
        }else{
            // ROS_INFO("theta else");
            if(x2<x1)
            {
                x = x2 + i;
            }else{
                x = x1 + i;
            }
            if(x>639.0||x<0.0)
            {
                continue;
            }
            y = (x*para_a)+para_b;
            if(y>479.0||y<0.0)
            {
                continue;
            }
            float para_a_inv = (-1.0/para_a);
            float para_b_inv = y - (para_a_inv * x);
            for(int j = 0; j < 26; j++)
            {
                // ROS_INFO("j = %d",j);
                float x_1 = 0.0;
                float y_1 = 0.0;
                
                x_1 = float(x - 13 + j );
                                
                if(x_1>639.0||x_1<0.0)
                {
                    
                    ROS_INFO("else continue x_1 = %f",x_1);
                    continue;
                }
                y_1 = (x_1 * para_a_inv) + para_b_inv;
                // ROS_INFO("para_inv = (%f,%f)",para_a_inv,para_a_inv);
                // ROS_INFO("pos = (%f,%f)",x_1,y_1);
                if(y_1>479.0||y_1<0.0)
                {
                    ROS_INFO("else continue y_1 = %f",y_1);
                    continue;
                }
                
                int b = image_Enhance.at<Vec3b>((int)y_1, (int)x_1)[0];
                int g = image_Enhance.at<Vec3b>((int)y_1, (int)x_1)[1];
                int r = image_Enhance.at<Vec3b>((int)y_1, (int)x_1)[2];
                int edge = canny.at<uchar>((int)y_1, (int)x_1);
                // ROS_INFO("edge = %d",edge);
                double diff = 0;
                if(edge == 255)
                {               
                    Vedge++;
                }
                if(g >= 245 && b >=245 && r >= 245)
                {
                    Vwhite++;
                }
                else if(g >= 245 && b <=10 && r <= 10)
                {
                    Coordinate checkgreen = {(int)x_1,(int)y_1};
                    Coordinate checkpoint = {(int)x,(int)y};
                    diff = calculate_3D(checkgreen,checkpoint);
                    if(diff <= 10)
                    {
                        Vgreen++;
                    }
                }else{
                    Verror++;
                }
            }
        }
        countfor ++;
    }
    ROS_INFO("Vgreen = %d, Vwhite = %d,Vedge = %d",Vgreen,Vwhite,Vedge);
    float avg_G = (float)Vgreen/(float)(countfor);
    float avg_W = (float)Vwhite/(float)(countfor);
    float avg_E = (float)Vedge/(float)(countfor);
    ROS_INFO("avg_G = %f, avg_W = %f,avg_E = %f",avg_G,avg_W,avg_E);
    // clock_gettime(CLOCK_PROCESS_CPUTIME_ID, & time2);
    // cout << "checkline total time (clock_gettime) = " << diff(time1, time2).tv_sec << ":" << diff(time1, time2).tv_nsec << endl;
    if(avg_G >= 10.0 && avg_W >= 4.0 && avg_E >=0.4) return 1;
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

Mat LineDetected::Merge_similar_line(const Mat iframe,const Mat canny_iframe,const Mat ori_frame)
{
    // clock_gettime(CLOCK_PROCESS_CPUTIME_ID, & time1);
    all_lines.clear();
    all_lines1.clear();
    tmp.clear();
    reduce_similar_lines.clear();
    merge_similar_lines.clear();
    check_lines.clear();
    original_frame = iframe.clone();
    hough_frame = ori_frame.clone();
    merge_hough_frame = ori_frame.clone();
    // ROS_INFO("hough_threshold = %d hough_minLineLength = %f hough_maxLineGap = %f",hough_threshold,hough_minLineLength,hough_maxLineGap);
    HoughLinesP(canny_iframe,all_lines,1,CV_PI/180,hough_threshold,hough_minLineLength,hough_maxLineGap); 
    all_lines1 = all_lines;

    int all_lines_Size = all_lines.size();
    ROS_INFO("all_lines_Size = %d",all_lines_Size);
    // ROS_INFO("/---------------start-------------/");
    merge_similar_lines = all_lines;

    for(size_t i = 0 ; i < all_lines.size(); i++)
    {
        // ROS_INFO("all_lines %d",i);
        Vec4i X = all_lines[i];
        // if(LineorNot(X) == 0)
        // {
        //     merge_similar_lines = complement(merge_similar_lines,X);
        //     all_lines = complement(all_lines,X);
        //     continue;
        // }
        ROS_INFO("all_line(%d)=(x1 = %d ,y1 =%d, x2 =%d ,y2 =%d)  slope = %f",i,X[0],X[1],X[2],X[3],Slope(X));
        merge_similar_lines = complement(merge_similar_lines,X);
        int merge_similar_lines_SIZE = merge_similar_lines.size();
        int check = 0;

        if(all_lines.size() == 1)
        {
            int checklineX = checkline(nobackgroud_image,canny_iframe,X);
            ROS_INFO("checklineX = %d",checklineX);
            if(checklineX == 1)
            {
                check_lines.push_back(X);
            }
            break;

        }else{
            for(size_t j = 0 ; j < merge_similar_lines.size(); j++)
            {
                // ROS_INFO("merge_similar_lines %d",j);
                Vec4i Y = merge_similar_lines[j];
                // ROS_INFO("merge_similar_lines(%d)=(x1 = %d ,y1 =%d, x2 =%d ,y2 =%d)  slope = %f",j,Y[0],Y[1],Y[2],Y[3],Slope(Y));
                int checklineY = checkline(nobackgroud_image,canny_iframe,Y);
                if(checklineY == 0)
                {
                    merge_similar_lines = complement(merge_similar_lines,Y);
                    all_lines = complement(all_lines,Y);
                }else{
                    // ROS_INFO("merge_similar_lines(%d)=(x1 = %d ,y1 =%d, x2 =%d ,y2 =%d)  slope = %f",j,Y[0],Y[1],Y[2],Y[3],Slope(Y));
                    ROS_INFO("MinDistance = %f",MinDistance(X,Y));
                    ROS_INFO("AngleDiff = %f",AngleDiff(X,Y));
                    if(MinDistance(X,Y) < 10 && AngleDiff(X,Y) < 5 && LineorNot(Y) == 1)
                    {
                        // ROS_INFO("--------Merge--------");
                        Merge(X,Y);
                        int checklinenum = checkline(nobackgroud_image,canny_iframe,NewLine);
                        // ROS_INFO("checkline = %d \n",checklinenum);
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
                            // ROS_INFO("Not same Line");
                        } 
                                        
                    }else{                
                        //ROS_INFO("all_lines_Size (NO Merge ) '%d' = %d",j,all_lines_Size);
                    }
                }

            }   
        }
            
        int Maxdis =0;
        //ROS_INFO("reduce_similar_lines = %d",reduce_similar_lines.size());
        
        vector<Vec4i> tmp2;
        if(merge_similar_lines.size()>0)
        {
            if(reduce_similar_lines.size()>1)
            {
                // ROS_INFO("--------------------------");
                double th = Slope(reduce_similar_lines[0]);
                              
                for(size_t k=0 ; k < reduce_similar_lines.size() ; k++)
                {
                    Vec4i doublecheck1 = reduce_similar_lines[k];
                    
                    Coordinate NewLinestart;
                    Coordinate NewLineend;
                    Coordinate Xstart;
                    Coordinate Xend;
                    if(doublecheck1[0]>doublecheck1[2])
                    {
                        Xstart = {doublecheck1[2],doublecheck1[3]};
                        Xend = {doublecheck1[0],doublecheck1[1]};
                    }else{
                        Xstart = {doublecheck1[0],doublecheck1[1]};
                        Xend = {doublecheck1[2],doublecheck1[3]};
                    } 

                    if(k == 0)
                    {
                        NewLinestart = Xstart;
                        NewLineend = Xend;
                    }

                    if(th == 90.0)
                    {
                        if (Xstart.Y < NewLinestart.Y ){
                            NewLinestart = {Xstart.X,Xstart.Y};
                        }else if(Xstart.Y > NewLineend.Y )
                        {
                            NewLineend = {Xend.X,Xend.Y};
                        }else if(Xend.Y < NewLinestart.Y )
                        {
                            NewLinestart = {Xend.X,Xend.Y};
                        }
                        else if (Xend.Y > NewLineend.Y ){
                            NewLineend = {Xend.X,Xend.Y};
                        }  
                    }else{
                        if (Xstart.X < NewLinestart.X ){
                            NewLinestart = {Xstart.X,Xstart.Y};
                        }else if(Xstart.X > NewLineend.X )
                        {
                            NewLineend = {Xend.X,Xend.Y};
                        }else if(Xend.X < NewLinestart.X )
                        {
                            NewLinestart = {Xend.X,Xend.Y};
                        }
                        else if (Xend.X > NewLineend.X ){
                            NewLineend = {Xend.X,Xend.Y};
                        }  
                    }
                    
                    MaxLine = {NewLinestart.X,NewLinestart.Y,NewLineend.X,NewLineend.Y};
                    
                }
                // ROS_INFO("MaxLine = (x1= %d ,y1 = %d ,x2 = %d ,y2 = %d)",MaxLine[0],MaxLine[1],MaxLine[2],MaxLine[3]);
                check_lines.push_back(MaxLine);
            }
            if(reduce_similar_lines.size() == 1)
            {
                //ROS_INFO("push_back(reduce_similar_lines[0])");
                // ROS_INFO("reduce_similar_lines = (x1= %d ,y1 = %d ,x2 = %d ,y2 = %d)",reduce_similar_lines[0][0],reduce_similar_lines[0][1],reduce_similar_lines[0][2],reduce_similar_lines[0][3]);
                check_lines.push_back(reduce_similar_lines[0]);
            }else if(merge_similar_lines.size() == merge_similar_lines_SIZE){
                int checklinenum = checkline(nobackgroud_image,canny_iframe,X);
                if(checklinenum == 1)
                {
                    check_lines.push_back(X);
                }else{
                    //ROS_INFO("not line");
                }
                
                // ROS_INFO("X = (x1= %d ,y1 = %d ,x2 = %d ,y2 = %d)",X[0],X[1],X[2],X[3]);
            }
            reduce_similar_lines.clear();
        }else if(reduce_similar_lines.size() == 1)
        {
            // ROS_INFO("reduce_similar_lines = (x1= %d ,y1 = %d ,x2 = %d ,y2 = %d)",reduce_similar_lines[0][0],reduce_similar_lines[0][1],reduce_similar_lines[0][2],reduce_similar_lines[0][3]);
            check_lines.push_back(reduce_similar_lines[0]);
        }else if(check != 0){
            // ROS_INFO("X = (x1= %d ,y1 = %d ,x2 = %d ,y2 = %d)",X[0],X[1],X[2],X[3]);
            check_lines.push_back(X);
        }
        reduce_similar_lines.clear();    
        all_lines_Size ++;
    }
    ROS_INFO("check_lines.size = %d",check_lines.size());

    // ROS_INFO("/---------------finish-------------/");
    
    for(size_t i = 0 ; i < all_lines1.size(); i++)
    {
        Vec4i X = all_lines1[i];
        line( hough_frame, Point(X[0], X[1]), Point(X[2], X[3]), Scalar(255,0,0), 2, CV_AA);
    }
    sort(check_lines.begin(), check_lines.end(), tocompare);
    for( size_t i = 0; i < check_lines.size(); i++ )
	{
        Vec4i l = check_lines[i]; 
        // ROS_INFO("check_lines %d",i);
        if(LineorNot(l) == 0) 
        {
            check_lines = complement(check_lines,l);
            continue;
        }
        
        ROS_INFO("check_lines = (x1= %d ,y1 = %d ,x2 = %d ,y2 = %d)",l[0],l[1],l[2],l[3]);
        Coordinate startp;
        Coordinate endp;
        if(l[3]>l[1])
        {
            startp = {l[2],l[3]};
            endp = {l[0],l[1]};
        }else{
            startp = {l[0],l[1]};
            endp = {l[2],l[3]};
        }
        Coordinate pointP = {320,480};
        Coordinate midpoint = Midpoint(l);
        // Distance middlepoint = measure(midpoint.X,midpoint.Y,CameraType::stereo);
        Distance start_ = measure(startp.X,startp.Y,CameraType::stereo);
        Distance end_ = measure(endp.X,endp.Y,CameraType::stereo);
        double dis =  sqrt(pow((start_.x_dis - end_.x_dis),2) + pow((start_.y_dis - end_.y_dis),2));
        // Coordinate FOV_Bottom_POS = {image_bottom_width_length/2,0};
        // double AdotB = FOV_Bottom_POS.X*middlepoint.x_dis + FOV_Bottom_POS.Y*middlepoint.y_dis;
        
        tku_msgs::LineData line_tmp;
        tku_msgs::Cooridinate mid_point;
        mid_point.x = midpoint.X;
        mid_point.y = midpoint.Y;
        tku_msgs::Cooridinate startpoint;
        startpoint.x = startp.X;
        startpoint.y = startp.Y;
        tku_msgs::Cooridinate endpoint;
        endpoint.x = endp.X;
        endpoint.y = endp.Y;
        // tku_msgs::Distance relative_dis;
        double mindis = disMin(startp,endp,pointP,CameraType::Monocular);//點P到線段AB的最短距離
        Point minIntersectPoint = MinIntersectPoint(l,Point(320,479),mindis);
        Distance relativedis = measure(minIntersectPoint.x,minIntersectPoint.y,CameraType::stereo);
        tku_msgs::Cooridinate IntersectPoint;
        IntersectPoint.x = minIntersectPoint.x;
        IntersectPoint.y = minIntersectPoint.y;
        // relative_dis.dis =  relativedis.dis;
        line_tmp.start_point = startpoint;
        line_tmp.end_point = endpoint;
        line_tmp.center_point = mid_point;
        line_tmp.Line_length = dis;
        line_tmp.Line_theta = Slope(l);
        line_tmp.relative_distance = relativedis.dis;
        line_tmp.Nearest_point = IntersectPoint;
        // line_tmp.Line_theta = acos((AdotB)/((image_bottom_width_length/2)*(middlepoint.dis)))*180/CV_PI;
        ROS_INFO("Line %d : %d %d %d %d (x,y) = (%d ,%d) , distance =  %d", i,l[0],l[1],l[2],l[3],minIntersectPoint.x,minIntersectPoint.y,relativedis.dis);
        JustLine_Data.landmark.push_back(line_tmp);
 
        line( merge_hough_frame, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 2, CV_AA);
        circle(merge_hough_frame, Point(midpoint.X, midpoint.Y), 1, Scalar(0, 255, 0), -1);
    }
    // imshow("hough_frame",hough_frame);
    // imshow("merge_hough_frame",merge_hough_frame);
    // clock_gettime(CLOCK_PROCESS_CPUTIME_ID, & time2);
    // cout << "Merge_similar_line total time (clock_gettime) = " << diff(time1, time2).tv_sec << ":" << diff(time1, time2).tv_nsec << endl;
    return merge_hough_frame;
}


//-----------Hough threshold-------------------
void LineDetected::LoadHoughFile()
{
    fstream fin;
    char line[100]; 
    char path[200];
    std::string PATH = tool->getPackagePath("strategy");
    strcpy(path, PATH.c_str());
    strcat(path, "/Hough_Value.ini");
    
    fin.open(path, ios::in);
    //fin.open(("../../Parameter/Color_Model_Data/ColorModelData.ini"), ios::in);
    try
    {
        ROS_INFO("path = %s",path);
        fin.getline(line,sizeof(line),'\n');
        hough_threshold = tool->readvalue(fin, "Hough_threshold", 0);
        hough_minLineLength = tool->readvalue(fin, "Hough_minLineLength", 0);
        hough_maxLineGap = tool->readvalue(fin, "Hough_maxLineGap", 0);
        ROS_INFO("hough_threshold = %d hough_minLineLength = %d hough_maxLineGap = %d",hough_threshold,hough_minLineLength,hough_maxLineGap);

        fin.close();
    }
    catch(exception e)
    {
    }
}
void LineDetected::SaveHoughFile()
{
    char path[200];
    printf("%s",path);
    std::string PATH = tool->getPackagePath("strategy");
    strcpy(path, PATH.c_str());
    strcat(path, "/Hough_Value.ini");
    try
    {
//       ofstream OutFile(sFileName.c_str());
        ofstream OutFile(path);
        OutFile << "[HoughRange]";
        OutFile << "\n";
        OutFile << "Hough_threshold = ";
        OutFile << hough_threshold;
        OutFile << "\n";
        OutFile << "Hough_minLineLength = ";
        OutFile << hough_minLineLength;
        OutFile << "\n";
        OutFile << "Hough_maxLineGap = ";
        OutFile << hough_maxLineGap;
        OutFile << "\n";
        OutFile.close();
    }
    catch( exception e )
    {
    }
}

Mat LineDetected::fitLineRANSAC(Mat ori,Mat drawing,vector<vector<Point> > allfieldpoints)
{
    vector<Point> Allgreenhull;

    for(int i=0; i < allfieldpoints.size(); i++)
    {
        Allgreenhull.insert(Allgreenhull.end(), allfieldpoints[i].begin(), allfieldpoints[i].end());
    }
  
    vector<Point> filteredpoints;
    vector<Point> upperCH;
    
    vector<Point> points = Allgreenhull;
    int n_points = points.size();

    sort(points.begin(), points.end(), sortPoints);
 
    //Computing upper convex hull
    upperCH.push_back(points[n_points-1]);
    upperCH.push_back(points[n_points-2]);
    
    for(int i=2; i< n_points; i++)
    {
        while(upperCH.size() > 1 and (!right_turn(upperCH[upperCH.size()-2],upperCH[upperCH.size()-1], points[n_points-i-1])))
            upperCH.pop_back();
        upperCH.push_back(points[n_points-i-1]);
    }
    for ( size_t i = 0; i < upperCH.size(); i++ )
    {
        circle(drawing,upperCH[i],3,Scalar(255,0,255),CV_FILLED,-1);	
    }
    vector<vector<Point> > contours(1);
    convexHull(Mat(upperCH), contours[0], false );

    for ( size_t i = 0; i < contours.size(); i++ )
    {
		drawContours( drawing, contours, i, Scalar(255,0,255), -1);
	}

    Vec4i lineParam = {0,0,0,0};
    Point ls = Point(0,0);
    Point le = Point(0,0);    
    unsigned int n = upperCH.size();
    if(n<2)
    {
        return drawing;
    }
    double kmax = 5.;
    double kmin = -5.;
    RNG rng;
    double bestscore = -1.;
    for(int i=0; i<1000;i++)
    {
        int i1 = 0;
        int i2 = 0;
        while(i1==i2)
        {
            i1 = rng(n);
            i2 = rng(n);
        }
        const Point& p1 = upperCH[i1];
        const Point& p2 = upperCH[i2];

        Point2f dp = p2 - p1;
        
        dp *= 1./norm(dp);
        double score = 0.0;
        if((dp.y/dp.x) <= kmax && (dp.y/dp.x) >= kmin)
        {    

            for(int j=0; j<n; j++)
            {    
                Point2f checkpoint = upperCH[j]-p1;
                double d = (checkpoint.y * dp.x) - (checkpoint.x*dp.y);
                if(fabs(d)< 10)
                    score += 1.;
            }
        }
        if(score > bestscore)
        {
            lineParam = Vec4i(int(dp.x),int(dp.y),int(p1.x),int(p1.y));
            bestscore = score;
        }
    } 
    // double k = (double)lineParam[1] / (double)lineParam[0];
    // double b = (double)lineParam[3] - k*(double)lineParam[2];

    // cv::Point p1,p2;
    // p1.y = 480;
    // p1.x = round(( p1.y - b) / k);

    // p2.y = 240;
    // p2.x = round((p2.y-b) / k);
    
    // line(ori, p1, p2, Scalar(255, 255, 0), 3);
    // imshow("ori",ori);
    // waitKey(0);

    return drawing;
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
