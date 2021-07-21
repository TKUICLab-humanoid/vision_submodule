#include <LineDetected/LineDetected.h>

LineDetected::LineDetected()
{
    mX = 0.0;
    mY = 0.0;

    R_value = 150;
    G_value = 220;
    B_value = 220;
    hough_threshold = 100;
    hough_minLineLength = 60;
    hough_maxLineGap = 40;

}
LineDetected::~LineDetected()
{
    
}

Mat LineDetected::ImagePreprocessing(const Mat iframe)
{
    orign = iframe.clone();
    // bilateralFilter(iframe, orign, 10, 10, 10);
    blur(orign,orign,Size(4,4));
    int frame_rows = iframe.rows;
    int frame_cols = iframe.cols;
    // imshow("orign",orign);
    //濾除非場地部份(laplace)
    Mat mask = Mat::zeros(orign.rows,orign.cols, CV_8UC3); 
    Mat Kernel = (Mat_<float>(3, 3) << 0, -1, 0, -1, 6, -1, 0, -1, 0);
    Mat imageEnhance;
    filter2D(orign, imageEnhance, CV_8UC3, Kernel);
    // imshow("imageEnhance",imageEnhance);
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
    
    // imshow("imageGamma",imageGamma);
    R_value = Model_Base->BGRColorRange->ReValue;
    G_value = Model_Base->BGRColorRange->GrValue;
    B_value = Model_Base->BGRColorRange->BuValue;
    // ROS_INFO("R_ = %d,G_ = %d,B_ = %d",R_value,G_value,B_value);
    
    vector<Point> BoundaryPoint;
    int horization_line = 0;
    if(std::isfinite(RealsenseIMUData[0]) && round(RealsenseIMUData[0]) >= 45.0)
    {
        horization_line = 150 - (5 * round(90.0 - RealsenseIMUData[0]));
        if(horization_line <= 0)
        {
            horization_line = 0;
        }
    }else{
        horization_line = 0;
    }
    // ROS_INFO("horization_line = %d",horization_line);
    for(int col = 0; col < imageGamma.cols;col++)
    {
        int score = 0;
        int Minscore = 0;
        PointScore S = {Point(col,horization_line),0};        
        // vector<PointScore> PixelScore;
        for(int row = horization_line; row < imageGamma.rows+1;row++)
        {
            int B = imageGamma.at<Vec3b>(row-1, col)[0];
            int G = imageGamma.at<Vec3b>(row-1, col)[1];
            int R = imageGamma.at<Vec3b>(row-1, col)[2];
            
            // if(row >= horization_line && horization_line >= 0)
            // {
            if( G >= G_value && B >= B_value && R <= R_value)
            {
                score = score+1;
                // S.pixelpoint = Point(col,row-1);
                // S.Score = score;
            }else
            {
                score = score-1;
                // S.pixelpoint = Point(col,row-1);
                // S.Score = score;
            }
            if(score < Minscore)
            {
                Minscore = score;
                S.Score = Minscore;
                S.pixelpoint = Point(col,row-1);
            }
            // ROS_INFO("score(%d,%d) = %d",col,row-1,score);
            // }
        }
        // PixelScore.push_back(S); 
        // sort(PixelScore.begin(),PixelScore.end(),Scorecompare);
        BoundaryPoint.push_back(S.pixelpoint);
    }

    for(int i = 0; i < BoundaryPoint.size();i++)
    {
        Point boundary = BoundaryPoint[i];
        for(int k = 0 ; k < mask.rows ;k++)
        {
            if( k > boundary.y)
            {
                mask.at<Vec3b>(k, boundary.x)[0] = 255;
                mask.at<Vec3b>(k, boundary.x)[1] = 255;
                mask.at<Vec3b>(k, boundary.x)[2] = 255;
            }else{
                mask.at<Vec3b>(k, boundary.x)[0] = 0;
                mask.at<Vec3b>(k, boundary.x)[1] = 0;
                mask.at<Vec3b>(k, boundary.x)[2] = 0;
            }
        }
    }
    
    Mat mask_element = getStructuringElement(MORPH_RECT, Size(8, 8)); 
    // imshow("mask1",mask);
    dilate(mask,mask,mask_element);
    // imshow("mask2",mask);
    // Mat maskno ;
    // orign.copyTo(maskno,mask);
    // imshow("mask3",maskno);
    cvtColor(mask,mask,COLOR_BGR2GRAY);
    threshold(mask,mask,200,255,THRESH_BINARY);
    
    nobackgroud_image = Mat::zeros(orign.rows,orign.cols, CV_8UC3); 

    //顯示BoundaryPoint
    // Mat orign3 = orign.clone(); 
    // // ROS_INFO("start");
    // for( int j = 0; j< BoundaryPoint.size(); j++ )
    // {
    //     Point p = BoundaryPoint[j];
    //     // ROS_INFO("BoundaryPoint(%d,%d)",p.x,p.y);
    //     if(p.y < iframe.rows-4)
    //     {
    //         circle(orign3,p,2,Scalar(0,0,255),CV_FILLED,-1);
    //     }
    // }
    // imshow("orign3",orign3);
    // ROS_INFO("finish");

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours( mask, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

    Mat green_mask( orign.size(), CV_8U, Scalar(0));
    Mat green_maskw( orign.size(), CV_8U, Scalar(255));
    Gmask = Mat::zeros(orign.rows,orign.cols, CV_8U); 
    
    // Find the convex hull object for each contour
    vector<vector<Point> >hull(contours.size());
    vector<vector<Point> >greenhull;
    // ROS_INFO("----------------contours = %d",contours.size());
    
    if(contours.size() != 0)
    {
        for( int i = 0; i < contours.size(); i++ )
        {     
            // convexHull( Mat(contours[i]), hull[i], false );
            greenhull.push_back(contours[i]);
        }
        // if(greenhull.size() < 1 && hull.size() >= 1)
        // {
        //     Gmask = UpperConvexHull(orign,green_mask,hull);
        // }else 
        if(greenhull.size() >= 1 && hull.size() >= 1){
            Gmask = UpperConvexHull(orign,green_mask,greenhull,horization_line);
        }else{
            Gmask = green_maskw;
        }
    }else{
        Gmask = green_maskw;
    }

    // ROS_INFO("----------------hull = %d",hull.size());  
    // ROS_INFO("----------------greenhull = %d",greenhull.size()); 
    imageGamma.copyTo(nobackgroud_image,Gmask);
    
    // imshow("nobackgroud_image",nobackgroud_image);
    for(int col = 0; col < nobackgroud_image.cols;col++)
    {
        for(int row = nobackgroud_image.rows -1 ; row > 0 ;row--)
        {
            int B = nobackgroud_image.at<Vec3b>(row, col)[0];
            int G = nobackgroud_image.at<Vec3b>(row, col)[1];
            int R = nobackgroud_image.at<Vec3b>(row, col)[2];
            if( B <= 50 && G >= 220 && R <= 230)
            {
                nobackgroud_image.at<Vec3b>(row, col)[0] = 0;
                nobackgroud_image.at<Vec3b>(row, col)[1] = 255;
                nobackgroud_image.at<Vec3b>(row, col)[2] = 0;
            }else if( B <= 20 && G <= 20 && R <= 50)
            {
                nobackgroud_image.at<Vec3b>(row, col)[0] = 0;
                nobackgroud_image.at<Vec3b>(row, col)[1] = 0;
                nobackgroud_image.at<Vec3b>(row, col)[2] = 0;
            }else{
                nobackgroud_image.at<Vec3b>(row, col)[0] = 255;
                nobackgroud_image.at<Vec3b>(row, col)[1] = 255;
                nobackgroud_image.at<Vec3b>(row, col)[2] = 255;
            }
        }
    }

    contours.clear();
    hierarchy.clear();
    greenhull.clear();
    hull.clear();
    
    // imshow("nobackgroud_image1",nobackgroud_image);
    // resize(nobackgroud_image, nobackgroud_image, cv::Size(320, 240));
    return nobackgroud_image;
}

Mat LineDetected::ImageCanny(const Mat iframe)
{     
    edge = iframe.clone() ;
    cvtColor(edge,edge,COLOR_BGR2GRAY);
    threshold(edge,edge,220,255,THRESH_BINARY);
    Mat greenmask_element = getStructuringElement(MORPH_RECT, Size(6, 6)); 
    Mat greenmask_element5 = getStructuringElement(MORPH_RECT, Size(3, 3));
    // imshow("edge1",edge);
    dilate(edge,edge,greenmask_element5);
    morphologyEx(nobackgroud_image,nobackgroud_image,MORPH_CLOSE,greenmask_element);
    // erode(nobackgroud_image,nobackgroud_image,greenmask_element5);
    morphologyEx(nobackgroud_image,nobackgroud_image,MORPH_OPEN,greenmask_element5);

    // imshow("edge2",edge);
    Canny(edge, edge, 100, 150, 3);
    //imshow("GreenField1",GreenField);
    // imshow("edge",edge);
    return  edge ;
}

Pixel3Dpoint LineDetected::deproject_pixel2point(Coordinate point,float depth)
{
    // ROS_INFO("deproject_pixel2point");
    Pixel3Dpoint pixel3Dpoint = {0,0,0};
    // Intrinsicscolor color_Intrinsics={312.2850,249.5522,615.0078,615.1781};
    Intrinsicscolor color_Intrinsics={329.0786,242.4088,606.2370,606.1384};//
    pixel3Dpoint.x = depth * (point.X - color_Intrinsics.PPX)/color_Intrinsics.Fx;
    pixel3Dpoint.y = depth * (point.Y - color_Intrinsics.PPY)/color_Intrinsics.Fy;
    pixel3Dpoint.z = depth;
 
    return pixel3Dpoint;
}
double LineDetected::calculate_3D(Coordinate a, Coordinate b)
{
    // ROS_INFO("calculate_3D");
    float pointA = 0.0;
    float pointB = 0.0;
    double dist = 0.0;
    if(!depth_buffer.empty())
    {
        pointA = AvgPixelDistance(a.X, a.Y);
        pointB = AvgPixelDistance(b.X, b.Y);
        
    }
    // ROS_INFO("pointA = %f,pointB = %f",pointA,pointB);
    Pixel3Dpoint A = {0,0,0};
    Pixel3Dpoint B = {0,0,0};
    if((pointA && pointB) != 0.0 && std::isfinite(pointA) && std::isfinite(pointB))
    {
        // ROS_INFO("!= 0");
        A = deproject_pixel2point(a,pointA);
        B = deproject_pixel2point(b,pointB);
        dist = sqrt(pow((A.x-B.x),2)+pow((A.y-B.y),2)+pow((A.z-B.z),2));
    }else{
        // ROS_INFO("== 0");
        dist = sqrt(dis2(a,b));
    }
    // ROS_INFO("A.x = %f,A.y = %f,A.z = %f",A.x,A.y,A.z);
    // ROS_INFO("B.x = %f,B.y = %f,B.z = %f",B.x,B.y,B.z);
    
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
    stereo_flag = true;
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
            stereo_flag = false;
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
        stereo_flag = false;
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
    // ROS_INFO("stereo_flag = %d",stereo_flag);
    
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
        return 0.0;
    }else{                                                   
        //如不相交, 則最短距離為每個端點到另一條線段距離的最小值
               
        Point Xstart1 = Point(X[0],X[1]);
        Point Xend1 = Point(X[2],X[3]);
        Point Ystart1 = Point(Y[0],Y[1]);
        Point Yend1 = Point(Y[2],Y[3]);
        int x1 = 0;
        int y1 = 0;
        int x2 = 0;
        int y2 = 0;
        int x11 = 0;
        int y11 = 0;
        int x21 = 0;
        int y21 = 0;
        if( Xstart1.x > Xend1.x )
        {
            x1 = Xend1.x;
            y1 = Xend1.y;
            x2 = Xstart1.x;
            y2 = Xstart1.y;
        }else{
            x1 = Xstart1.x;
            y1 = Xstart1.y;
            x2 = Xend1.x;
            y2 = Xend1.y;
        }
        if( Ystart1.x > Yend1.x )
        {
            x11 = Yend1.x;
            y11 = Yend1.y;
            x21 = Ystart1.x;
            y21 = Ystart1.y;
        }else{
            x11 = Ystart1.x;
            y11 = Ystart1.y;
            x21 = Yend1.x;
            y21 = Yend1.y;
        }
        float para_a = (float)((float)(y1-y2)/(float)(x1-x2));
        float para_b = ((float)((x1*y2)-(x2*y1))/(float)(x1-x2));

        float para_a2 = (float)((float)(y11-y21)/(float)(x11-x21));
        float para_b2 = ((float)((x11*y21)-(x21*y11))/(float)(x11-x21));
        // ROS_INFO("para_a = %f para_b = %f para_a2 = %f para_b2 = %f ",para_a,para_b,para_a2,para_b2);
        if(para_a == para_a2)
        {
            if(abs(para_b-para_b2) < 55.0)
            {
                return 1;
            }
            
        }else
        {
            double XYMinDistance = min(min(min(disMin(Xstart, Xend, Ystart,CameraType::stereo), disMin(Xstart, Xend, Yend,CameraType::stereo)), disMin(Ystart, Yend, Xstart,CameraType::stereo)),disMin(Ystart,Yend,Xend,CameraType::stereo));
            return XYMinDistance;
        }
        // ROS_INFO("s");
    }
}

double LineDetected::AngleDiff(Vec4i X,Vec4i Y)
{
    Coordinate Xstart = {X[0],X[1]};
    Coordinate Xend = {X[2],X[3]};
    if(X[0]>X[2])
    {
        Xstart.X = X[2];
        Xstart.Y = X[3];
        Xend.X = X[0];
        Xend.Y = X[1];
    }else{
        Xstart.X = X[0];
        Xstart.Y = X[1];
        Xend.X = X[2];
        Xend.Y = X[3];
    }   
    Coordinate Ystart = {Y[0],Y[1]};
    Coordinate Yend = {Y[2],Y[3]};

    if((Xend.X-Xstart.X) == 0 ) mX = 90.0;
    else mX = Slope(X);
    if((Yend.X-Ystart.X) == 0)  mY = 90.0;
    else mY = Slope(Y);;
    
    double angle = 0.0;
    if(abs(mX - mY) > 175.0)
    {
        angle = 180.0 - abs(mX - mY);
    }
    else{
        angle = abs(mX - mY);
    }
    
    return angle;
    // if( mX >= 0 && mY >= 0 || mX <= 0 && mY <= 0 ) return abs( mX - mY );
    // else if( mX > 0 && mY < 0 ) return mX + abs(mY);
    // else if( mX < 0 && mY > 0 ) return abs(mX) + mY;
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
        double s = normalize_angle(atan2((double(Xend.y)-double(Xstart.y)),(double(Xend.x)-double(Xstart.x))) * RAD2DEG );
        // ROS_INFO("line %d %d %d %d theta = %f",line[0],line[1],line[2],line[3],s);
        if(s < 0.0) return 180.0 + s;
        else return s;
    }
}

void LineDetected::Merge(Vec4i X,Vec4i Y)
{
    // ROS_INFO("X = %d %d %d %d",X[0],X[1],X[2],X[3]);
    // ROS_INFO("Y = %d %d %d %d",Y[0],Y[1],Y[2],Y[3]);
    //ROS_INFO("Merge");
    
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
    Vec4i XX = {Xstart.X,Xstart.Y,Xend.X,Xend.Y};
    Vec4i YY = {Ystart.X,Ystart.Y,Yend.X,Yend.Y};
    Coordinate Xm = Midpoint(XX);
    Coordinate Ym = Midpoint(YY);

    XDistance = double(sqrt(dis2(Xstart,Xend)));
    YDistance = double(sqrt(dis2(Ystart,Yend)));
    
    
    if(XDistance >= YDistance) theta = Slope(XX);
    else theta = Slope(YY);
    // ROS_INFO("Slope(X) = %f ",Slope(X));
    // ROS_INFO("Slope(Y) = %f ",Slope(Y));
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
        Coordinate P = { round((r * Xm.X)+((1.0-r)*Ym.X) ), round((r * Xm.Y) + ((1.0 - r) * Ym.Y))};
        double Za = tan( theta * DEG2RAD);
        double Zb = double(P.Y) - (Za * double(P.X)); 

        double m = -(1.0/Za);
        double SbXstart = double(Xstart.Y)-(m*double(Xstart.X));
        double SbXend = double(Xend.Y)-(m*double(Xend.X));
        double SbYstart = double(Ystart.Y)-(m*double(Ystart.X));
        double SbYend = double(Yend.Y)-(m*double(Yend.X));

        //printf("Za = %f Zb =%f m = %f SbXstart = %f SbXend = %f SbYstart = %f SbYend = %f\n",Za,Zb,m,SbXstart,SbXend,SbYstart,SbYend);
        int x1 = round(-(Zb-SbXstart)/(Za-m));
        int y1 = round((((Za+m)*x1)+(Zb+SbXstart))/2);
        Coordinate projXs={x1,y1};
        // printf("x1 = %d y1 =%d\n",x1,y1);
        int x2 = round(-(Zb-SbXend)/(Za-m));
        int y2 = round((((Za+m)*x2)+(Zb+SbXend))/2);
        Coordinate projXe={x2,y2};
        // printf("x2 = %d y2 =%d\n",x2,y2);
        int x3 = round(-(Zb-SbYstart)/(Za-m));
        int y3 = round((((Za+m)*x3)+(Zb+SbYstart))/2);
        Coordinate projYs={x3,y3};
        // printf("x3 = %d y3 =%d\n",x3,y3);
        int x4 = round(-(Zb-SbYend)/(Za-m));
        int y4 = round((((Za+m)*x4)+(Zb+SbYend))/2);
        Coordinate projYe={x4,y4};
        // printf("x4 = %d y4 =%d\n",x4,y4);

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
            // ROS_INFO("1");
            NewLine = {x1,y1,x2,y2};
        }
        if(distanceset[5] == projXsToprojYs )
        {
            // ROS_INFO("2");
            NewLine = {x1,y1,x3,y3};
        }
        if(distanceset[5] == projXsToprojYe )
        {
            // ROS_INFO("3");
            NewLine = {x1,y1,x4,y4};
        }
        if(distanceset[5] == projXeToprojYs )
        {
            // ROS_INFO("4");
            NewLine = {x2,y2,x3,y3};
        }
        if(distanceset[5] == projXeToprojYe )
        {
            // ROS_INFO("5");
            NewLine = {x2,y2,x4,y4};
        }
        if(distanceset[5] == projYsToprojYe )
        {
            // ROS_INFO("6");
            NewLine = {x3,y3,x4,y4};
        }

    }
    // ROS_INFO("theta = %f,Slope(X) = %f, Slope(Y) = %f NewLine = %f",theta,Slope(X),Slope(Y),Slope(NewLine));

    // ROS_INFO("NewLine[0]=%d NewLine[1]=%d NewLine[2]=%d NewLine[3]=%d\n",NewLine[0],NewLine[1],NewLine[2],NewLine[3]);
}

int LineDetected::checkline(const Mat image_Enhance,const Mat canny,Vec4i line)
{ 
    // ROS_INFO("checkline");
    Point Xstart = Point(line[0],line[1]);
    Point Xend = Point(line[2],line[3]);
    
    int x1 = 0;
    int y1 = 0;
    int x2 = 0;
    int y2 = 0;
    
    if( Xstart.x > Xend.x )
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

    int Vgreen = 0;
    int Vwhite = 0;
    int Vedge = 0;
    int Verror = 0;

    Point Linelength = Point(x2,y2)-Point(x1,y1);
    int Linestep = ceil(max(fabs(Linelength.x),fabs(Linelength.y)));
    Point Linegap = Linelength / Linestep;

    Point p1 = Point(x1,y1);
    // ROS_INFO("line %d %d %d %d, theta = %f,para = (%f,%f) ",x1,y1,x2,y2,theta,para_a,para_b);
    // ROS_INFO("Linelength = %d %d",Linelength.x,Linelength.y);
    // ROS_INFO("Linestep = %d",Linestep);
    // ROS_INFO("Linegap = %d %d",Linegap.x,Linegap.y);

    int counter = 0;
    if(Linestep > 100)
    {
        counter = 10;
    }else if(Linestep > 20)
    {
        counter = 5;
    }else if(Linestep > 10 && Linestep <= 20){
        counter = 3;
    }else{
        counter = 1;
    }
    int countfor = 0;
    // ROS_INFO("counter = %d",counter);
    for(int i = 0; i < Linestep; i = i + counter)
    {
        int x = round(p1.x);
        int y = round(p1.y);
        if(!onImage(x, y))
        {
            p1 = p1 + (Linegap * counter);
            continue;
        } 

        int dis = 0;
        if(theta == 0.0 || round(para_a == 0.0))
        {
            // ROS_INFO("theta 0");
            int x_1 = x;
            int y_1 = y - 5;   
            Point checkstart = Point(x_1,y_1);
            int x_2 = x ;
            int y_2 = y + 5;   
            Point checkend = Point(x_2,y_2);

            Point length = checkend - checkstart;
            int step = ceil(max(fabs(length.x),fabs(length.y)));
            Point gap = length/step;

            for(int j = 0; j < step ; j++)
            {
                // ROS_INFO("j = %d",j);
                int checkx = round(checkstart.x);
                int checky = round(checkstart.y);
                  
                if(!onImage(checkx, checky))
                {
                    checkstart = checkstart + gap;
                    continue;
                }                
                int b = image_Enhance.at<Vec3b>(checky, checkx)[0];
                int g = image_Enhance.at<Vec3b>(checky, checkx)[1];
                int r = image_Enhance.at<Vec3b>(checky, checkx)[2];
                int edge_ = canny.at<uchar>(checky, checkx);
                // ROS_INFO("edge = %d",edge_);
                double diff = 0.0;
                if(edge_ == 255)
                {               
                    Vedge++;
                }
                if(g >= 245 && b >=245 && r >= 245)
                {
                    Vwhite++;
                }
                else if(g >= 245 && b <=10 && r <= 10)
                {
                    Coordinate checkgreen = {checkx,checky};
                    Coordinate checkpoint = {(int)x,(int)y};
                    // ROS_INFO("checkgreen = %d %d",checkgreen.X,checkgreen.Y);
                    // ROS_INFO("checkpoint = %d %d",checkpoint.X,checkpoint.Y);
                    diff = calculate_3D(checkgreen,checkpoint);
                    // ROS_INFO("diff = %f",diff);
                    if(diff <= 35)
                    {
                        Vgreen++;
                    }
                }else{
                    Verror++;
                }
                checkstart = checkstart + gap;
            }
        }else if(theta == 90.0)
        {
            // ROS_INFO("theta 90");           
            int x_1 = x - 5;
            int y_1 = y;   
            Point checkstart = Point(x_1,y_1);
            int x_2 = x + 5;
            int y_2 = y;   
            Point checkend = Point(x_2,y_2);

            Point length = checkend - checkstart;
            int step = ceil(max(fabs(length.x),fabs(length.y)));
            Point gap = length/step;

            for(int j = 0; j < step ; j++)
            {
                // ROS_INFO("j = %d",j);
                int checkx = round(checkstart.x);
                int checky = round(checkstart.y);
                  
                if(!onImage(checkx, checky))
                {
                    checkstart = checkstart + gap;
                    continue;
                } 
                
                int b = image_Enhance.at<Vec3b>(checky, checkx)[0];
                int g = image_Enhance.at<Vec3b>(checky, checkx)[1];
                int r = image_Enhance.at<Vec3b>(checky, checkx)[2];
                int edge_ = canny.at<uchar>(checky, checkx);
                // ROS_INFO("edge = %d",edge_);
                double diff = 0.0;
                if(edge_ == 255)
                {               
                    Vedge++;
                }
                if(g >= 245 && b >=245 && r >= 245)
                {
                    Vwhite++;
                }
                else if(g >= 245 && b <=10 && r <= 10)
                {
                    Coordinate checkgreen = {checkx,checky};
                    Coordinate checkpoint = {(int)x,(int)y};
                    // ROS_INFO("checkgreen = %d %d",checkgreen.X,checkgreen.Y);
                    // ROS_INFO("checkpoint = %d %d",checkpoint.X,checkpoint.Y);
                    diff = calculate_3D(checkgreen,checkpoint);
                    // ROS_INFO("diff = %f",diff);
                    if(diff <= 35)
                    {
                        Vgreen++;
                    }
                }else{
                    Verror++;
                }
                checkstart = checkstart + gap;
            }
        }else{
            // ROS_INFO("theta else");
            float para_a_inv = (-1.0/para_a);
            float para_b_inv = y - (para_a_inv * x);
            // ROS_INFO("line %d %d %d %d, theta = %f,para = (%f,%f) ",x1,y1,x2,y2,theta,para_a,para_b);
            // ROS_INFO("para_inv = (%f,%f)",para_a_inv,para_a_inv);
            int x_1 = x - 5;
            int y_1 = round((x_1 * para_a_inv) + para_b_inv);   
            Point checkstart = Point(x_1,y_1);
            int x_2 = x + 5;
            int y_2 = round((x_2 * para_a_inv) + para_b_inv);   
            Point checkend = Point(x_2,y_2);
            // ROS_INFO("x_1 y_1= (%d,%d)",x_1,y_1);
            // ROS_INFO("x_2 y_2= (%d,%d)",x_2,y_2);
            
            Point length = checkend - checkstart;
            int step = ceil(max(fabs(length.x),fabs(length.y)));
            Point gap = length/step;

            // ROS_INFO("length = %d %d",length.x,length.y);
            // ROS_INFO("step = %d",step);
            // ROS_INFO("gap = %d %d",gap.x,gap.y);

            for(int j = 0; j < step ; j++)
            {
                // ROS_INFO("j = %d",j);
                int checkx = round(checkstart.x);
                int checky = round(checkstart.y);
                // ROS_INFO("checkx checky= (%d,%d)",checkx,checky);
                if(!onImage(checkx, checky))
                {
                    checkstart = checkstart + gap;
                    continue;
                } 

                int b = image_Enhance.at<Vec3b>(checky, checkx)[0];
                int g = image_Enhance.at<Vec3b>(checky, checkx)[1];
                int r = image_Enhance.at<Vec3b>(checky, checkx)[2];
                int edge_ = canny.at<uchar>(checky, checkx);
                // ROS_INFO("edge = %d",edge_);
                double diff = 0.0;
                if(edge_ == 255)
                {               
                    Vedge++;
                }
                if(g >= 245 && b >=245 && r >= 245)
                {
                    Vwhite++;
                }
                else if(g >= 245 && b <=10 && r <= 10)
                {
                    Coordinate checkgreen = {checkx,checky};
                    Coordinate checkpoint = {x,y};
                    // ROS_INFO("checkgreen = %d %d",checkgreen.X,checkgreen.Y);
                    // ROS_INFO("checkpoint = %d %d",checkpoint.X,checkpoint.Y);
                    diff = calculate_3D(checkgreen,checkpoint);
                    // ROS_INFO("diff = %f",diff);
                    if(diff <= 20)
                    {
                        Vgreen++;
                    }
                }else{
                    Verror++;
                }
                checkstart = checkstart + gap;
            }
            
        }
        p1 = p1 + (Linegap * counter);
        countfor ++;
    }
    // ROS_INFO("Vgreen = %d, Vwhite = %d,Vedge = %d",Vgreen,Vwhite,Vedge);
    float avg_G = (float)Vgreen/(float)(countfor);
    float avg_W = (float)Vwhite/(float)(countfor);
    float avg_E = (float)Vedge/(float)(countfor);
    ROS_INFO("avg_G = %f, avg_W = %f,avg_E = %f,countfor = %d",avg_G,avg_W,avg_E,countfor);
    if(avg_G >= 2.0 && avg_W >= 1.0 && avg_E >=0.1) return 1;
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
    all_lines.clear();
    all_lines1.clear();
    tmp.clear();
    reduce_similar_lines.clear();
    merge_similar_lines.clear();
    check_lines.clear();
    hough_frame = ori_frame.clone();
    merge_hough_frame = ori_frame.clone();
    hough_threshold = Model_Base->houghrange->hough_threshold;
    hough_minLineLength = Model_Base->houghrange->hough_minLineLength;
    hough_maxLineGap = Model_Base->houghrange->hough_maxLineGap;
    // ROS_INFO("hough_threshold = %d hough_minLineLength = %d hough_maxLineGap = %d",hough_threshold,hough_minLineLength,hough_maxLineGap);
    HoughLinesP(canny_iframe,all_lines,1,CV_PI/180,hough_threshold,hough_minLineLength,hough_maxLineGap); 
    all_lines1 = all_lines;
    Mat EnhanceImage = iframe.clone();
    int all_lines_Size = all_lines.size();
    // ROS_INFO("all_lines_Size = %d",all_lines_Size);
    // ROS_INFO("/---------------start-------------/");
    merge_similar_lines = all_lines;

    for(size_t i = 0 ; i < all_lines.size(); i++)
    {
        // ROS_INFO("all_lines %d",i);
        Vec4i X = all_lines[i];
        // ROS_INFO("all_line(%d)=(x1 = %d ,y1 =%d, x2 =%d ,y2 =%d)  slope = %f",i,X[0],X[1],X[2],X[3],Slope(X));
        merge_similar_lines = complement(merge_similar_lines,X);
        int merge_similar_lines_SIZE = merge_similar_lines.size();
        int check = 0;
        int checklineX = checkline(EnhanceImage,canny_iframe,X);
        // ROS_INFO("all_line(%d) checkline X = %d",i,checklineX);

        if(all_lines.size() == 1)
        {
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
                int checklineY = checkline(EnhanceImage,canny_iframe,Y);
                // ROS_INFO("merge_line(%d) checkline Y = %d",j,checklineY);
                if(checklineY == 0)
                {
                    merge_similar_lines = complement(merge_similar_lines,Y);
                    all_lines = complement(all_lines,Y);
                    all_lines_Size -= 2;
                    j--;
                }else{
                    // ROS_INFO("merge_similar_lines(%d)=(x1 = %d ,y1 =%d, x2 =%d ,y2 =%d)  slope = %f",j,Y[0],Y[1],Y[2],Y[3],Slope(Y));
                    ROS_INFO("MinDistance = %f",MinDistance(X,Y));
                    ROS_INFO("AngleDiff = %f",AngleDiff(X,Y));
                    if((MinDistance(X,Y) < 35.0 && AngleDiff(X,Y) < 1.5 && stereo_flag != false && LineorNot(Y) == 1) || (MinDistance(X,Y) < 50.0 && AngleDiff(X,Y) < 1.5 && stereo_flag == false && LineorNot(Y) == 1))
                    {
                        // ROS_INFO("--------Merge--------");
                        // ROS_INFO("stereo_flag = %d",stereo_flag);
                        Merge(X,Y);
                        int checklinenum = checkline(EnhanceImage,canny_iframe,NewLine);
                        // ROS_INFO("NewLine checkline = %d",checklinenum);
                        if(checklinenum == 1)
                        {
                            check = 1; 
                            if(AngleDiff(X,NewLine) < 1.0 || AngleDiff(Y,NewLine) < 1.0)
                            {
                                reduce_similar_lines.push_back(NewLine);
                                merge_similar_lines = complement(merge_similar_lines,Y);
                                all_lines = complement(all_lines,Y);
                                all_lines_Size -= 2;
                                //ROS_INFO("all_lines_Size (After Merge ) '%d' = %d",j,all_lines_Size);
                                j--;
                            }else{
                                continue;
                                //ROS_INFO("Merge ERROR");
                            }
                        }else{
                            check = 0; 
                            // ROS_INFO("Not same Line");
                        } 
                                        
                    }else{   
                        // ROS_INFO("all_line(%d) checkline X = %d",i,checklineX);
                        //ROS_INFO("all_lines_Size (NO Merge ) '%d' = %d",j,all_lines_Size);
                    }
                }

            }   
            //ROS_INFO("reduce_similar_lines = %d",reduce_similar_lines.size());
            
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
                            continue;
                        }
                        Vec4i doublecheck2 = {Xstart.X,Xstart.Y,Xend.X,Xend.Y};

                        MaxLine = {NewLinestart.X,NewLinestart.Y,NewLineend.X,NewLineend.Y};
                        Merge(MaxLine,doublecheck2);
                        MaxLine = NewLine;
                    }
                    // ROS_INFO("MaxLine = (x1= %d ,y1 = %d ,x2 = %d ,y2 = %d),Slope = %f",MaxLine[0],MaxLine[1],MaxLine[2],MaxLine[3],Slope(MaxLine));
                    int checkMaxLine = checkline(EnhanceImage,canny_iframe,NewLine);
                    if(checkMaxLine == 1)
                    {
                        all_lines.push_back(MaxLine);
                    } 
                }
                else if(reduce_similar_lines.size() == 1)
                {
                    //ROS_INFO("push_back(reduce_similar_lines[0])");
                    // ROS_INFO("reduce_similar_lines = (x1= %d ,y1 = %d ,x2 = %d ,y2 = %d)",reduce_similar_lines[0][0],reduce_similar_lines[0][1],reduce_similar_lines[0][2],reduce_similar_lines[0][3]);
                    all_lines.push_back(reduce_similar_lines[0]);
                }else if(merge_similar_lines.size() == merge_similar_lines_SIZE){
                    if(checklineX == 1)
                    {
                        check_lines.push_back(X);
                    }
                    // ROS_INFO("X = (x1= %d ,y1 = %d ,x2 = %d ,y2 = %d)",X[0],X[1],X[2],X[3]);
                }
                reduce_similar_lines.clear();
            }else if(merge_similar_lines.size() == 0 && reduce_similar_lines.size() == 1)
            {
                // ROS_INFO("reduce_similar_lines = (x1= %d ,y1 = %d ,x2 = %d ,y2 = %d)",reduce_similar_lines[0][0],reduce_similar_lines[0][1],reduce_similar_lines[0][2],reduce_similar_lines[0][3]);
                all_lines.push_back(reduce_similar_lines[0]);
            }else if(merge_similar_lines.size() == merge_similar_lines_SIZE && checklineX != 0 && all_lines.size() != 1){
                // ROS_INFO("X = (x1= %d ,y1 = %d ,x2 = %d ,y2 = %d)",X[0],X[1],X[2],X[3]);
                check_lines.push_back(X);
            }
            reduce_similar_lines.clear();    
            // all_lines_Size ++;
        }
    }
    // ROS_INFO("check_lines.size = %d",check_lines.size());

    for(size_t i = 0 ; i < all_lines1.size(); i++)
    {
        Vec4i X = all_lines1[i];
        line( hough_frame, Point(X[0], X[1]), Point(X[2], X[3]), Scalar(255,0,0), 2, CV_AA);
        // ROS_INFO("---------all_line(%d)=(x1 = %d ,y1 =%d, x2 =%d ,y2 =%d)  slope = %f",i,X[0],X[1],X[2],X[3],Slope(X));
    }
    // sort(check_lines.begin(), check_lines.end(), tocompare);
    JustLine_Data.landmark.clear();
    for( size_t i = 0; i < check_lines.size(); i++ )
	{
        Vec4i l = check_lines[i];
        // ROS_INFO("--------check_lines(%d)=(x1 = %d ,y1 =%d, x2 =%d ,y2 =%d)  slope = %f",i,l[0],l[1],l[2],l[3],Slope(l)); 
        // ROS_INFO("check_lines %d",i);
        if(LineorNot(l) == 0) 
        {
            check_lines = complement(check_lines,l);
            continue;
        }
        
        // ROS_INFO("check_lines = (x1= %d ,y1 = %d ,x2 = %d ,y2 = %d)",l[0],l[1],l[2],l[3]);
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
        Coordinate pointP = {319,479};
        Coordinate midpoint = Midpoint(l);
        Distance start_ = measure(startp.X,startp.Y,CameraType::stereo);
        Distance end_ = measure(endp.X,endp.Y,CameraType::stereo);
        int dis =  int(round(sqrt(pow((start_.x_dis - end_.x_dis),2) + pow((start_.y_dis - end_.y_dis),2))));
        
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
        double mindis = disMin(startp,endp,pointP,CameraType::Monocular);//點P到線段AB的最短距離
        Point minIntersectPoint = MinIntersectPoint(l,Point(319,478),mindis);
        // circle(merge_hough_frame, Point(minIntersectPoint.x, minIntersectPoint.y), 3, Scalar(0, 255, 255), 6);
        Distance relativedis = measure(minIntersectPoint.x,minIntersectPoint.y,CameraType::stereo);
        tku_msgs::Cooridinate IntersectPoint;
        IntersectPoint.x = minIntersectPoint.x;
        IntersectPoint.y = minIntersectPoint.y;
        line_tmp.start_point = startpoint;
        line_tmp.end_point = endpoint;
        line_tmp.center_point = mid_point;
        line_tmp.Line_length = dis;
        line_tmp.Line_theta = normalize_angle_RAD(Slope(l));
        line_tmp.relative_distance = float(relativedis.dis)/100.0;
        line_tmp.Nearest_point = IntersectPoint;
        // line_tmp.Line_theta = acos((AdotB)/((image_bottom_width_length/2)*(middlepoint.dis)))*180/CV_PI;
        // ROS_INFO("Line %d : %d %d %d %d (x,y) = (%d ,%d) , distance =  %d", i,l[0],l[1],l[2],l[3],minIntersectPoint.x,minIntersectPoint.y,relativedis.dis);
        JustLine_Data.landmark.push_back(line_tmp);
 
        line( merge_hough_frame, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 2, CV_AA);
        circle(merge_hough_frame, Point(midpoint.X, midpoint.Y), 1, Scalar(0, 255, 0), -1);
    }
    // imshow("hough_frame",hough_frame);
    // imshow("merge_hough_frame",merge_hough_frame);
    return merge_hough_frame;
}

Mat LineDetected::UpperConvexHull(const Mat ori,Mat drawing,vector<vector<Point> > allfieldpoints,int horization_line)
{
    vector<Point> Allgreenhull;

    for(int i=0; i < allfieldpoints.size(); i++)
    {
        Allgreenhull.insert(Allgreenhull.end(), allfieldpoints[i].begin(), allfieldpoints[i].end());
    }
  
    vector<Point> upperCH;
    vector<Point> points = Allgreenhull;
    int n_points = points.size();
    vector<vector<Point> > contours(1);

    sort(points.begin(), points.end(), sortPoints);
 
    //Computing upper convex hull
    upperCH.push_back(points[n_points-1]);
    upperCH.push_back(points[n_points-2]);
    // ROS_INFO("000000000");
    for(int i=2; i< n_points; i++)
    {
        while(upperCH.size() > 1 and (!right_turn(upperCH[upperCH.size()-2],upperCH[upperCH.size()-1], points[n_points-i-1])))
            upperCH.pop_back();
        upperCH.push_back(points[n_points-i-1]);
    }

    // 畫出upper convex hull
    Mat mask = Mat::zeros(orign.rows,orign.cols, CV_8UC1); 
    for ( size_t i = 0; i < upperCH.size(); i++ )
    {
        circle(mask,upperCH[i],3,Scalar(255,255,255),CV_FILLED,1);	
    }

    Mat mask1 = Mat::zeros(orign.rows,orign.cols, CV_8UC3); 
    
    convexHull(Mat(upperCH), contours[0], false );
    double area = contourArea(contours[0]);

    if(area < 155000.0)
    {
        for(int i = 0 ; i < orign.cols; i ++)
        {
            for(int j = 0; j < orign.rows; j++)
            {
                if(j >= horization_line)
                {
                    drawing.at<uchar>(j, i) = 255;
                }
            }
        }
    }else{
        for ( size_t i = 0; i < contours.size(); i++ )
        {
            drawContours( mask1, contours, i, Scalar(255,255,255), -1);
        }
        // imshow("mask1",mask1);

        vector<Point> fieldboundary;
        for(int i = 0; i < mask1.cols;i++)
        {
            Point A;
            for(int k = 0 ; k < mask1.rows ;k++)
            {
                int a = mask1.at<Vec3b>(k, i)[0];
                int b = mask1.at<Vec3b>(k, i)[1];
                int c = mask1.at<Vec3b>(k, i)[2];
                if( a == 255 && b == 255 && c ==255 && k < mask1.rows -5 )
                {
                    A = Point(i,k);
                    break;
                }
            }
            fieldboundary.push_back(A);
        }


        for(int i = 0; i < fieldboundary.size();i++)
        {
            Point B = fieldboundary[i];
            for(int k = 0 ; k < drawing.rows ;k++)
            {
                if( k >= B.y)
                {
                    drawing.at<uchar>(k, B.x) = 255;
                }else{
                    drawing.at<uchar>(k, B.x) = 0;
                }
            }
        }
        fieldboundary.clear();
    }

    upperCH.clear();
    contours.clear();
    points.clear();
    Allgreenhull.clear();
    // imshow("drawing",drawing);
    
    return drawing;
}

