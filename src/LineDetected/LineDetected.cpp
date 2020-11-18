#include <LineDetected/LineDetected.h>
#include <algorithm>
LineDetected::LineDetected()
{
    ErRoR = 65;
    mX = 0;
    mY = 0;
    temp_max = 0; 
    i_max = 0; 
    temp_min = 0; 
    i_min = 0;

}
LineDetected::~LineDetected()
{
    
}

Mat LineDetected::ImagePreprocessing(const Mat iframe)
{
    blur(iframe,iframe,Size(3,3));
    //濾除非場地部份
    Mat mask = Mat::zeros(iframe.rows,iframe.cols, CV_8UC3); 
    Mat Kernel = (Mat_<float>(3, 3) << 0, -1, 0, -1, 6.9, -1, 0, -1, 0);
    Mat imageEnhance;
    filter2D(iframe, imageEnhance, CV_8UC3, Kernel);

    Mat imageGamma = Mat::zeros(imageEnhance.rows,imageEnhance.cols, CV_32FC3); 
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
    //namedWindow("imageGamma",WINDOW_NORMAL);
    //imshow("imageGamma",imageGamma);

    int R_value = 150;
    int G_value = 220;
    int B_value = 220;
    Mat nobackgroud_image;

    if(Vertical_Head_Angle > 50.0)
    {
        for(int col = 0; col < imageGamma.cols; col++)
        {
            for(int row = imageGamma.rows - 1; row > 0; row--)
            {
                int b = imageGamma.at<Vec3b>(row, col)[0];
                int g = imageGamma.at<Vec3b>(row, col)[1];
                int r = imageGamma.at<Vec3b>(row, col)[2];
                if( g >= G_value) //0:Y452X61
                {
                    /*if((b > 230 && g > 230 && r < 150) || (b == 255 && g == 255 && r == 255))
                    {
                        int b2 = imageGamma.at<Vec3b>(row - 3, col)[0];
                        int g2 = imageGamma.at<Vec3b>(row - 3, col)[1];
                        int r2 = imageGamma.at<Vec3b>(row - 3, col)[2];
                        if(b2 > 230 && g2 > 230 && r2 < 150 || (b2 == 255 && g2 == 255 && r2 == 255))
                        {
                            int b3 = imageGamma.at<Vec3b>(row - 9, col)[0];
                            int g3 = imageGamma.at<Vec3b>(row - 9, col)[1];
                            int r3 = imageGamma.at<Vec3b>(row - 9, col)[2];
                            if(b3 > 230 && g3 > 230 && r3 < 150 || (b3 == 255 && g3 == 255 && r3 == 255))
                            {
                                break;
                            }
                        }
                    }*/
                    mask.at<Vec3b>(row, col)[0] = 255;
                    mask.at<Vec3b>(row, col)[1] = 255;
                    mask.at<Vec3b>(row, col)[2] = 255;
                }
                else
                {
                    break;
                } 
            }
        }

        //Mat mask_element = getStructuringElement(MORPH_RECT, Size(3, 3)); 
        //morphologyEx(mask, mask, CV_MOP_OPEN, mask_element); 
        imageGamma.copyTo(nobackgroud_image,mask);
    }
    else
    {
        nobackgroud_image = imageGamma.clone();
    }
    namedWindow("nobackgroud_image",WINDOW_NORMAL);
    imshow("nobackgroud_image",nobackgroud_image);
  
    for(int row = 0; row < nobackgroud_image.rows;row++)
    {
        for(int col = 0; col < nobackgroud_image.cols;col++)
        {
            int b = nobackgroud_image.at<Vec3b>(row, col)[0];
            int g = nobackgroud_image.at<Vec3b>(row, col)[1];
            int r = nobackgroud_image.at<Vec3b>(row, col)[2];
            if(b >= B_value && g >= G_value && r <= R_value)
            {
                nobackgroud_image.at<Vec3b>(row, col)[0] = 255;
                nobackgroud_image.at<Vec3b>(row, col)[1] = 255;
                nobackgroud_image.at<Vec3b>(row, col)[2] = 255;
            }
            else
            {
                nobackgroud_image.at<Vec3b>(row, col)[0] = 0;
                nobackgroud_image.at<Vec3b>(row, col)[1] = 0;
                nobackgroud_image.at<Vec3b>(row, col)[2] = 0;
            }
        }
    }
    Mat element = getStructuringElement(MORPH_RECT, Size(15, 15));
    Mat element_2 = getStructuringElement(MORPH_RECT, Size(5, 5));
    dilate(nobackgroud_image,element,element);
    erode(nobackgroud_image,element,element_2);

    element = getStructuringElement(MORPH_RECT, Size(2,2));
    element_2 = getStructuringElement(MORPH_RECT, Size(15,15));
    morphologyEx(nobackgroud_image, nobackgroud_image, CV_MOP_CLOSE, element_2);
    morphologyEx(nobackgroud_image, nobackgroud_image, CV_MOP_OPEN, element);
    
    Mat edge;
    Canny(nobackgroud_image, edge, 50, 150, 3);
    
    return  edge;
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
    else return atan2((double(line[3])-double(line[1])),(double(line[2])-double(line[0])))*(180.0/CV_PI);
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

Mat LineDetected::Merge_similar_line(const Mat canny_iframe,const Mat original_frame)
{
    tmp.clear();
    all_lines.clear();
    reduce_similar_lines.clear();
    HoughLinesP(canny_iframe,all_lines,1,CV_PI/180,110,50,30); 
    Mat original_frame1=original_frame.clone();
    int all_lines_Size = all_lines.size();
    merge_similar_lines = all_lines;
    
    ROS_INFO("all_lines_Size = %d",all_lines_Size);
    ROS_INFO("/---------------start-------------/");
    for(size_t i = 0 ; i < all_lines.size(); i++)
    {
        //ROS_INFO("/---------------first[%d]-------------/",i+1);
        Vec4i X = all_lines[i];
        //line( original_frame1, Point(X[0], X[1]), Point(X[2], X[3]), Scalar(255,0,0), 2, CV_AA);
        merge_similar_lines = complement(merge_similar_lines,X);
        all_lines_Size--;
        //ROS_INFO("merge_similar_lines = %d",merge_similar_lines.size());
        for(size_t j = 0 ; j < merge_similar_lines.size(); j++)
        {
            //ROS_INFO("/---------------second[%d]-------------/",j+1);
            Vec4i Y = merge_similar_lines[j];
            //ROS_INFO("MinDistance = %f",MinDistance(X,Y));
            //ROS_INFO("AngleDiff = %f",AngleDiff(X,Y));
            if(MinDistance(X,Y) < 30 && AngleDiff(X,Y) < 1)
            {
                //ROS_INFO("stage 2");
                Merge(X,Y);
                reduce_similar_lines.push_back(NewLine);
                merge_similar_lines = complement(merge_similar_lines,Y);
                all_lines_Size--;
            }
        }
        if(merge_similar_lines.size() == all_lines_Size) reduce_similar_lines.push_back(X);
    }
    ROS_INFO("reduce_similar_lines.size(2) = %d",reduce_similar_lines.size());

    //int reduce_similar_lines_size = reduce_similar_lines.size();
    /*for(size_t i = 0 ; i < reduce_similar_lines.size(); i++)
    {
        Vec4i R = reduce_similar_lines[i];
        reduce_similar_lines = complement(reduce_similar_lines,R);
        int reduce_similar_lines_SIZE = reduce_similar_lines.size();
        for(size_t j = 0 ; j < reduce_similar_lines.size(); j++)
        {
            Vec4i N = reduce_similar_lines[j];
            if(MinDistance(R,N) < 5 && AngleDiff(R,N) < 1)
            {
                ROS_INFO("2222222222222");
                Merge(R,N);
                tmp.push_back(NewLine);
                reduce_similar_lines = complement(reduce_similar_lines,N);
                reduce_similar_lines_SIZE --;
            }
        }
        if(reduce_similar_lines.size() == reduce_similar_lines_SIZE )
        {
            ROS_INFO("555555");
            tmp.push_back(R);
        }
    }*/

    ROS_INFO("/---------------finish-------------/");
    ROS_INFO("reduce_similar_lines.size(2) = %d",reduce_similar_lines.size());
    for( size_t i = 0; i < reduce_similar_lines.size(); i++ )
	{
        Vec4i l = reduce_similar_lines[i];   
	    line( original_frame, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255,255,0), 3, CV_AA);
    }
    //imshow("original_frame1",original_frame1);
    //imshow("original_frame",original_frame);
    
    return original_frame;
}

int LineDetected::getHistograph(const Mat grayImage)
{
    int channels[]={0};
	const int histSize[]={256};
	float range[]={0,256};
	const float* ranges[]={range};
	calcHist(&grayImage,1,channels,Mat(),hist,1,histSize,ranges,true,false);
	double maxValue=0;
	minMaxLoc(hist,0,&maxValue,0,0);
	int rows=cvRound(maxValue);
	for(int i=0;i<256;i++)
	{
		int temp=(int)(hist.at<float>(i,0));
        if (i>=10&&temp_max<=temp)
        {
            temp_max=temp;
            i_min = i;
        }
        if (temp > 0)
        {
            temp_min=temp;
            i_max = i;
        }
	}
    return (i_max-i_min)*0.5+i_min;	
}
