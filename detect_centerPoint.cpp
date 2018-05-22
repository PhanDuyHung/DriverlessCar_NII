//
//  detect_centerPoint.cpp
//  test_
//
//  Created by Macbook Pro on 08/03/2017.
//  Copyright © 2017 Macbook Pro. All rights reserved.
//

#include "detect_centerPoint.hpp"



void edgeProcessing(Mat src, Mat &dst, Rect roi)
{
    Mat imgGray, imgClone, imgClone_gray;
    cvtColor(src, imgGray, CV_BGR2GRAY);
    //cvtColor(imgClone, imgClone_gray, CV_BGR2GRAY);
    imgClone_gray = imgGray;
    Mat imgThresh;
    //adaptiveThreshold(imgClone_gray, imgThresh, 255, CV_ADAPTIVE_THRESH_GAUSSIAN_C, 1, 11, 9);
    double otsu_thres = threshold(imgClone_gray, imgThresh, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
    double highThresh = otsu_thres;
    //	threshold(grayImage1, grayImage, highThresh*0.2, 255, CV_THRESH_BINARY);
    cv::Canny(imgGray, imgThresh, highThresh*0.3, highThresh, 3);
   // imshow("Thresh", imgThresh);
    Mat img_bin = imgThresh(roi).clone();
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    
    ////CvMemStorage *storage = cvCreateMemStorage(0);
    ////CvSeq *seq_contour = cvCreateSeq(CV_SEQ_ELTYPE_POINT, sizeof(CvSeq), sizeof(CvPoint), storage);
    cv::findContours(img_bin, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
    /// Draw contours
    int xmax, ymax, xmin, ymin;
    int xleft, xright, ytop, ybottom;
    Mat drawing = Mat::zeros(src(roi).size(), CV_8UC3);
    int width = drawing.cols;
    int height = drawing.rows;
    int x=20,y=20;
    int maxhL=0,imaxL=-1,maxhR=0,imaxR=-1;
    
    for (int i = 0; i< contours.size(); i++)
    {
        int count = contours[i].size();
        if (count>100)
        {
            for (int t = 0; t < count; t++)
            {
                if (t == 0){
                    xleft = xright = contours[i][t].x;
                    ytop = ybottom = contours[i][t].y;
                }
                if (contours[i][t].x > xright){
                    xright = contours[i][t].x;
                }
                if (contours[i][t].x < xleft)
                {
                    xleft = contours[i][t].x;
                }
                if (contours[i][t].y > ytop)
                {
                    ytop = contours[i][t].y;
                }
                if (contours[i][t].y < ybottom)
                {
                    ybottom = contours[i][t].y;
                }
                xmin = xleft; xmax = xright;
                ymin = ybottom; ymax = ytop;
            }
            int w = xmax - xmin, h = ymax - ymin;
            int s = w*h;
            if (((xmax + xmin) / 2)>width / 10 && ((xmax + xmin) / 2) < width *2/ 5 && (ymax + ymin) / 2<height * 3 / 4 && (ymax + ymin) / 2>height / 4 && s>height&&h>height/4)
                //if ()
            {
                if (h > maxhL)
                {
                    maxhL = h; imaxL = i;
                }
            }
            if (((xmax + xmin) / 2)>width / 2 && ((xmax + xmin) / 2) < width  && (ymax + ymin) / 2<height * 3 / 4 && (ymax + ymin) / 2>height / 4 && s>height&&h>height / 4)
                //if ()
            {
                if (h > maxhR)
                {
                    maxhR = h; imaxR = i;
                }
            }
        }
    }
    Scalar color = Scalar(0, 0, 255);
    if(imaxL!=-1)drawContours(drawing, contours, imaxL, color, 2, 8, hierarchy, 3, Point());
    if (imaxR != -1)drawContours(drawing, contours, imaxR, color, 2, 8, hierarchy, 3, Point());
    
    /// Show in a window
   // namedWindow("Contours", CV_WINDOW_AUTOSIZE);
    imshow("Contours", drawing);
    
    Mat img_bin_gray;
    cvtColor(drawing, img_bin_gray, CV_BGR2GRAY);
    threshold(img_bin_gray, img_bin_gray, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
    
    
    int erosion_size = 5;
    cv::Mat element = cv::getStructuringElement(MORPH_RECT,
                                                Size(2 * erosion_size + 1, 2 * erosion_size + 1),
                                                Point(erosion_size, erosion_size));
    dilate(img_bin_gray, img_bin_gray, element);
    erode(img_bin_gray, img_bin_gray, element);
    //imshow("thres2", img_bin_gray);
    dst = img_bin_gray;
}
float range(Point a, Point b)
{
    return sqrt((a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y));
}
void detect_centerP(Mat src, Point &centerP,Point &centerL,Point &centerR, Rect roi, Mat &dst, int &t,int &l,int &r){
    Mat img_edge;
    edgeProcessing(src, img_edge, roi);
    dst = img_edge;
    vector<cv::Point> auxL,auxR;
    for (int i = 0; i < img_edge.rows; i++)
    {
        int j = 1;
        while (dst.at<uchar>(i, j) != 0 && dst.at<uchar>(i, j - 1) == 255&&j<dst.cols/2)
        {
            j++;
        }
        while (dst.at<uchar>(i, j) == 0 && dst.at<uchar>(i, j - 1) != 255&&j<dst.cols/2)
        {
            j++;
        }
        if (j!=dst.cols/2)
        {
            Point p(j, i);
            auxL.push_back(p);
        }
        j = dst.cols-1;
        while (dst.at<uchar>(i, j) != 0 && dst.at<uchar>(i, j - 1) == 255 && j>dst.cols / 2)
        {
            j--;
        }
        
        while (dst.at<uchar>(i, j) == 0 && dst.at<uchar>(i, j - 1) != 255 && j>dst.cols / 2)
        {
            j--;
        }
        if (j != dst.cols / 2)
        {
            Point p(j, i);
            auxR.push_back(p);
        }
        
    }
    Point center_L(0, src.rows * 7 / 8);
    Point center_R(0, src.rows * 7 / 8);
    if (auxL.size() > 0)
    {
        l=1;
        for (int i = 0; i < auxL.size(); i++)
        {
            center_L.x += auxL[i].x + roi.x;
            //		center_L.y += auxL[i].y + roi.y;
        }
        center_L.x = center_L.x / auxL.size();
        cv::circle(src, center_L, 4, cv::Scalar(130, 255, 20), 3);
    }
    if (auxR.size() > 0)
    {
        r=1;
        for (int i = 0; i < auxR.size(); i++)
        {
            center_R.x += auxR[i].x + roi.x;
            //	center_L.y += auxR[i].y + roi.y;
        }
        center_R.x = center_R.x / auxR.size();
        cv::circle(src, center_R, 4, cv::Scalar(130, 255, 20), 3);
    }
    if(l==1&&r==1) {
        centerR.x=center_R.x;
        centerL.x=center_L.x;
        centerP.x=(centerL.x+centerR.x)/2;
       
    }
    else if(l==1){
        centerL.x=center_L.x;
        centerP.x=(centerL.x+centerR.x)/2;
    }
    else if(r==1){
        centerR.x=center_R.x;
        centerP.x=(centerL.x+centerR.x)/2;
    }
    //Point midP(src.cols / 2, src.rows),leftP;
    //int houghThreshold = 100;
    //vector<cv::Vec4i> lines;
    //cv::HoughLinesP(img_edge, lines, 2, CV_PI / 90, houghThreshold, 10, 30);
    //if (lines.size() > 0)
    //{
    //	cv::Point pt1, pt2,pt_mid,centerL;
    //	pt1.x = lines[0][0]+roi.x;
    //	pt1.y = lines[0][1]+roi.y;
    //	pt2.x = lines[0][2]+roi.x;
    //	pt2.y = lines[0][3]+roi.y;
    //	cv::line(src, pt1, pt2, cv::Scalar(0, 0, 255, 0), 2);
    //	//pt dt (y2-y1)(x-x1)=(x2-x1)(y-y1) hay a(x-x1)=b(y-y1)
    //	int a1, a2, b1, b2;
    //	a1 = pt2.y - pt1.y; b1 = pt2.x - pt1.x;
    //	leftP.y = src.rows;
    //	leftP.x = ((float)b1 / a1)*leftP.y + ((float)a1*pt1.x - (float)b1*pt1.y) / a1;
    //	centerL.y = src.rows * 7 / 8;
    //	centerL.x = ((float)b1 / a1)*centerL.y + ((float)a1*pt1.x - (float)b1*pt1.y) / a1;
    //	midP.x = leftP.x + src.cols/4;
    //	if (t == 1)
    //	{
    //		centerP.x = midP.x;
    //		t = 0;
    //	}
    //	a2 = midP.y - centerP.y; b2 = midP.x - centerP.x;
    //	if (a1 != 0 && a2 != 0 && b1 != 0 && b2 != 0)
    //	{
    //		pt_mid.x = (((float)b2*centerP.y - (float)a2*centerP.x) / b2 - ((float)b1*pt1.y - (float)a1*pt1.x) / b1) / ((float)a1 / b1 - (float)a2 / b2);
    //		pt_mid.y = ((float)a1 / b1)*pt_mid.x + ((float)b1*pt1.y - (float)a1*pt1.x) / b1;
    //	}
    //	
    //	float range_P = (range(leftP, midP)*range(pt_mid, centerL)) / range(pt_mid, leftP);
    //	centerP.x = centerL.x + (int)range_P;
    //	cv::circle(src, pt1, 4, cv::Scalar(140, 255, 255), 3);
    //	cv::circle(src, pt2, 4, cv::Scalar(140, 255, 255), 3);
    //	cv::circle(src, midP, 4, cv::Scalar(140, 255, 255), 3);
    //	cv::circle(src, leftP, 4, cv::Scalar(100, 0, 255), 3);
    //	cv::circle(src, pt_mid, 4, cv::Scalar(0, 255, 0), 3);
    	cv::imshow("circle", src);
    //}
}