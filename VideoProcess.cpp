//
//  VideoProcess.cpp
//  test_
//
//  Created by Macbook Pro on 06/03/2017.
//  Copyright © 2017 Macbook Pro. All rights reserved.
//

#include "VideoProcess.hpp"
#include "convertColor.hpp"
#include "detect_centerPoint.hpp"
void VideoProcess::LoadVideo(string s)
{
  //  CvCapture *cap = cvCreateFileCapture("guitarplaying.avi");
    cv::VideoCapture cap("/Users/macbookpro/Desktop/3.mp4"); // Mở các file video đọc
    if (!cap.isOpened()) //Nếu ko mở được thoát khỏi chương trình
    {
        std::cout << "Không thể mở file video" << std::endl;
    }
    double fps = cap.get(CV_CAP_PROP_FPS); //Lấy các khung hình mỗi giây của video
    std::string f = "Frame / seconds : ";
    //
    //
    std::cout << "Frame / seconds : " << fps << std::endl;
    //cv::namedWindow("MyVideo", CV_WINDOW_AUTOSIZE); //Tạo ra một cửa số "MyVideo"
    int video_frame_width = cap.get(CV_CAP_PROP_FRAME_WIDTH);
    int video_frame_height = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
    int x = video_frame_width;
    int y = video_frame_height;
    Size output_size(video_frame_width, video_frame_height);
    cv::Rect roi = cv::Rect(0, video_frame_height * 3 / 4,
                            video_frame_width,video_frame_height / 4);
    
    Mat depthImg, colorImg, grayImage, grayImage1;
    Point carPosition(video_frame_width / 2, video_frame_height * 7 / 8);
    /**
     We chose car position is center of bottom of the image.
     Indeed, it's a rectangle. We used only a single point to make it simple and enough-to-work.
     **/
    Point prvPosition = carPosition;
//    MSAC msac;
//    api_vanishing_point_init(msac);
    cv::Point center_p1_l(x / 4, y * 19 / 24), center_p1_r(x * 3 / 4, y * 19 / 24), center_p2_l(x / 4, y * 19 / 24), center_p2_r(x * 3 / 4, y * 7 / 8), center_p3_l(x / 4, y *7 / 8), center_p3_r(x * 3 / 4, y * 23 / 24);
    
    Point center_point(video_frame_width / 2, video_frame_height * 7 / 8);
    Point centerL(video_frame_width / 4, video_frame_height * 7 / 8);
    Point centerR(video_frame_width *3/ 4, video_frame_height * 7 / 8);
    int t = 1;
    int l=0,r=0;
    convertColor I;
//    detect_centerPoint U;
    vector<cv::Mat>hsv;
    while (1)
    {
        
        cv::Mat frame;
        
        bool bSuccess = cap.read(frame); // Đọc cá khung từ video
        
        if (!bSuccess) //Kiểm tra xem có đọc được ko
        {
            std::cout<<"Khong mo duoc video";
            break;
        }//detect_Lines_me(frame, roi, grayImage);
        
//        I.cvtRGB2HSV(frame, hsv);
//        cv::imshow("h", hsv[0]);
//        cv::imshow("s", hsv[1]);
//        cv::imshow("v", hsv[2]);

        	cvtColor(frame, grayImage, CV_BGR2GRAY);
        //cv::fastNlMeansDenoisingMulti(grayImage,grayImage1,,3,7,21)
        //fastNlMeansDenoising(grayImage, grayImage1, 3, 7, 21);
        //	cvtColor(frame, grayImage1, CV_BGR2GRAY);
        //	edgeProcessing(frame, grayImage,roi);
        
//        detect_centerP(frame, center_point, roi, grayImage,t);
        //	double otsu_thres = threshold(grayImage, grayImage, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
        //	double highThresh = otsu_thres;
        //threshold(grayImage1, grayImage, highThresh*0.2, 255, CV_THRESH_BINARY);
        //	cv::Canny(grayImage1, grayImage, highThresh*0.5, highThresh, 3);
        //	adaptiveThreshold(grayImage1, grayImage, 255, CV_ADAPTIVE_THRESH_GAUSSIAN_C, 1, 3,2);
        //	int erosion_size = 1;
        //	cv::Mat element = cv::getStructuringElement(MORPH_RECT,
        //		Size(2 * erosion_size + 1, 2 * erosion_size + 1),
        //		Point(erosion_size, erosion_size));
        ////	cv::dilate(grayImage, grayImage, element, cv::Point(erosion_size, erosion_size), 3);
        
        //	cv::erode(grayImage, grayImage, element, cv::Point(erosion_size, erosion_size),2);
        
        //	get_lines_me(grayImage, roi1, center_point, center_p1_l, center_p1_r, center_p2_l, center_p2_r, center_p3_l, center_p3_r);
        //	api_get_vanishing_point(frame, roi, msac, center_point, true, "Canny");
        //	api_get_lane_center(grayImage, center_point, true);
        l=0;r=0;
        detect_centerP(frame, center_point,centerL,centerR, roi, grayImage, t,l,r);
        
        if (center_point.x == 0 && center_point.y == 0) center_point = prvPosition;
        cv::circle(frame, center_point, 4, cv::Scalar(0, 255, 255), 3);
        cv::circle(frame, centerL, 4, cv::Scalar(0, 255, 0), 3);
        cv::circle(frame, centerR, 4, cv::Scalar(0, 255, 0), 3);
        prvPosition = center_point;
        cv::imshow("gray", grayImage);
        cv::imshow("src", frame);
        //cv::imshow("gray2", grayImage);
        //cv::Mat gray=ImageProcess(frame);
        //cv::cvtColor(frame, gray, CV_BGR2GRAY);
        //cv::threshold(gray, gray, 128, 255, CV_THRESH_BINARY);
        //viết chữ trên màn hình
        /*	int fontFace = FONT_HERSHEY_SIMPLEX;
         double fontScale = 0.6;
         int thickness = 2;
         cv::Point textOrg(20, 20);*/
        //	cv::putText(gray, f, textOrg, fontFace, fontScale, Scalar(255, 255, 255), thickness,8);
        
        //
        //	cv::imshow("MyVideo", gray); //Hiển thị khung hình trong MyVideo
        
        //cvPutText(img, text, cvPoint(200, 400), &font, cvScalar(255, 255, 0));
        if (cv::waitKey(30) == 48) //chờ người dùng nhân "esc"
        {
            cv::waitKey();
        }
    }
}

