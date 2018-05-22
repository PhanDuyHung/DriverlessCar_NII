//
//  detect_centerPoint.hpp
//  test_
//
//  Created by Macbook Pro on 08/03/2017.
//  Copyright © 2017 Macbook Pro. All rights reserved.
//

#ifndef detect_centerPoint_hpp
#define detect_centerPoint_hpp

#include <stdio.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/ml/ml.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

void edgeProcessing(Mat src, Mat &dst, Rect roi);
void detect_centerP(Mat src, Point &centerP,Point &centerL,Point &centerR, Rect roi, Mat &dst, int &t,int &l,int &r);
    //    void LoadVideo(std::string s);
    //    Mat ImageProcess(cv::Mat src);
    //    void Convert(cv::Mat& img, std::vector<cv::Mat>& cmyk);
    //    void Sample(cv::Mat src);


#endif /* detect_centerPoint_hpp */
