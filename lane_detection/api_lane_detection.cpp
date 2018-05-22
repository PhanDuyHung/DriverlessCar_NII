#include "api_lane_detection.h"
#define SCARLAR_LOWER_GREEN_OBS 30,170,30//41, 54, 63
#define SCARLAR_UPPER_GREEN_OBS 50,255,60//71, 255, 255

int set_offset = 210;
int offset = 210;
int offset_vip = 100;
int min_area = 500, max_area = 3500;

void setOffset(int o)
{
	offset = set_offset+o;
}

bool CenterPoint_NII(const Mat& imgColor, double& theta,int& lane)
{
	float roi_width = VIDEO_FRAME_WIDTH, roi_height = VIDEO_FRAME_HEIGHT * 0.1;
	cv::Rect roi = cv::Rect(0, VIDEO_FRAME_HEIGHT * 0.9,roi_width, roi_height);//50:0.8

	Mat imgLane = imgColor(roi).clone();
	imwrite("co.jpg",imgLane);
	Mat img;
	cvtColor(imgLane,img,CV_BGR2GRAY);
	GaussianBlur(img,img,Size(11,11),5);
	threshold(img,img,200,255,THRESH_BINARY);
	imshow("thresh",img);
	
	vector<vector<cv::Point> > contours;
	vector<cv::Vec4i> hierarchy;

	findContours(img, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
	vector<cv::Point> convex_hull;
	vector<cv::Point> contour;
	int size = contours.size();
	Point p(0,0);
	for (size_t i = 0; i < size; i++)
	{
		int area = (int)cv::contourArea(contours[i]);
		//cout <<"area"<<area<<endl;
		if (area <= min_area || area >= max_area) continue;
		
		// simplify large contours
		cv::approxPolyDP(cv::Mat(contours[i]), contour, 5, true);

		// convex hull
		cv::convexHull(contour, convex_hull, false);

		// center of gravity
		cv::Moments mo = cv::moments(convex_hull);
		p = cv::Point(mo.m10 / mo.m00, mo.m01 / mo.m00);

		if (p.x > roi_width*0.55 && lane == 1) break;
		if (p.x < roi_width*0.45 && lane == -1) break;
	}

	//imshow("lane",imgLane);
	//cout<<"p:"<<p.x<<endl;
	if (p.x == 0 || (p.x >= roi_width*0.45 && p.x <= roi_width*0.55))
	{
		Mat imgGreen;
		Mat imgLaneTemp;
		cvtColor(imgLane,imgLaneTemp,CV_BGR2HSV);
		inRange(imgLaneTemp,Scalar(SCARLAR_LOWER_GREEN_OBS),Scalar(SCARLAR_UPPER_GREEN_OBS),imgGreen);
		imshow("range",imgGreen);
		normalize(imgGreen,imgGreen,0,255,NORM_MINMAX);
		//medianBlur(imgGreen,imgGreen,5);
		//GaussianBlur(imgGreen,imgGreen,Size(5,5),0);
		threshold(imgGreen,imgGreen,127,255,CV_THRESH_BINARY);
		imshow("threshgreen",imgGreen);
		vector<vector<cv::Point> > contoursGreen;
		vector<cv::Vec4i> hierarchyGreen;

		findContours(imgGreen, contoursGreen, hierarchyGreen, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
		vector<cv::Point> convex_hull_Green;
		vector<cv::Point> contourGreen;
		int size = contoursGreen.size();
		Point q(0,0);
		for (size_t i = 0; i < size; i++)
		{
			int area = (int)cv::contourArea(contoursGreen[i]);
			//cout <<"area"<<area<<endl;
			if (area <= min_area || area >= max_area) continue;
			
			// simplify large contours
			cv::approxPolyDP(cv::Mat(contoursGreen[i]), contourGreen, 5, true);

			// convex hull
			cv::convexHull(contourGreen, convex_hull_Green, false);

			// center of gravity
			cv::Moments mo = cv::moments(convex_hull_Green);
			q = cv::Point(mo.m10 / mo.m00, mo.m01 / mo.m00);

			if (q.x > roi_width*0.55 && lane == 1) break;
			if (q.x < roi_width*0.45 && lane == -1) break;
		}

		if (q.x == 0 || (q.x >= roi_width*0.45 && q.x <= roi_width*0.55)) return false;


		circle(imgLane, Point(q.x,roi_height/2), 5, Scalar(255, 0, 0), 2);
		if (lane == 0)
		{
			lane = (q.x > roi_width*0.55)?lane = 1:lane=-1;
		}
		else
		if (q.x > roi_width * 0.55 && lane == 1)//dang phai thay phai
		{
			q.x -= (offset + 0);
			//lane = 1;//right
		} 
		else if (q.x < roi_width * 0.45 && lane == -1)//dang trai thay trai
		{
			q.x += offset;
			//lane = -1;//left
		}
		else if (q.x > roi_width * 0.55 && lane == -1)//dang phai thay trai
		{
			q.x = q.x - offset - offset_vip;
		}
		else if (q.x < roi_width * 0.45 && lane == 1)//dang trai thay phai
		{
			q.x = q.x + offset + 0 + offset_vip;
		}
		circle(imgLane, Point(q.x,roi_height/2), 5, Scalar(0, 255, 0), 2);
		int val = p.x - roi_width/2;
		
		//PID(double dt, double max, double min, double Kp, double Kd, double Ki);
		//PID pid = PID(0.01, 90, -90, 0.1, 0.01, 0.5); //dt truoc = 0.1
		PID pid = PID(0.015, 90, -90, 0.1, 0.0087, 0.5); //dt truoc = 0.1
		theta = pid.calculate(0, val);

		imshow("laneGreen",imgLane);
		return true;
	} 

	circle(imgLane, Point(p.x,roi_height/2), 5, Scalar(255, 0, 0), 2);
	if (lane == 0)
	{
		lane = (p.x > roi_width*0.55)?lane = 1:lane=-1;
	}
	else
	if (p.x > roi_width * 0.55 && lane == 1)//dang phai thay phai
	{
		 p.x -= (offset + 0);
		//lane = 1;//right
	} 
	else if (p.x < roi_width * 0.45 && lane == -1)//dang trai thay trai
	{
		p.x += offset;
		//lane = -1;//left
	}
	else if (p.x > roi_width * 0.55 && lane == -1)//dang phai thay trai
	{
		p.x = p.x - offset - offset_vip;
	}
	else if (p.x < roi_width * 0.45 && lane == 1)//dang trai thay phai
	{
		p.x = p.x + offset + 0 + offset_vip;
	}
	circle(imgLane, Point(p.x,roi_height/2), 5, Scalar(0, 255, 0), 2);
	int val = p.x - roi_width/2;
	
	//PID(double dt, double max, double min, double Kp, double Kd, double Ki);
	//PID pid = PID(0.01, 90, -90, 0.1, 0.01, 0.5); //dt truoc = 0.1
	PID pid = PID(0.015, 90, -90, 0.1, 0.0087, 0.5); //dt truoc = 0.1
	theta = pid.calculate(0, val);

	imshow("lane",imgLane);
	return true;
}

