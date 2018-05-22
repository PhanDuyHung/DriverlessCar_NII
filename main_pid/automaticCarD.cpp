/**
	This code runs our car automatically and log video, controller (optional)
	Line detection method: Canny
	Targer point: vanishing point
	Control: pca9685

	You should understand this code run to image how we can make this car works from image processing coder's perspective.
	Image processing methods used are very simple, your task is optimize it.
	Besure you set throttle val to 0 before end process. If not, you should stop the car by hand.
	In our experience, if you accidental end the processing and didn't stop the car, you may catch it and switch off the controller physically or run the code again (press up direction button then enter).
	**/
#include "openni2.h"
#include "../openni2/Singleton.h"
#include "DetecterTrafficSign_NII.h"
#include "api_kinect_cv.h"
// api_kinect_cv.h: manipulate openNI2, kinect, depthMap and object detection
#include "api_lane_detection.h"
// api_lane_detection.h: manipulate line detection, finding lane center and vanishing point
#include "api_i2c_pwm.h"
#include "api_uart.h"
#include"myfuzzy.h"
#include "Hal.h"
#include "LCDI2C.h"

#include <time.h>
#include <iostream>
#include <fstream>
#include <queue>
#include<math.h>
#include<thread>
#include<mutex>
#include<condition_variable>

using namespace openni;
using namespace framework;
using namespace EmbeddedFramework;
#define SAMPLE_READ_WAIT_TIMEOUT 2000 //2000ms
#define VIDEO_FRAME_WIDTH 640
#define VIDEO_FRAME_HEIGHT 480
#define SCARLAR_LOWER_BLUE_OBS 90, 100, 150//98,109,20//170, 0, 0
#define SCARLAR_UPPER_BLUE_OBS 120, 255, 255//112,255,255//270,255,255
#define SCARLAR_LOWER_GREEN_OBS 70,50,30//41, 54, 63
#define SCARLAR_UPPER_GREEN_OBS 90,255,255//71, 255, 255
#define SW1_PIN	164 //160
#define SW2_PIN	163//161
#define SW3_PIN	161 //163
#define SW4_PIN	160 
#define SENSOR	165

int set_throttle_val = 0;
int dir = 0, throttle_val = 0;
int current_state = 0;
int lane = 1;
PCA9685 *pca9685 = new PCA9685();
Point carPosition, centerPosition(-1,-1);
mutex m1;

double Angle(Point pt1, Point pt2, Point pt0)
{
	double dx1 = pt1.x - pt0.x;
	double dy1 = pt1.y - pt0.y;
	double dx2 = pt2.x - pt0.x;
	double dy2 = pt2.y - pt0.y;
	return (dx1*dx2 + dy1*dy2) / sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

int DetectObsDepth(const Mat& img)
{
	int nguong = 60;
	// int min_area = 8000,max = 0;
	// //get object
	
	threshold(img,img,nguong,255,THRESH_BINARY_INV);

	// max = countNonZero(img);
	// //cout <<"area:"<<max<<endl;
	// if (max <= min_area) max = 0;
	// return max;	

	vector<vector<Point>> contours;
	findContours(img,contours,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE);
	vector<Point> approx;
	
	if (approx.size() == 4 &&
		fabs(contourArea(Mat(approx))) > 1000 &&
		isContourConvex(Mat(approx)))
	{
		double maxCosine = 0;

		for (int j = 2; j < 5; j++)
		{
			// find the maximum cosine of the angle between joint edges
			double cosine = fabs(Angle(approx[j % 4], approx[j - 2], approx[j - 1]));
			maxCosine = MAX(maxCosine, cosine);
		}
		if (maxCosine < 0.3)
			{
				//figure out square_center
				Point center = Point((approx[2].x + approx[0].x) / 2, (approx[2].y + approx[0].y) / 2);
				if (center.y > 320) return 1;
				return -1;
			}
	}
	return 0;
}

Point getpoint(Point p = Point(0, 0)){
	lock_guard<mutex> lock(m1);
	if (p == Point(0, 0)){
		return centerPosition;
		
	}
	else{
		centerPosition = p;
		return Point(0, 0);
	}
}

/// Get depth Image or BGR image from openNI device
/// Return represent character of each image catched
char analyzeFrame(const VideoFrameRef& frame, Mat& depth_img, Mat& color_img) {
	DepthPixel* depth_img_data;
    RGB888Pixel* color_img_data;

    depth_img = Mat(VIDEO_FRAME_HEIGHT,VIDEO_FRAME_WIDTH, CV_16U);
    Mat depth_img_8u;
    color_img = Mat(VIDEO_FRAME_HEIGHT, VIDEO_FRAME_WIDTH, CV_8UC3);
	
	
    switch (frame.getVideoMode().getPixelFormat())
    {
   	case PIXEL_FORMAT_DEPTH_1_MM: return 'm';
    case PIXEL_FORMAT_DEPTH_100_UM:
	
				
        depth_img_data = (DepthPixel*)frame.getData();
        memcpy(depth_img.data, depth_img_data, VIDEO_FRAME_HEIGHT*VIDEO_FRAME_WIDTH*sizeof(DepthPixel));
		
        normalize(depth_img, depth_img_8u, 255, 0, NORM_MINMAX);
		//normalize(depth_img, depth_img, 255, 0, NORM_MINMAX);

        depth_img_8u.convertTo(depth_img_8u, CV_8U);
		//depth_img.convertTo(depth_img, CV_8U);
		
        depth_img = depth_img_8u;
		// imshow("depth",depth_img);
		 
		return 'd';
        break;
    case PIXEL_FORMAT_RGB888:
        color_img_data = (RGB888Pixel*)frame.getData();

        memcpy(color_img.data, color_img_data, VIDEO_FRAME_HEIGHT*VIDEO_FRAME_WIDTH*sizeof(RGB888Pixel));

        cvtColor(color_img, color_img, COLOR_RGB2BGR);
		return 'c';
        break;
		
    default:
        printf("Unknown format\n");
		return 'u';
    }
}
//
void clear(queue<double> &q)
{
	queue<double> empty;
	swap(q, empty);
}
double et = 0, tong = 0;
double dieukhienpid(double in){
	in = in / 40;
	double kp = 0, kd = 0, ki = 0;
	double theta = in*kp + kd*(in - et) + tong;
	et = in;
	//tong += in*ki;
	return theta;
}
//double dieukhienpid(double in){
//	double kp = 0.48, kd = 0, ki = 0.05;
//	double theta = in*kp + kd*(in - et) + tong;
//	et = in;
//	tong += in*ki;
//	return theta;
//}
/// Return angle between veritcal line containing car and destination point in degree
double getTheta(Point car, Point dst) {
	double goc;
	if (dst.x == car.x) return 0;
	//if (dst.y == car.y) return (dst.x < car.x ? -90 : 90);
	double pi = acos(-1.0);
	double dx = dst.x - car.x;
	double dy = car.y - dst.y;// + 40; // image coordinates system: car.y > dst.y
	if (dx < 0){
		goc = -atan(-dx / dy) * 180 / pi;
	}
	else{
		goc = atan(dx / dy) * 180 / pi;
	}
	return goc;
}
double prv_e = 0;
double getTheta_me(Point car, Point dst)
{
	double alpha, theta;
	if (dst.x == car.x) theta = 0;
	//if (dst.y == car.y) alpha = (dst.x < car.x ? -90 : 90);
	else
	{
		double L = 25, ld = 50;
		double pi = acos(-1.0);
		double dx = dst.x - car.x;
		double dy = car.y - dst.y + ld; // image coordinates system: car.y > dst.y
		// if (dx < 0) alpha = -atan(-dx / dy) * 180 / pi;
		// else alpha = atan(dx / dy) * 180 / pi;

		if (dx < 0) theta = -atan((2 * L / ld)*(-dx / sqrt(dx*dx + dy*dy))) * 180 / pi;

		else theta = atan((2 * L / ld)*(dx / sqrt(dx*dx + dy*dy))) * 180 / pi;
		//PID
		double kP = 0.01, kI = 0.02;
		// theta=theta+kP*dx+kI*(dx-prv_e);
		prv_e = dx;
		//
	}
	return theta;
}

int cport_nr; // port id of uart.
char buf_send[BUFF_SIZE]; // buffer to store and recive controller messages.

char buf_recv[BUFF_SIZE];

/// Write speed to buffer
void setThrottle(int speed) {
	// speed=60;
	if (speed >= 0)
		sprintf(buf_send, "f%d\n", speed);
	else {
		speed = -speed;
		sprintf(buf_send, "b%d\n", speed);
	}
}
double distance(){
	int n, kc = BUFF_SIZE, giatri = 0, chot = 0;
	if (cport_nr == -1) {
		cerr << "Error: Canot Open ComPort";
		return -1;
	}
	n = api_uart_read(cport_nr, buf_recv);
	if (n > 0)
	{
		buf_recv[n] = 0;
	}
	for (int i = 0; i < BUFF_SIZE; i++){
		if (buf_recv[i] == 'e'){
			kc = i;
		}
		if (i>kc){
			if (buf_recv[i] >= '0'&&buf_recv[i] <= '9')
				giatri = giatri * 10 + (buf_recv[i] - '0');
			else {
				kc = BUFF_SIZE;
				chot = giatri;
				giatri = 0;
			}

		}
		buf_recv[i] = 0;
	}
	return (double)chot * 117 / 20000;
}

double dieukhienmo(double in1){
	static HamThuoc *hamthuoc[8];
	hamthuoc[0] = new hamhinhthangtrai(-55, -40);
	hamthuoc[1] = new hamtamgiac(-55, -40, -30);
	hamthuoc[2] = new hamtamgiac(-40, -30, -20);
	hamthuoc[3] = new hamtamgiac(-30, -20, -5);
	hamthuoc[4] = new hamtamgiac(5, 20, 30);
	hamthuoc[5] = new hamtamgiac(20, 30, 40);
	hamthuoc[6] = new hamtamgiac(30, 40, 55);
	hamthuoc[7] = new hamhinhthangphai(40, 55);

	static ThuocTinh saiso(8, hamthuoc);
	static ThuocTinh tt[1] = { saiso };
	static double	maxvalue[8] = { 200, 180, 130, 80, -80, -130, -180, -200 };
	//static double	maxvalue[8] = { 200, 170, 120, 70, -70, -120, -170, -200 };
	static MyFuzzy myfuzzy(tt, maxvalue);
	//myfuzzy.GetTable(57,35);
	static double vValue;
	vValue = myfuzzy.getvalue(in1);
	return vValue;
}

void dieukhienxe(){
	static double oldthrottval = 0;
	double newtheta;
	Point p = getpoint();
	double angDiff = getTheta(carPosition, getpoint());
	newtheta = dieukhienmo(angDiff);
	//double theta = (-angDiff * 3);
	int pwm2 = api_set_STEERING_control(pca9685, newtheta);
	throttle_val = dieukhienpid(abs(newtheta));
	throttle_val = set_throttle_val - throttle_val;
	if (abs(throttle_val - oldthrottval) > 2){
		oldthrottval = throttle_val;
		api_set_FORWARD_control(pca9685, throttle_val);
		api_uart_write(cport_nr, buf_send);
	}
}

bool ready = false;
mutex m;
condition_variable cd;
void dieukhien(){
	int kc;
	while (1){
		unique_lock<mutex> lk(m);
		cd.wait(lk, []{return ready; });
		dieukhienxe();
		ready = false;
	}
}

Mat img_obs;
int hasObs = 0;
int timeObs = 12;
Mat GetImageObs(Mat img = Mat())
{
	lock_guard<mutex> lock(m1);
	if (img.empty()) return img_obs;
	img_obs = img.clone();
	return Mat();
}
std::mutex m3;
Mat img_traffic;
Mat GetImageColor(Mat img = Mat())
{
	lock_guard<mutex> lock(m3);
	if (img.empty()) return img_traffic;
	img_traffic = img.clone();
	return Mat();
}
int DetectObsColor(Mat& img)
{
	int min_area = 3000,max = 0;
	Mat imgRed,maskRed1,maskRed2,imgBlue,imgGreen;
	cvtColor(img,img,CV_BGR2HSV);

	inRange(img,Scalar(SCARLAR_LOWER_RED_1),Scalar(SCARLAR_UPPER_RED_1),maskRed1);
	inRange(img,Scalar(SCARLAR_LOWER_RED_2),Scalar(SCARLAR_UPPER_RED_2),maskRed2);
	add(maskRed1,maskRed2,imgRed);
	//add(maskRed1,maskRed2,img);

	// inRange(img,Scalar(SCARLAR_LOWER_BLUE_OBS),Scalar(SCARLAR_UPPER_BLUE_OBS),imgBlue);
	// add(imgRed,imgBlue,img);

	inRange(img,Scalar(SCARLAR_LOWER_GREEN_OBS),Scalar(SCARLAR_UPPER_GREEN_OBS),imgGreen);
	add(imgGreen,imgRed,img);

	normalize(img,img,0,255,NORM_MINMAX);
	medianBlur(img,img,5);
	GaussianBlur(img,img,Size(5,5),0);
	threshold(img,img,127,255,CV_THRESH_BINARY);

	max = countNonZero(img);
	//cout <<"area:"<<max<<endl;
	if (max <= min_area) max = 0;
	return max;		
}
std::mutex m_obs;
std::condition_variable cv_obs;
bool ready_obs = false;
bool processed_obs = false;
void DetectObsThread()
{
    // Mat img,roi_img_left,roi_img_right;
	// float w = 640, h =480;
	// int roi_width = 150, roi_height = h * 0.35;
	// cv::Rect roi_left = cv::Rect(70, h * 0.5,
	// roi_width, roi_height);
	// cv::Rect roi_right = cv::Rect(420, h * 0.5,
	// roi_width, roi_height);
	Mat img,roi_img;
	float w = 640,h = 480;
	int roi_width = w, roi_height = h * 0.35;
	cv::Rect roi = cv::Rect(0, h * 0.3,roi_width, roi_height);
    while (1)
    {
		
        std::unique_lock<std::mutex> lk(m_obs);
        cv_obs.wait(lk,[]{return ready_obs;});

		if (!hasObs)
        {
			img = GetImageObs();
			if (!img.empty())
			{
				//imshow("obs",img);
				// roi_img_left = img(roi_left).clone();
				// roi_img_right = img(roi_right).clone();
				// imshow("obsleft",roi_img_left);
				// imshow("obsright",roi_img_right);
				//int area_left = DetectObsColor(roi_img_left);
				//int area_right = DetectObsColor(roi_img_right);
				//int area_left = DetectObsDepth(roi_img_left);
				//int area_right = DetectObsDepth(roi_img_right);
				// //hasObs = DetectObs(img);
				//imshow("left",roi_img_left);
				//imshow("right",roi_img_right);
				// cout <<"left:"<<area_left<<" right:"<<area_right<<endl;
				// if (area_left == 0 && area_right == 0) hasObs = 0;
				// else {
				// 	if (area_left>area_right) 
				// 	{
				// 		hasObs = -1;
				// 	}
				// 	else
				// 	{
				// 		 hasObs = 1;
				// 	}
				// 	//hasObs = (area_left>area_right)? - 1: 1;
				// }
				//cout << "obs:"<<hasObs<<endl;
				roi_img = img(roi).clone();
				imshow("obj",roi_img);
				imwrite("obj.jpg",roi_img);
				hasObs = DetectObsDepth(roi_img);
				imshow("obj_bin",roi_img);
				cout << "obs:"<<hasObs<<endl;
				if (hasObs != 0)
				{
					timeObs = 20;
					//cout << "obs:"<<hasObs<<endl;
				}
			}
		}
        processed_obs = true;
        ready_obs = false;
        lk.unlock();
        cv_obs.notify_one();
    }
}

std::mutex m_imgControlCar;
std::mutex m_controlCar;
std::condition_variable cv_controlCar;
bool ready_controlCar = false;
bool processed_controlCar = false;
Mat img_ControlCar;
double theta = 0;
Mat GetImageControlCar(Mat img = Mat())
{
	lock_guard<mutex> lock(m_imgControlCar);
	if (img.empty()) return img_ControlCar;
	img_ControlCar = img.clone();
	return Mat();
}

void ControlCarThread()
{
	Mat img;
    while (1)
    {
        std::unique_lock<std::mutex> lk(m_controlCar);
        cv_controlCar.wait(lk,[]{return ready_controlCar;});

		
		img = GetImageControlCar();
		if (!img.empty())
		{
			//imshow("test",img);
			double angDiff  = 0;
			bool hasCenter = CenterPoint_NII(img,angDiff,lane);	
			if (hasCenter) 
			{
				if ( (abs(theta - 0.95*angDiff) > 100))
					{
						//(^-^)!
					}
				else theta = 0.95*angDiff;//60:1   65:0.9
				//theta = 1*angDiff;
			}


			//else throttle_val = set_throttle_val - 5;

		}
		
        processed_controlCar = true;
        ready_controlCar = false;
        lk.unlock();
        cv_controlCar.notify_one();
    }
}

std::mutex m_traffic;
std::condition_variable cv_traffic;
bool ready_traffic = false;
bool processed_traffic = false;
int labelTraffic = -1;
void DetectTrafficThread()
{
    Mat img;
    while (1)
    {
        std::unique_lock<std::mutex> lk(m_traffic);
        cv_traffic.wait(lk,[]{return ready_traffic;});
        img = GetImageColor();
        if (!img.empty())
        {
            labelTraffic = Detecter_NII::GetTrafficSignDetected(img);
        }
        processed_traffic = true;
        ready_traffic = false;
        lk.unlock();
        cv_traffic.notify_one();
    }
}

///////// utilitie functions  ///////////////////////////

bool isTraffic = false;
int main(int argc, char* argv[]) {

	GPIO *gpio = new GPIO();
	I2C *i2c_device = new I2C();
	LCDI2C *lcd = new LCDI2C();
	ofstream thetalog;
	thetalog.open("thetalog.txt", ios::out);
	thetalog << "====Theta log====" << endl;
	
	int sw1_stat = 0;
	int sw2_stat = 0;
	int sw3_stat = 0;
	int sw4_stat = 0;
	int sensor = 1;

	// Setup input
	gpio->gpioExport(SW1_PIN);
	gpio->gpioExport(SW2_PIN);
	gpio->gpioExport(SW3_PIN);
	gpio->gpioExport(SW4_PIN);
	gpio->gpioExport(SENSOR);

	gpio->gpioSetDirection(SW1_PIN, INPUT);
	gpio->gpioSetDirection(SW2_PIN, INPUT);
	gpio->gpioSetDirection(SW3_PIN, INPUT);
	gpio->gpioSetDirection(SW4_PIN, INPUT);
	gpio->gpioSetDirection(SENSOR, INPUT);

	i2c_device->m_i2c_bus = 2;
	
	if (!i2c_device->HALOpen()) {
		printf("Cannot open I2C peripheral\n");
		exit(-1);
	} else printf("I2C peripheral is opened\n");
	
	unsigned char data;
	if (!i2c_device->HALRead(0x38, 0xFF, 1, &data, "")) {
		printf("LCD is not found!\n");
		//exit(-1);
	} else printf ("LCD is connected\n");
	
	usleep(10000);
	lcd->LCDInit(i2c_device, 0x38, 20, 4);
	lcd->LCDBacklightOn();
	lcd->LCDCursorOn();
	lcd->LCDSetCursor(6,0);
	lcd->LCDPrintStr("NII Team");
	lcd->LCDSetCursor(2,1);
	lcd->LCDPrintStr("Self Car Driving");
	lcd->LCDSetCursor(0, 2);
	lcd->LCDPrintStr("Speed:");
	


//tf
	OpenNI2::Instance() -> init();
	Mat depthImg, colorImg, grayImage, disparity;
	
	//double theta = 0;

	char key = 0;

	bool hasTraffic = false;
	int timeTurnHasTraffic = 5; //20
	//=========== Init  =======================================================

	////////  Init PCA9685 driver   ///////////////////////////////////////////


	api_pwm_pca9685_init(pca9685);

	if (pca9685->error >= 0)
		api_set_STEERING_control(pca9685, theta);

	/////////  Init UART here   ///////////////////////////////////////////////

	
	//std::thread first(dieukhien);
	std::thread thread_controlCar(ControlCarThread);
    //std::thread thread_obs(DetectObsThread);
    //std::thread thread_traffic(DetectTrafficThread);
	if (cport_nr == -1) {
		cerr << "Error: Canot Open ComPort";
		return -1;
	}

	////////  Init direction and ESC speed  ///////////////////////////
	dir = DIR_REVERSE;
	throttle_val = 0;
	theta = 0;
	// int nguong;
    // if (argc == 3) 
	// {nguong = atoi(argv[2]);
	// set_throttle_val = atoi(argv[1]);
	// }
	// Argc == 2 eg ./test-autocar 27 means initial throttle is 27
	if (argc == 2) set_throttle_val = atoi(argv[1]);
	fprintf(stderr, "Initial throttle: %d\n", set_throttle_val);
	
	//getpoint(Point(video_VIDEO_FRAME_WIDTH / 2, video_frame_height * 7 / 8));
	//int frame_width = VIDEO_FRAME_WIDTH;
	//int frame_height = VIDEO_FRAME_HEIGHT;

	//Point prvPosition(VIDEO_FRAME_WIDTH / 2 - 14, VIDEO_FRAME_HEIGHT * 7 / 8);
	//============ End of Init phase  ==========================================

	//============  PID Car Control start here. You must modify this section ===

	bool running = false, started = false, stopped = false;

	double st = 0, et = 0, fps = 0;
	double freq = getTickFrequency();

	bool is_show_cam = true;

	int countF = 0;
	queue<double> Q;
	double S = 0;
	double newtheta = 0;
	double distanceLane = VIDEO_FRAME_WIDTH / 2;
	int count = 0;
	bool hasSensor = false;
	
	unsigned int sensor_status = 1;
	unsigned int sensor_old = 1;
	unsigned int bt_val[4], bt_old_val[4];

	std::vector<TrafficSign> vecTraffic;

	bool barie = true;
	char ch_array[4];

	while (true)
	{
		// int count_store_theta = 3;
		// double store_theta = 0;
		Point tmp_point(0, 0);
		st = getTickCount();
		key = getkey();

		
		gpio->gpioGetValue(SW1_PIN, &bt_val[0]);
		gpio->gpioGetValue(SW2_PIN, &bt_val[1]);
		gpio->gpioGetValue(SW3_PIN, &bt_val[2]);
		gpio->gpioGetValue(SW4_PIN, &bt_val[3]);
		// if (bt_val[0] == 0 && bt_old_val[0] == 1) {
		// 	set_throttle_val += 5;
		// 	sprintf(ch_array, "%d", set_throttle_val);
		// 	lcd->LCDSetCursor(7,2);
		// 	lcd->LCDPrintStr("   ");
		// 	lcd->LCDSetCursor(7,2);
		// 	lcd->LCDPrintStr(ch_array);
		// }
		if (bt_val[2] == 0 && bt_old_val[2] == 1) {
			key = 's';
		}
		if (bt_val[3] == 0 && bt_old_val[3] == 1) {
			key = 'f';
		}
		bt_old_val[0] = bt_val[0];
		bt_old_val[1] = bt_val[1];
		bt_old_val[2] = bt_val[2];
		bt_old_val[3] = bt_val[3];
	
		//cout << "bt1:" << bt_val[0] << endl;

		//gpio->gpioGetValue(SENSOR, &sensor_status);



		if (key == 's') {
			distanceLane = VIDEO_FRAME_WIDTH / 2;
			running = !running;
			barie = true;
			//clear(Q);
			// S = 0;
			// //  Reset=0;
			// tong = 0;
		}
		if (key == 'f') {
			thetalog.close();
			fprintf(stderr, "End process.\n");
			theta = 0;
			throttle_val = 0;
			api_set_FORWARD_control(pca9685, throttle_val);
			api_uart_write(cport_nr, buf_send);
			api_set_STEERING_control(pca9685, theta);
			break;
		}

		
		if (running) 
		{

			// while(barie)
			// {
			// 	gpio->gpioGetValue(SENSOR, &sensor_status);
			// 	if (sensor_status)
			// 	{
			// 		barie = false;
			// 	} 
			// }
			
			//// Check PCA9685 driver ////////////////////////////////////////////

			if (pca9685->error < 0)
			{
				cout << endl << "Error: PWM driver" << endl << flush;
				break;
			}
			if (!started)
			{
				fprintf(stderr, "ON\n");
				started = true; stopped = false;
				throttle_val = set_throttle_val;
				api_set_FORWARD_control(pca9685, throttle_val);
				api_uart_write(cport_nr, buf_send);
			}
			throttle_val = set_throttle_val;

			////////  Get input image from camera   //////////////////////////////
			
			
			OpenNI2::Instance()->getData(colorImg, depthImg, grayImage, disparity);
			
			imshow("disparity",disparity);
			////////// Detect Center Point ////////////////////////////////////

			//if (recordStatus == 'c')
			if(0)
			{
				/**
					Status = BGR Image. Since we developed this project to run a sample, we only used information of gray image.
					**/
				flip(colorImg, colorImg, 1);
				//imshow("color", colorImg);
				//cvtColor(colorImg, grayImage, CV_BGR2GRAY);
				
				GetImageControlCar(colorImg);

//NII ne vat can
			// {
			//     std::lock_guard<std::mutex> lk(m_obs);
			//     ready_obs = true;
			// }
			// cv_obs.notify_one();
			// {
			//     std::unique_lock<std::mutex> lk(m_obs);
			//     cv_obs.wait(lk,[]{return processed_obs;});
			//     processed_obs = false;
			// }
			// //cout << "==========="<<timeObs<<endl;
			// if (hasObs) 
			// {
			// 	setOffset(50);
			// 	hasObs = 0;
			// }
			// if (timeObs == 0)
			// {
			// 	setOffset(-50);
			// }
			// else timeObs--; 
		

	//NII traffic
				//clock_t begin = clock();
				int labelTraffic = Detecter_NII::GetTrafficSignDetected(colorImg);
				//cout <<"label:"<<labelTraffic<<endl;
				// clock_t end = clock();
				// double time_spent = (double)(end - begin)/CLOCKS_PER_SEC;
				// cout <<"time lane"<<time_spent<<endl;

				if (labelTraffic != -1)
				{
					hasTraffic = true;
					//if (set_throttle_val > 45) throttle_val = set_throttle_val - 5;
					if (labelTraffic == 1) //turn right
					{
						cout <<"=================right================"<<endl;
						if (isTraffic)
						{
							if (lane != 1)
							{
								theta = -90;
								lane = 1;
							}
							isTraffic = false;
						}
						isTraffic = true;
					}
					else if (labelTraffic == 6)//turn left
					{
						cout <<"=================left================"<<endl;
						//if (isTraffic)
						{
							if (lane != -1) 
							{
								theta = +90;
								lane = -1;
							}
						//	isTraffic = false;
						}
						//isTraffic = true;
					}	
					else if (labelTraffic == 2) //stop
					{
						cout <<"===============stop============="<<endl;
						if (isTraffic) 
						{
							running = false;
							isTraffic = false;
						}
						isTraffic = true;
					}
				}


				if (hasTraffic)
					if (timeTurnHasTraffic > 0)
					{
						timeTurnHasTraffic--;
						//cout <<"time turn:"<<timeTurnHasTraffic<<endl;
						api_set_STEERING_control(pca9685, theta); 
						api_set_FORWARD_control(pca9685, throttle_val);
						//cout <<"=============================theta: "<<theta<<endl;
						continue;
					}
					else 
					{
						hasTraffic = false;
						timeTurnHasTraffic = 5;
					}
				
				//		get center point of lane
				
//NII control
			{
			    std::lock_guard<std::mutex> lk(m_controlCar);
			    ready_controlCar = true;
			}
			cv_controlCar.notify_one();
			{
			    std::unique_lock<std::mutex> lk(m_controlCar);
			    cv_controlCar.wait(lk,[]{return processed_controlCar;});
			    processed_controlCar = false;
			}				
				
				
				
				cout <<"theta: "<<theta<<endl;
				cout << "lane: " << lane << endl;
				
//Fri-6-4-18
				api_set_STEERING_control(pca9685, theta);
				////////// using for debug stuff  ///////////////////////////////////
				if (is_show_cam) {
					waitKey(1);
				}
			}
				
			
			///////  Your PID code here"  //////////////////////////////////////////
			//cout << "Throttle: " << throttle_val << endl;
			//if (abs(theta)>60) throttle_val = set_throttle_val - 5;
			api_set_FORWARD_control(pca9685, throttle_val);
			et = getTickCount();
			fps = 1.0 / ((et - st) / freq);
			//if (abs(theta) > 40) usleep(200000);
			//            cout << "FPS: "<< fps<< "   throttle:"<<throttle_val<< '\n';

			// waitKey(1);
			//if (key == 27) break;
		}
		else
		{	
			thetalog.close();
			theta = 0;
			throttle_val = 0;
			if (!stopped)
			{
				fprintf(stderr, "OFF\n");
				stopped = true; started = false;
				api_set_FORWARD_control(pca9685, throttle_val);
				api_uart_write(cport_nr, buf_send);
			}
			api_set_STEERING_control(pca9685, theta);
			//sleep(1);
		}

	}
	//////////  Release //////////////////////////////////////////////////////
	// if(is_save_file)
	//        {
	//        gray_videoWriter.release();
	//        color_videoWriter.release();
	//        //depth_videoWriter.release();
	//        fclose(thetaLogFile);
	// }
	return 0;
}