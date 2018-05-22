#include "DetecterTrafficSign_NII.h"



vector<TrafficSign> Detecter_NII::trafficSigns;
PyObject* Detecter_NII::pyModule = NULL;
PyObject* Detecter_NII::pyFuncAcc = NULL;
PyObject* Detecter_NII::pyFuncPredict1 = NULL;
PyObject* Detecter_NII::pyFuncPredict2 = NULL;
PyObject* Detecter_NII::pyFuncPredict3 = NULL;
PyObject* Detecter_NII::pyResultAcc = NULL;
PyObject* Detecter_NII::pyResultPredict = NULL;

Detecter_NII::Detecter_NII()
{

}
Detecter_NII::~Detecter_NII()
{
	delete pyModule;
	delete pyFuncPredict1;
	delete pyFuncPredict2;
	delete pyFuncPredict3;
	delete pyFuncAcc;
	delete pyResultAcc;
	delete pyResultPredict;
}

void Detecter_NII::init()
{
		// Init python
    Py_Initialize();
	//PySys_SetArgv(argc, argv);

	//Import Module
	pyModule = PyImport_ImportModule("Predict");
	pyFuncPredict1 = PyObject_GetAttrString(pyModule, "getPredict1");
	pyFuncPredict2 = PyObject_GetAttrString(pyModule, "getPredict2");
	pyFuncPredict3 = PyObject_GetAttrString(pyModule, "getPredict3");
	pyFuncAcc = PyObject_GetAttrString(pyModule, "getAcc");
	PyErr_Occurred();
	PyErr_Print();
}

void Detecter_NII::FindEllipses(const Mat& img, const vector<Point>&contour) {

	if (contour.size() >= 5)
	{
		RotatedRect temp = fitEllipse(Mat(contour));
		//if (abs(temp.size.width - temp.size.height) > 20) continue;
		//=======loại bỏ các hình không nằm hoàn toàn trong image hoac những hình không phải hình tròn
		if (temp.size.width > temp.size.height) //ellipse nam ngang
		{
			if (temp.size.width / temp.size.height > RATIO) return;
			if (temp.center.x - temp.size.height / 2 < 0 || temp.center.x + temp.size.height / 2 > img.cols || temp.center.y - temp.size.width / 2 < 0 || temp.center.y + temp.size.width / 2 > img.rows) return;
		}
		else
		{
			if (temp.size.height / temp.size.width > RATIO) return;
			if (temp.center.x - temp.size.width / 2 < 0 || temp.center.x + temp.size.width / 2 > img.cols || temp.center.y - temp.size.height / 2 < 0 || temp.center.y + temp.size.height / 2 > img.rows) return;
		}

		TrafficSign traffic;
		traffic.setCenTer(temp.center);
		//int length = (int)(temp.size.width < temp.size.height) ? (int)temp.size.height : (int)temp.size.width;
		traffic.setSize(temp.size);
		//cout << "size dong 67:"<<temp.size.width<<" "<<temp.size.height<<endl;
		if (temp.size.width >= MINIMUM_SIZE && temp.size.height >= MINIMUM_SIZE
		&& temp.size.width <= MAXIMUM_SIZE && temp.size.height <= MAXIMUM_SIZE)
			//push into trafficSigns
			trafficSigns.push_back(traffic);
	}

}

void Detecter_NII::DetectTrafficSigns(const Mat& imgSrc)
{	
	Mat img;
	cvtColor(imgSrc, img, CV_BGR2HSV);

	Mat imgBlue,imgRed1,imgRed2,imgRed,mask;
	inRange(img, Scalar(SCARLAR_LOWER_BLUE), Scalar(SCARLAR_UPPER_BLUE), imgBlue);
	//inRange(img, Scalar(SCARLAR_LOWER_RED_2), Scalar(SCARLAR_UPPER_RED_2), imgRed2);
	//inRange(img, Scalar(SCARLAR_LOWER_RED_1), Scalar(SCARLAR_UPPER_RED_1), imgRed1);
	//add(imgRed1,imgRed2,imgRed);
	//imshow("red",imgRed);
	//imshow("blue",imgBlue);
	//add(imgRed,imgBlue,mask);
	mask = imgBlue;
	normalize(mask, mask, 0, 255, NORM_MINMAX);
	//khử nhiễu
	medianBlur(mask, mask, 5);
	//xóa đối tượng nhỏ
	Mat blured; GaussianBlur(mask, blured, Size(5, 5), 0);
	//tách biên
	Mat edge; Canny(blured, edge, 30, 100);
	

	//phóng to để nối liền ảnh
	Mat kernel = getStructuringElement(MORPH_RECT, Size(2, 2));
	Mat dilation; dilate(edge, dilation, kernel);

	Mat img_detect;  threshold(dilation, img_detect, 127, 255, CV_THRESH_BINARY);
	// find contours and store them all as a list
	vector<vector<Point> >contours;
	findContours(img_detect, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

	int size = contours.size();
	for (int i = 0;i<size;i++)
	{
		/*============CIRCLE===============*/
		FindEllipses(img, contours[i]);
	}
	if (trafficSigns.empty()) return;

	//sap xep theo thu giam dan
	stack<int> s;
	int left = 0, right = (int)trafficSigns.size() - 1;
	s.push(left); s.push(right);
	
	while (!s.empty())
	{
		right = s.top(); s.pop();
		left = s.top(); s.pop();
		int i = left, j = right, k = (i + j) / 2;
		while (i <= j)
		{
			int length_i = trafficSigns[i].getSize().width + trafficSigns[i].getSize().height;
			int length_j = trafficSigns[j].getSize().width + trafficSigns[j].getSize().height;
			int length_k = trafficSigns[k].getSize().width + trafficSigns[k].getSize().height;
			while (length_i < length_k) {
				i++;
				length_i = trafficSigns[i].getSize().width + trafficSigns[i].getSize().height;
			}
			while (length_j > length_k) {
				j--;
				length_j = trafficSigns[j].getSize().width + trafficSigns[j].getSize().height;
			}
			if (i <= j)
			{
				TrafficSign t = trafficSigns[i];
				trafficSigns[i] = trafficSigns[j];
				trafficSigns[j] = t;
				i++; j--;
			}
		}
		if (i < right) {
			s.push(i); s.push(right);
		}
		if (j > left){
			s.push(left); s.push(j);
		}
	}

	//xoa nhung duong tron nam trong duong tron khac
	for (int i = (int)trafficSigns.size() - 1; i > 0; --i) {
		Point center_i = trafficSigns[i].getCenTer();
		//float radius_i = (float)trafficSigns[i].getLength() / 2;
		int radius_i = ((trafficSigns[i].getSize().width > trafficSigns[i].getSize().height) ? trafficSigns[i].getSize().width : trafficSigns[i].getSize().height)/2;
		

		for (int j = i - 1; j >= 0; --j){
			Point center_j = trafficSigns[j].getCenTer();
			int radius_j = ((trafficSigns[j].getSize().width > trafficSigns[j].getSize().height) ? trafficSigns[j].getSize().width : trafficSigns[j].getSize().height) / 2;
			//float radius_j = (float)trafficSigns[j].getLength() / 2;

			double d = sqrt((center_i.x - center_j.x)*(center_i.x - center_j.x) + (center_i.y - center_j.y)*(center_i.y - center_j.y));
			if (d < radius_i + radius_j)
			{
				trafficSigns.erase(trafficSigns.begin() + j);
				i--;
			}
		}
	}
}

void Detecter_NII::DetectTrafficSigns(const Mat& imgSrc,char c)
{	
	Mat img;
	cvtColor(imgSrc, img, CV_BGR2HSV);

	Mat mask;
	if (c=='b') 
	{
		inRange(img, Scalar(SCARLAR_LOWER_BLUE), Scalar(SCARLAR_UPPER_BLUE), mask);
		//imshow("blue",mask);
	}	
	else 
	{
		Mat imgRed1,imgRed2;
		//inRange(img, Scalar(SCARLAR_LOWER_RED_2), Scalar(SCARLAR_UPPER_RED_2), mask);
		inRange(img, Scalar(SCARLAR_LOWER_RED_2), Scalar(SCARLAR_UPPER_RED_2), imgRed2);
		inRange(img, Scalar(SCARLAR_LOWER_RED_1), Scalar(SCARLAR_UPPER_RED_1), imgRed1);
		add(imgRed1,imgRed2,mask);
		//imshow("red",mask);
	}
	//inRange(img, Scalar(SCARLAR_LOWER_RED_2), Scalar(SCARLAR_UPPER_RED_2), imgRed2);
	//inRange(img, Scalar(SCARLAR_LOWER_RED_1), Scalar(SCARLAR_UPPER_RED_1), imgRed1);
	//add(imgRed1,imgRed2,imgRed);
	//imshow("red",imgRed);
	//imshow("blue",imgBlue);
	//add(imgRed,imgBlue,mask);
	normalize(mask, mask, 0, 255, NORM_MINMAX);
	//khử nhiễu
	medianBlur(mask, mask, 5);
	//xóa đối tượng nhỏ
	Mat blured; GaussianBlur(mask, blured, Size(5, 5), 0);
	//tách biên
	Mat edge; Canny(blured, edge, 30, 150);
	

	//phóng to để nối liền ảnh
	Mat kernel = getStructuringElement(MORPH_RECT, Size(4, 4));
	Mat dilation; dilate(edge, dilation, kernel);

	Mat img_detect;  threshold(dilation, img_detect, 127, 255, CV_THRESH_BINARY);
	//if (c!='b') imshow("red",img_detect);
	// find contours and store them all as a list
	vector<vector<Point> >contours;
	findContours(img_detect, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

	int size = contours.size();
	for (int i = 0;i<size;i++)
	{
		/*============CIRCLE===============*/
		FindEllipses(img, contours[i]);
	}
	if (trafficSigns.empty()) return;

	//sap xep theo thu giam dan
	stack<int> s;
	int left = 0, right = (int)trafficSigns.size() - 1;
	s.push(left); s.push(right);
	
	while (!s.empty())
	{
		right = s.top(); s.pop();
		left = s.top(); s.pop();
		int i = left, j = right, k = (i + j) / 2;
		while (i <= j)
		{
			int length_i = trafficSigns[i].getSize().width + trafficSigns[i].getSize().height;
			int length_j = trafficSigns[j].getSize().width + trafficSigns[j].getSize().height;
			int length_k = trafficSigns[k].getSize().width + trafficSigns[k].getSize().height;
			while (length_i < length_k) {
				i++;
				length_i = trafficSigns[i].getSize().width + trafficSigns[i].getSize().height;
			}
			while (length_j > length_k) {
				j--;
				length_j = trafficSigns[j].getSize().width + trafficSigns[j].getSize().height;
			}
			if (i <= j)
			{
				TrafficSign t = trafficSigns[i];
				trafficSigns[i] = trafficSigns[j];
				trafficSigns[j] = t;
				i++; j--;
			}
		}
		if (i < right) {
			s.push(i); s.push(right);
		}
		if (j > left){
			s.push(left); s.push(j);
		}
	}

	//xoa nhung duong tron nam trong duong tron khac
	for (int i = (int)trafficSigns.size() - 1; i > 0; --i) {
		Point center_i = trafficSigns[i].getCenTer();
		//float radius_i = (float)trafficSigns[i].getLength() / 2;
		int radius_i = ((trafficSigns[i].getSize().width > trafficSigns[i].getSize().height) ? trafficSigns[i].getSize().width : trafficSigns[i].getSize().height)/2;
		

		for (int j = i - 1; j >= 0; --j){
			Point center_j = trafficSigns[j].getCenTer();
			int radius_j = ((trafficSigns[j].getSize().width > trafficSigns[j].getSize().height) ? trafficSigns[j].getSize().width : trafficSigns[j].getSize().height) / 2;
			//float radius_j = (float)trafficSigns[j].getLength() / 2;

			double d = sqrt((center_i.x - center_j.x)*(center_i.x - center_j.x) + (center_i.y - center_j.y)*(center_i.y - center_j.y));
			if (d < radius_i + radius_j)
			{
				trafficSigns.erase(trafficSigns.begin() + j);
				i--;
			}
		}
	}
}

void Detecter_NII::DrawTrafficSigns(Mat& imgSrc,TrafficSign traffic)
{
		Point center = traffic.getCenTer();
		Size size = traffic.getSize();
		//tìm tọa độ 4 đỉnh A B C D
		Point A, B, C, D;
		A = Point(center.x - size.width / 2, center.y - size.height / 2);
		B = Point(center.x + size.width / 2, center.y - size.height / 2);
		C = Point(center.x + size.width / 2, center.y + size.height / 2);
		D = Point(center.x - size.width / 2, center.y + size.height / 2);
		//Draw square
		line(imgSrc, A, B, Scalar(SCALAR_SOLID), THICKNESS);
		line(imgSrc, B, C, Scalar(SCALAR_SOLID), THICKNESS);
		line(imgSrc, C, D, Scalar(SCALAR_SOLID), THICKNESS);
		line(imgSrc, D, A, Scalar(SCALAR_SOLID), THICKNESS);
}

Mat Detecter_NII::CutTrafficSign(const Mat& imgSrc, TrafficSign& traffic) {

	Point c = traffic.getCenTer();
	Size size = traffic.getSize();
	Range x(0, 0), y(0, 0);
	x.start = (c.x - size.width / 2 >= 0) ? c.x - size.width / 2 : 0;
	x.end = (c.x + size.width / 2 <= imgSrc.cols) ? c.x + size.width / 2 : imgSrc.cols;
	y.start = (c.y - size.height / 2 >= 0) ? c.y - size.height / 2 : 0;
	y.end = (c.y + size.height / 2 <= imgSrc.rows) ? c.y + size.height / 2 : imgSrc.rows;

	Mat temp(imgSrc, y, x);
	return temp;
}

double alpha = 1.1;
int belta = 50;
void Detecter_NII::BrightUp(const Mat& imgSrc,Mat& img)
{
	img = Mat::zeros(imgSrc.size(),imgSrc.type());
	for (int y = 0; y<imgSrc.rows;y++)
	{
		for (int x = 0;x<imgSrc.cols;x++)
		{
			for (int c = 0;c<3;c++)
			{
				img.at<Vec3b>(y,x)[c] = saturate_cast<uchar>(alpha*(imgSrc.at<Vec3b>(y,x)[c]) +belta);
			}
		}
	}
}

int roi_width = 240, roi_height = 480 * 0.2;
cv::Rect roi = cv::Rect(200, 150,
		roi_width, roi_height);
int Detecter_NII::GetTrafficSignDetected(Mat& frame)
{
	int result = -1;
	
	Mat img = frame(roi).clone();
	//imwrite("roi.jpg", img);
	DetectTrafficSigns(img);
	int size = trafficSigns.size();
	for (int i=0;i<size;i++)
	{
		Mat imgTraffic = CutTrafficSign(img, trafficSigns[i]);
		imwrite("imageDetected1.jpg", imgTraffic);

		//======RECOGNIZE BY PYTHON==========
		pyResultPredict = PyObject_CallObject(pyFuncPredict1, NULL);
		int id = (int)PyFloat_AsDouble(pyResultPredict);
		//sss
		//cout << "=====================id : " << id << endl;
		pyResultAcc = PyObject_CallObject(pyFuncAcc, NULL);
		float acc = PyFloat_AsDouble(pyResultAcc);
		//cout << "acc:"<<acc<<endl;
		
		if (acc > MINIMUM_ACCURACY && id < 2) result = id;
	}
	//imshow("traffic",img);
	trafficSigns.clear();
	return result;
}

int Detecter_NII::GetTrafficSignDetected(const Mat& img1,const Mat& img2,const Mat& img3)
{
	int result = -1;
	

	imwrite("imageDetected1.jpg", img1);

	//======RECOGNIZE BY PYTHON==========
	pyResultPredict = PyObject_CallObject(pyFuncPredict1, NULL);
	int id1 = (int)PyFloat_AsDouble(pyResultPredict);
	//sss
	////cout << "id 1 : " << id1 << endl;
	pyResultAcc = PyObject_CallObject(pyFuncAcc, NULL);
	float acc1 = PyFloat_AsDouble(pyResultAcc);
	////cout << "acc 1 :"<<acc1<<endl;


	imwrite("imageDetected2.jpg", img2);

	//======RECOGNIZE BY PYTHON==========
	pyResultPredict = PyObject_CallObject(pyFuncPredict2, NULL);
	int id2 = (int)PyFloat_AsDouble(pyResultPredict);
	//sss
	////cout << "id 2 : " << id2 << endl;
	pyResultAcc = PyObject_CallObject(pyFuncAcc, NULL);
	float acc2 = PyFloat_AsDouble(pyResultAcc);
	////cout << "acc 2 :"<<acc2<<endl;

	imwrite("imageDetected3.jpg", img3);

	//======RECOGNIZE BY PYTHON==========
	pyResultPredict = PyObject_CallObject(pyFuncPredict3, NULL);
	int id3 = (int)PyFloat_AsDouble(pyResultPredict);
	//sss
	//cout << "id 3 : " << id3 << endl;
	pyResultAcc = PyObject_CallObject(pyFuncAcc, NULL);
	float acc3 = PyFloat_AsDouble(pyResultAcc);
	//cout << "acc 3 :"<<acc3<<endl;
	
	if (acc1 >= 0.999 && id1 != 0 && acc1>acc2 && acc1>acc3) result = id1;
	if (acc2 >= 0.999 && id2 != 0 && acc2>acc1 && acc2>acc3) result = id2;
	if (acc3 >= 0.999 && id3 != 0 && acc3>acc2 && acc3>acc1) result = id3;
	
	if (result == 0) result = -1;
	//imshow("traffic1",img1);imshow("traffic2",img2);imshow("traffic3",img3);
	
	return result;
}

int Detecter_NII::GetTrafficSignDetectedRed(const Mat& img)
{
	int result = -1;
	Mat imgDetect;// = img.clone();
	BrightUp(img,imgDetect);
	DetectTrafficSigns(imgDetect,'r');
	imshow("imgDetectRed",imgDetect);
	int size = trafficSigns.size();
	for (int i=0;i<size;i++)
	{
		Mat imgTraffic = CutTrafficSign(img, trafficSigns[i]);
		imwrite("imageDetected1.jpg", imgTraffic);

		//======RECOGNIZE BY PYTHON==========
		pyResultPredict = PyObject_CallObject(pyFuncPredict1, NULL);
		int id = (int)PyFloat_AsDouble(pyResultPredict);
		//sss
		cout << "=====================id : " << id << endl;
		pyResultAcc = PyObject_CallObject(pyFuncAcc, NULL);
		float acc = PyFloat_AsDouble(pyResultAcc);
		cout << "acc:"<<acc<<endl;
		
		if (acc > MINIMUM_ACCURACY_STOP && (id == 2 || id == 3))
		{
			if (trafficSigns[i].getSize().width >= MINIMUM_SIZE_STOP &&trafficSigns[i].getSize().width <= MAXIMUM_SIZE_STOP
			   &&trafficSigns[i].getSize().height >= MINIMUM_SIZE_STOP &&trafficSigns[i].getSize().height <= MAXIMUM_SIZE_STOP)
			result =2;
		}
	}
	trafficSigns.clear();
	return result;
}

int Detecter_NII::GetTrafficSignDetectedBlue(const Mat& img)
{
	int result = -1;
	Mat imgDetect;// = img.clone();
	BrightUp(img,imgDetect);
	DetectTrafficSigns(imgDetect,'b');
	//imshow("imgDetectBlue",imgDetect);
	int size = trafficSigns.size();
	for (int i=0;i<size;i++)
	{
		Mat imgTraffic = CutTrafficSign(img, trafficSigns[i]);
		imwrite("imageDetected2.jpg", imgTraffic);

		//======RECOGNIZE BY PYTHON==========
		pyResultPredict = PyObject_CallObject(pyFuncPredict2, NULL);
		int id = (int)PyFloat_AsDouble(pyResultPredict);
		//sss
		cout << "=====================id : " << id << endl;
		pyResultAcc = PyObject_CallObject(pyFuncAcc, NULL);
		float acc = PyFloat_AsDouble(pyResultAcc);
		cout << "acc:"<<acc<<endl;
		
		if (acc > MINIMUM_ACCURACY && id < 2)
		{
			result = id;
		}
	}
	trafficSigns.clear();
	return result;
}