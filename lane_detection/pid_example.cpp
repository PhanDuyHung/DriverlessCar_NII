#include "pid.h"
#include <stdio.h>
#include <iostream>
#include "LaneDetection.h"
#include <time.h>
int main() {

	/*PID pid = PID(0.1, 100, -100, 0.1, 0.01, 0.5);

	double val = 100;
	for (int i = 0; i < 100; i++) {
		double inc = pid.calculate(0, val);
		printf("val:% 7.3f inc:% 7.3f\n", val, inc);
		val += inc;
	}*/


	Mat img = imread("img.jpg", 1);
	imshow("ori", img);

	clock_t begin = clock();
	/* here, do your time-consuming job */

	LaneDetection::ProcessImage(img);

	clock_t end = clock();
	double time_spent = (double)(end - begin) / CLOCKS_PER_SEC;
	std::cout << time_spent;
	imshow("processed", img);

	waitKey(0);
	system("pause");
	return 0;
}