#include "openn2.h"

int main()
{
	OpenNI2 cap;
	cap.init();
	Mat img;
	while(1)
	{
		cap.getBGRImage(img);
		imshow("test",img);
	}
	return 0;
}
