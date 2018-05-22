#include "DetecterTrafficSign_NII.h"

int main()
{
    Mat img = imread("/home/ubuntu/DriverlessCarChallenge_MTARacer-master/round_1/DriverlessCar/carControl/src/0.3/imageDetected.jpg",1);
    imshow("img",img);
    Detecter_NII::init();
    int labelTraffic = Detecter_NII::GetTrafficSignDetected(img);
    cout <<"label:"<<labelTraffic<<endl;
    cout <<"done";
    waitKey(0);
    return 0;
}