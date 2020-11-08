#include<iostream>  
#include<opencv2/opencv.hpp>  
#include<fstream>  

using namespace cv;
using namespace std;
typedef unsigned short      UINT16, *PUINT16;

vector<Point> src;

int i = 0;
int nWidth = 512;
int nHeight = 424;
UINT16 *pBuffer = NULL;

Mat depth_image(nHeight, nWidth, CV_16UC1);
Mat DepthImage(nHeight, nWidth, CV_16UC1);

Point p;
void onMouse(int event, int x, int y, int flags, void *param)
{
    Mat *img = reinterpret_cast<Mat*>(param);
    if (event == CV_EVENT_LBUTTONDOWN)//左键按下，读取初始坐标  
    {
        i++;//统计点击的次数  
        p.x = x;
        p.y = y;
        src.push_back(p);
        cout << p << static_cast<int>(img->at<unsigned short>(cv::Point(x, y))) << endl;
        cout << i << endl;
        cout << p.x << " " << p.y << " " << static_cast<int>(img->at<unsigned short>(cv::Point(x, y))) << endl;
    }
}
int main(int argc, char** argv)
{
    Mat frame;
    depth_image = imread("resource", IMREAD_ANYDEPTH);
    while (true)
    {
        namedWindow("image", CV_WINDOW_AUTOSIZE);
        setMouseCallback("image", onMouse, &depth_image);
        imshow("image", depth_image*20);
        waitKey(1);
    }
    return 0;
}