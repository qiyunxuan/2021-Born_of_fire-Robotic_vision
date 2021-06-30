#pragma once
#include<opencv2/opencv.hpp>
#include<array>
#include"../General/General.h"
#include<opencv2/ml.hpp>
using namespace cv;
using namespace std;

namespace rm
{


enum Energy_Center_State
{
    FOUND = 1,
    NOT_FOUND = 0,
};

class EnergyDetector
{
public:
    int detect_video();
    int detect();
    void setEnemyColor(int enemy_color);
    void loadImg(const cv::Mat & srcImg);
    Point2i Windwill_pred(Point2i armor_center); //返回能量机关预测击打点

    int markred(Mat srcImage);
    const std::vector<cv::Point2f> getEnergyVertex() const;


    Point2f diff_angle(Point2f resPoint);//respoint refers to pixel different,need PNP first!
    Point2i center_of_Aromr;
    Point2f _predicted_armors_vertex[4];
    RotatedRect last_armor;//need
private:
    vector <Point2i>all_center;//need

    double radius =0.0;  //need 拟合圆的半经

    Mat srcImage,_roiImg;//need

    Rect _roi;



    float W =-(10.0*360.0/60.0)*(8.0/28.0)*(CV_PI/180.0);//   8m  25m/s //射速  10/9.55;  1 Rad/sec = 9.55 RPM

    float dist = 8000; //间隔
    double theta,delta,alpha,beta;   //度数
    float H = 80;    //云台旋转中心到底盘相机的竖直距离
    float D = 80;    //云台旋转中心到底盘相机的水平距离
    float drop; //弹道下坠补偿

    float width_of_last_armor;
    float height_of_last_armor;

    Point2i center_of_Windmill = Point2i(0,0);//need


    int _enemy_color;
    int _self_color;
};

}
