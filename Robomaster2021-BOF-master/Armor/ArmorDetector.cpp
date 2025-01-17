/**************************************************************

MIT License

Copyright (c) 2018 SEU-SuperNova-CVRA

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

Authors:	Rick_Hang, <213162574@seu.edu.cn>
		BinYan Hu
**************************************************************/
#include"ArmorDetector.h"
#include<iostream>
#include<vector>
#include<math.h>
#include<string>
#include<chrono>

#include"../General/opencv_extended.h"
#include"../General/numeric_rm.h"



#define MIN(a,b)  ((a) > (b) ? (b) : (a))
#define MAX(a,b)  ((a) < (b) ? (b) : (a))

std::vector<cv::Point3f>armor_history;
std::vector<int>flag_history;
std::vector<cv::Rect>_roi_save;
//std::vector<cv::Point2f> _Armor_rec;


using namespace std;
using namespace cv;
using namespace cv::ml;


namespace rm
{

enum
{
	WIDTH_GREATER_THAN_HEIGHT,
	ANGLE_TO_UP
};
/*
*	@Brief:		// regulate the rotated rect
*	@Input:		// rotated rec
*				// regulation mode
*	@Return:	// regulated rec
*/
cv::RotatedRect& adjustRec(cv::RotatedRect& rec, const int mode)
{
    using std::swap;

    float& width = rec.size.width;
    float& height = rec.size.height;
    float& angle = rec.angle;

    if(mode == WIDTH_GREATER_THAN_HEIGHT)
    {
        if(width < height)
        {
            swap(width, height);
            angle += 90.0;
        }
    }

    while(angle >= 90.0) angle -= 180.0;
    while(angle < -90.0) angle += 180.0;

    if(mode == ANGLE_TO_UP)
    {
        if(angle >= 45.0)
        {
            swap(width, height);
            angle -= 90.0;
        }
        else if(angle < -45.0)
        {
            swap(width, height);
            angle += 90.0;
        }
    }

    return rec;
}


ArmorDescriptor::ArmorDescriptor()
{
	rotationScore = 0;
	sizeScore = 0;
    vertex.resize(4);
	for(int i = 0; i < 4; i++)
	{
        vertex[i] = cv::Point2f(0, 0);
        //points_of_armor = cv::Point2f(0, 0);
	}
	type = UNKNOWN_ARMOR;
}

ArmorDescriptor::ArmorDescriptor(const LightDescriptor & lLight, const LightDescriptor & rLight, const int armorType, const cv::Mat & grayImg, float rotaScore, ArmorParam _param)
{
	//handle two lights
//    lightPairs[0] = lLight.rec();
//    lightPairs[1] = rLight.rec();


    Width = abs(lLight.center.x - rLight.center.x);
    y_diff = fabs(lLight.center.y - rLight.center.y);
    Height = (lLight.length + rLight.length)/2.0;
    lr_rate = lLight.length > rLight.length ? lLight.length / rLight.length : rLight.length / lLight.length;

    if(lLight.angle * rLight.angle == 0.0){// got 0
        if(lLight.angle != 0.0){//right is 0
            if(lLight.angle > 45){
                angle_diff = 90 - lLight.angle;
                Armor_Rotate = ONLY_ONE_0;
            }
            else{
                angle_diff = lLight.angle;
                Armor_Rotate = ONLY_ONE_0;
            }
        }
        else if(rLight.angle != 0.0){//left is 0
            if(rLight.angle > 45){
                angle_diff = 90 - rLight.angle;
                Armor_Rotate = ONLY_ONE_0;
            }
            else{
                angle_diff = rLight.angle;
                Armor_Rotate = ONLY_ONE_0;
            }
        }
        else{//both of them  are 0
            angle_diff = rLight.angle + lLight.angle;
            Armor_Rotate = ALL_0;
        }
    }
    else{//no 0
        Armor_Rotate = SAME_SIDE;
        angle_diff =fabs(lLight.angle - rLight.angle);
    }


    angle_of_armor = std::atan2(rLight.center.y - lLight.center.y, rLight.center.x - lLight.center.x);
    angle_of_armor = angle_diff * 180 / CV_PI;
    angle_of_armor = Width > Height ? abs(angle_of_armor) : 90 - abs(angle_of_armor);
    //        cout<<"the Armor_Rotate:\t"<<Armor_Rotate<<endl;
//        cout<<"angle_diff:\t"<<-angle_diff<<endl;


//    Width = abs(lightPairs[0].center.x - lightPairs[1].center.x);
//    Height = (lightPairs[0].size.height + lightPairs[1].size.height)/2.0;



//    cout<<"Width    "<<Width<<"\nHeight   "<<Height<<endl;
    _armor_center = Point2f((lLight.center.x + rLight.center.x)/2 , (lLight.center.y + rLight.center.y)/2);

//    _armor_center = Point2f((lightPairs[0].center.x+lightPairs[1].center.x)/2 , (lightPairs[0].center.y + lightPairs[1].center.y)/2);

//    cv::Size exLSize(int(lightPairs[0].size.width/2.0), int(lightPairs[0].size.height ));
//    cv::Size exRSize(int(lightPairs[1].size.width/2.0), int(lightPairs[1].size.height ));
//    cv::RotatedRect exLLight(lightPairs[0].center, exLSize, lightPairs[0].angle);
//    cv::RotatedRect exRLight(lightPairs[1].center, exRSize, lightPairs[1].angle);

    cv::RotatedRect armor_size(Point2f(_armor_center.x - Width/2 , _armor_center.y - Height/2),
                               Point2f(_armor_center.x + Width/2 , _armor_center.y - Height/2),
                               Point2f(_armor_center.x + Width/2 , _armor_center.y + Height/2));

    cv::Point2f upper_l =Point2f(lLight.center.x  , lLight.center.y - lLight.length/2);
    cv::Point2f lower_l =Point2f(lLight.center.x  , lLight.center.y + lLight.length/2);
    cv::Point2f upper_r =Point2f(rLight.center.x  , rLight.center.y - rLight.length/2);
    cv::Point2f lower_r =Point2f(rLight.center.x  , rLight.center.y + rLight.length/2);

//    cv::Point2f pts_l[4];
//    exLLight.points(pts_l);
//    cv::Point2f upper_l = pts_l[2];
//    cv::Point2f lower_l = pts_l[3];

//    cv::Point2f pts_r[4];
//    exRLight.points(pts_r);
//    cv::Point2f upper_r = pts_r[1];
//    cv::Point2f lower_r = pts_r[0];

    Point2f pts[4];
    armor_size.points(pts);

    vertex.resize(4);

    vertex[0] = upper_l;
    vertex[1] = upper_r;
    vertex[2] = lower_r;
    vertex[3] = lower_l;

//    vertex[0] = pts[1];
//    vertex[1] = pts[2];
//    vertex[2] = pts[3];
//    vertex[3] = pts[0];

//    vertex[0] = lLight.points[2];
//    vertex[1] = rLight.points[1];
//    vertex[2] = rLight.points[0];
//    vertex[3] = lLight.points[3];

//    points_of_armor[0] = lLight.points[2];
//    points_of_armor[1] = rLight.points[1];
//    points_of_armor[2] = rLight.points[0];
//    points_of_armor[3] = lLight.points[3];
	//set armor type
	type = armorType;
	//get front view
    //getFrontImg(grayImg);
	rotationScore = rotaScore;

	// calculate the size score
	float normalized_area = contourArea(vertex) / _param.area_normalized_base;
	sizeScore = exp(normalized_area);

	// calculate the distance score
	Point2f srcImgCenter(grayImg.cols / 2, grayImg.rows / 2);
	float sightOffset = cvex::distance(srcImgCenter, cvex::crossPointOf(array<Point2f, 2>{vertex[0],vertex[2]}, array<Point2f, 2>{vertex[1],vertex[3]}));
	distScore = exp(-sightOffset / _param.sight_offset_normalized_base);

}

void ArmorDescriptor::getFrontImg(const Mat& grayImg)
{
	using cvex::distance;
	const Point2f&
		tl = vertex[0],
		tr = vertex[1],
		br = vertex[2],
		bl = vertex[3];

	int width, height;
	if(type == BIG_ARMOR)
	{
		width = 92;
		height = 50;
	}
	else
	{
		width = 50;
		height = 50;
	}

	Point2f src[4]{Vec2f(tl), Vec2f(tr), Vec2f(br), Vec2f(bl)};
	Point2f dst[4]{Point2f(0.0, 0.0), Point2f(width, 0.0), Point2f(width, height), Point2f(0.0, height)};
	const Mat perspMat = getPerspectiveTransform(src, dst);
	cv::warpPerspective(grayImg, frontImg, perspMat, Size(width, height));
}

int ArmorDescriptor::ArmorVertex_update(vector<Point2f> imagePoints)
{
    Point2f temp(0.0,0.0);
    if(imagePoints[0] != temp)
    {
      vertex = imagePoints;
    }
    return 0;
}

ArmorDetector::ArmorDetector()
{
    _flag = ARMOR_NO;
    _roi = Rect(cv::Point(0, 0), _srcImg.size());
    _isTracking = false;

#if defined(DEBUG_DETECTION) || defined(SHOW_RESULT)
    _debugWindowName = "debug info";
#endif // DEBUG_DETECTION || SHOW_RESULT
}

ArmorDetector::ArmorDetector(const ArmorParam & armorParam)
{
	_param = armorParam;
	_flag = ARMOR_NO;
	_roi = Rect(cv::Point(0, 0), _srcImg.size());
//    _roiImg_save = _srcImg.clone();
	_isTracking = false;

#if defined(DEBUG_DETECTION) || defined(SHOW_RESULT)
	_debugWindowName = "debug info";
#endif // DEBUG_DETECTION || SHOW_RESULT
}

void ArmorDetector::init(const ArmorParam &armorParam)
{
    _param = armorParam;
}

void ArmorDetector::loadImg(const cv::Mat & srcImg)
{
	_srcImg = srcImg;

    Rect imgBound = Rect(cv::Point(0, 0), _srcImg.size());

    int Roi_state = GLOBAL;

    int size = 1;
    int all=0;
    flag_history.emplace_back(_flag);

    if(flag_history.size()>26){
        for(int i=flag_history.size();i>(flag_history.size()-25);--i)
        {
            all+=flag_history.at(i-1);
        }
    }
    if(flag_history.size() > 1e4)
        flag_history.clear();
    Roi_state=_flag;
    //cout<<"all\t"<<all<<endl;
    //cout<<"flag_history\t"<<flag_history.back()<<endl;

    if(_flag == ARMOR_NO){
        if(all >= 22 ){
            Roi_state = ROI_SAVE;
//            cout<<_cnt_trackCnt<<"\tARMOR_LOCAL"<<endl;
            size = 1;
        }
        else if(all >= 18){
            Roi_state = ROI_SAVE;
//            cout<<_cnt_trackCnt<<"\t\tARMOR_LOCAL"<<endl;
            size = 2;
        }
        else if(all >= 15){
//            cout<<_cnt_trackCnt<<"\t\t\tARMOR_LOCAL"<<endl;
            Roi_state = ROI_SAVE;
            size =3;
        }
        else
            Roi_state = GLOBAL;
    }
    else{
        Roi_state = GET_ROI;
//        cout<<_cnt_trackCnt<<"\t\t\t\tARMOR_LOCAL"<<endl;
//        _Armor_rec = _targetArmor.vertex;
    }

//    cout<<"_Armor_rec:\t"<<_Armor_rec<<endl;


    if(Roi_state == GET_ROI )
    {
        cv::Rect bRect = boundingRect(_targetArmor.vertex) + _roi.tl();
        bRect = cvex::scaleRect(bRect, Vec2f(3 * size, 2 * size));	//以中心为锚点放大2倍

        _roi = bRect & imgBound;
        _roi_save.push_back(_roi);
//        _roi=bRect;
        _roiImg = _srcImg(_roi).clone();
        _cnt_trackCnt = 0;
        _trackCnt++;
    }
    else if(Roi_state==ROI_SAVE)
    {

        cv::Rect cRect = _roi_save.back();
        cRect = cvex::scaleRect(cRect, Vec2f(3 * size,2 * size));	//以中心为锚点放大2倍
        _roi = cRect & imgBound;
        _roiImg = _srcImg(_roi).clone();
        _trackCnt = 0;
        _cnt_trackCnt++;

    }
    else
    {
        _cnt_trackCnt++;
          _roi = imgBound;
          _roiImg = _srcImg.clone();
        _trackCnt = 0;
    }

//    cout<<"_cnt_trackCnt:       "<<std::to_string(_cnt_trackCnt)
//        <<"_trackCnt:       "<<std::to_string(_trackCnt)<<endl;
}


void ArmorDetector::kalman_init()
{
    int stateNum = 4;
    int measureNum = 2;
    KalmanFilter KFC(stateNum, measureNum, 0);

//    KFC(stateNum, measureNum, 0);
    //Mat processNoise(stateNum, 1, CV_32F);
    measurement = Mat::zeros(measureNum, 1, CV_32F);
    KFC.transitionMatrix = (Mat_<float>(stateNum, stateNum) << 1, 0, 1, 0,//A 状态转移矩阵
                                                              0, 1, 0, 1,
                                                              0, 0, 1, 0,
                                                              0, 0, 0, 1);
    //这里没有设置控制矩阵B，默认为零
    setIdentity(KFC.measurementMatrix);//H=[1,0,0,0;0,1,0,0] 测量矩阵
    setIdentity(KFC.processNoiseCov, Scalar::all(1e-5));//Q高斯白噪声，单位阵
    setIdentity(KFC.measurementNoiseCov, Scalar::all(1e-1));//R高斯白噪声，单位阵
    setIdentity(KFC.errorCovPost, Scalar::all(1));//P后验误差估计协方差矩阵，初始化为单位阵
    randn(KFC.statePost, Scalar::all(0), Scalar::all(0.1));//初始化状态为随机值

    KF = KFC;

}


vector<cv::Point2f> ArmorDetector::kalman()
{
//    if(point_armor.empty() != true)
//    {
//        point_armor.clear();
//    }
    double  center_x = 0.0;
    double  center_y = 0.0;


    cv::Size exLSize(int(_targetArmor.lightPairs[0].size.width), int(_targetArmor.lightPairs[0].size.height ));
    cv::Size exRSize(int(_targetArmor.lightPairs[1].size.width), int(_targetArmor.lightPairs[1].size.height ));
    cv::RotatedRect exLLight(_targetArmor.lightPairs[0].center, exLSize, _targetArmor.lightPairs[0].angle);
    cv::RotatedRect exRLight(_targetArmor.lightPairs[1].center, exRSize, _targetArmor.lightPairs[1].angle);


//    center_x = (_targetArmor.lightPairs[0].center.x + _targetArmor.lightPairs[1].center.x)/2 ;
//    center_y = (_targetArmor.lightPairs[0].center.y + _targetArmor.lightPairs[1].center.y)/2 ;


    center_x = (exLLight.center.x + exRLight.center.x)/2 ;
    center_y = (exLLight.center.y + exRLight.center.y)/2 ;


//    cout<<"center_x:    "<<std::to_string(center_x)<<"center_y:    "<<std::to_string(center_y)<<endl;

//    double height_equal = abs(_targetArmor.lightPairs[0].center.x-_targetArmor.lightPairs[1].center.x)/2;
//    double width_equal  = abs((_targetArmor.lightPairs[0].size.height+_targetArmor.lightPairs[0].size.height)/4);


    double height_equal = abs( exLLight.center.x -    exRLight.center.x   )/2;
    double width_equal  = abs((exLLight.size.height + exRLight.size.height)/4);


    Mat prediction = KF.predict();
    //Point predict_pt = Point((int)prediction.at<float>(0), (int)prediction.at<float>(1));

    measurement.at<float>(0) = (float)center_x;
    measurement.at<float>(1) = (float)center_y;
    KF.correct(measurement);



    center_x = (int)prediction.at<float>(0);
    center_y = (int)prediction.at<float>(1);

//    cout<<"center_x(after kalman):            "<<std::to_string(center_x)<<"center_y(after kalman):         "<<std::to_string(center_y)<<endl;

    float x1 =  center_x -  height_equal;
    float y1 =  center_y -  width_equal;
    float x2 =  center_x +  height_equal;
    float y2 =  center_y +  width_equal;

    vector<cv::Point2f> point_armor;
    point_armor.resize(4);

    point_armor.emplace_back(Point2f(x1, y1));
    point_armor.emplace_back(Point2f(x2, y1));
    point_armor.emplace_back(Point2f(x2, y2));
    point_armor.emplace_back(Point2f(x1, y2));

    return point_armor;
}


Vec2f ArmorDetector::kalman_angle()
{


//    cout<<"center_x:    "<<std::to_string(center_x)<<"center_y:    "<<std::to_string(center_y)<<endl;
    if(armor_rec.size() < 1)
    return Vec2f(0,0);

    Vec2f predicted_angle = armor_rec.back();

    Mat prediction = KF.predict();
    //Point predict_pt = Point((int)prediction.at<float>(0), (int)prediction.at<float>(1));

    measurement.at<float>(0) = (float)predicted_angle[0];//Yaw
    measurement.at<float>(1) = (float)predicted_angle[1];//Pitch
    KF.correct(measurement);

    predicted_angle[0] = prediction.at<float>(0);
    predicted_angle[1] = prediction.at<float>(1);

    armor_rec.emplace_back(predicted_angle);
    return predicted_angle;
}

int ArmorDetector::Lost_Track()
{
    return _cnt_trackCnt;
}

int ArmorDetector::detect()
{
	/*
	*	Detect lights and build light bars' desciptors
	*/
#ifdef DEBUG_TIME_CONSUME
    chrono::high_resolution_clock::time_point detect_StartTime = chrono::high_resolution_clock::now();
#endif //DEBUG_TIME_CONSUME

    _armors.clear();

//    imshow("_SrcLimg", _srcImg);

//画出准星以便调试
/******************************************************************************************/
    Point center = Point(_srcImg.size().width/2,_srcImg.size().height/2);
    circle(_srcImg,center,2,cvex::WHITE,-1);
    line(_srcImg,Point(_srcImg.size().width/2-5,_srcImg.size().height/2),
                 Point(_srcImg.size().width/2+5,_srcImg.size().height/2),
                                                             cvex::WHITE);
    line(_srcImg,Point(_srcImg.size().width/2,_srcImg.size().height/2-5),
                 Point(_srcImg.size().width/2,_srcImg.size().height/2+5),
                                                             cvex::WHITE);
/******************************************************************************************/


	std::vector<LightDescriptor> lightInfos;
    Point2f offset_roi_point(_roi.x, _roi.y);

	{
        /******************************************************************************************
        *	pre-treatment
        ******************************************************************************************/
#ifdef DEBUG_TIME_CONSUME
        chrono::high_resolution_clock::time_point pre_treatment_StartTime = chrono::high_resolution_clock::now();
#endif // DEBUG_PRETREATMENT

#ifdef DEBUG_PRETREATMENT
//        imshow("_roiImg", _roiImg);
//        cv::Mat _grayImg;
//        cv::cvtColor(_roiImg, _grayImg, CV_BGR2GRAY);
//        imshow("_grayImg_1", _grayImg);
#endif // DEBUG_PRETREATMENT


//用调整亮度的方法来做预处理，已弃用
/******************************************************************************************/
//        _roiImg.convertTo(_roiImg,_roiImg.type(),1.0,_param.light_Brightness);
//        Adjust_Brightness(_param.light_Brightness);
//        cvex::Adjust_Brightness(_roiImg,_roiImg,_param.light_Brightness);
//        cvex::get_Template();
/******************************************************************************************/

#ifdef DEBUG_PRETREATMENT
//        cv::Mat _grayImg_b;
//        cv::cvtColor(_roiImg, _grayImg_b, CV_BGR2GRAY);
//        imshow("raw_Img", _roiImg);
//        imshow("_grayImg_b", _grayImg_b);
#endif // DEBUG_PRETREATMENT

        std::vector<Mat> channels;
        // 把一个3通道图像转换成3个单通道图像
        split(_roiImg,channels);//分离色彩通道

        //预处理删除己方装甲板颜色
        if(_enemy_color==RED)
             _grayImg_of_Enemy=channels.at(2)-channels.at(0);//Get red-blue image;
        else _grayImg_of_Enemy=channels.at(0)-channels.at(2);//Get blue-red image;

        cv::Mat _grayImg;
        cv::cvtColor(_roiImg, _grayImg, CV_BGR2GRAY);

        cv::Mat binBrightImg;
        cv::Mat binGrayImg;


//        cvtColor(_grayImg_of_Enemy, _grayImg_of_Enemy, COLOR_BGR2GRAY, 1);

//        imshow("_BrightLimg_!!", _grayImg_of_Enemy);

        if(_enemy_color == RED){//detect RED
            cv::threshold(_grayImg_of_Enemy, binBrightImg, 55, 255, cv::THRESH_BINARY);//15
            cv::threshold(_grayImg         , binGrayImg  , 25, 255, cv::THRESH_BINARY);
        }
        else{//detect BLUE
            cv::threshold(_grayImg_of_Enemy, binBrightImg, 100, 255, cv::THRESH_BINARY);
            cv::threshold(_grayImg         , binGrayImg  , 25, 255, cv::THRESH_BINARY);
        }

        cv::Mat element_d = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 , 2));
        dilate(binBrightImg, binBrightImg, element_d);

//        imshow("_grayImg!", binGrayImg);
//        imshow("_subImg!", binBrightImg);

        binBrightImg &= binGrayImg;

//        imshow("add!", binBrightImg);
//        erode(binBrightImg,binBrightImg,element_e);

        cv::Mat element_d0 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3 , 3));
        cv::Mat element_e0 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3 , 3));

        dilate(binBrightImg, binBrightImg, element_d0);

        erode(binBrightImg,binBrightImg,element_e0);


        imshow("dilate!", binBrightImg);

		/*
		*	find and filter light bars
		*/
        vector<vector<Point>> lightContours;
        vector<vector<Point>> GrayContours;


//        cv::findContours(binBrightImg.clone(), lightContours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
        cv::findContours(binBrightImg.clone(), lightContours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
        cv::findContours(binGrayImg.clone()  , GrayContours , CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point());

#ifdef DEBUG_TIME_CONSUME
//        chrono::high_resolution_clock::time_point pre_treatment_endTime = chrono::high_resolution_clock::now();
//        cout<<"pre_treatment_consume:   "\
//            <<to_string((static_cast<chrono::duration<double, std::milli>>(pre_treatment_endTime -pre_treatment_StartTime)).count())\
////            <<"     \n"<<"lightContours:"<<to_string(lightContours.size())
//            <<endl;
#endif

        /******************************************************************************************
        *	treatment
        ******************************************************************************************/
#ifdef DEBUG_TIME_CONSUME
    chrono::high_resolution_clock::time_point treatment_StartTime = chrono::high_resolution_clock::now();
#endif //DEBUG_TIME_CONSUME

    for(size_t ii = 0; ii < lightContours.size(); ii++)
    {

        //筛选掉噪声//
        double area_l = contourArea(lightContours[ii]);
//        cout<<"area_l:\t"<<to_string(area_l)<<endl;
        if(area_l < 4.0 || 1e4 < area_l) continue;
            //筛选掉噪声//

            if(lightContours[ii].size() >=1e4) continue;
//            if(lightContours[ii].size() < 5){
//                cout<<"lightContours[ii].size():\t"<<to_string(lightContours[ii].size())<<endl;
//                continue;
//            }

            double length = arcLength(lightContours[ii], true); // 灯条周长
//            cout<<"length:      "<<to_string(length)<<endl;
            if(length < 7 || length > 100) continue;

            //椭圆拟合生成相应的旋转矩形（注：只是用矩形去拟合灯条，方便获取灯条的长宽比等）
            Rect fliter = boundingRect(lightContours[ii]);
            if(1.2 * fliter.width > fliter.height)continue;
            //Q：采用最小外接矩形法而不是用椭圆拟合？
            //A：经实测，远距离目标（5M及更远）的二值化轮廓可能会有少于5个点包围的情况，会导致椭圆拟合报错导致BUG
            RotatedRect lightRec = minAreaRect(lightContours[ii]);

            //float solidity = lightContourArea / lightRec.size.area();
            //筛选出需要的灯条//

            if(lightRec.angle >-60 && lightRec.angle <-30)continue;

//            cout<<"angle of one LED?:\t"<<to_string(lightRec.angle)<<endl;
//            if(fabs(lightRec.angle) > 30) continue;

            float _ratio;
            if(lightRec.size.width >= lightRec.size.height)
                _ratio = lightRec.size.width / lightRec.size.height;
            else
                _ratio = lightRec.size.height / lightRec.size.width;

//            cout<<"light_ratio?:\t"<<to_string(_ratio)<<endl;

//            cout<<"light_contour_solidity?:\t"<<to_string(area_l / lightRec.size.area())<<endl;

            if((_ratio  > _param.light_min_ratio)
             ||(_ratio  < _param.light_max_ratio))continue;


            if((area_l / lightRec.size.area() < _param.light_contour_min_solidity)
             ||(area_l / lightRec.size.area() > _param.light_contour_max_solidity))continue;

#ifdef DEBUG_PRO
            Point2f rect_point[4];
            lightRec.points(rect_point);
            for(int i =0;i <4;i++)
            {
                line(_srcImg, rect_point[i]+offset_roi_point, rect_point[(i+1)%4]+offset_roi_point, cvex::MAGENTA,1);
            }
//            imshow("LED",_srcImg); //显示筛选出的灯条
#endif //DEBUG_PRO

            lightInfos.push_back(LightDescriptor(lightRec));
    }

#ifdef DEBUG_TIME_CONSUME
    chrono::high_resolution_clock::time_point treatment_endTime = chrono::high_resolution_clock::now();
    cout<<"LED_consume:   "<<to_string((static_cast<chrono::duration<double, std::milli>>(treatment_endTime - treatment_StartTime)).count())<<endl;
#endif //DEBUG_TIME_CONSUME

    #ifdef DEBUG_DETECTION
        vector<RotatedRect> lightsRecs;
        for(auto & light : lightInfos)
        {
            lightsRecs.emplace_back(light.rec());
        }
        cvex::showRectangles(_debugWindowName, _debugImg, _debugImg, lightsRecs, cvex::MAGENTA, 1, _roi.tl());
    #endif //DEBUG_DETECTION

        if(lightInfos.size() < 2) //检查是否检测到灯条//
        {
//            cout<<"not enough leds!!!"<<endl;
            return _flag = ARMOR_NO;
        }
    }

    /******************************************************************************************
	*	find and filter light bar pairs
    *   对灯条进行匹配筛选
    ******************************************************************************************/

	{

#ifdef DEBUG_TIME_CONSUME
        chrono::high_resolution_clock::time_point find_and_filter_StartTime = chrono::high_resolution_clock::now();
#endif //DEBUG_TIME_CONSUME

        //用到了C++11的lambda（可简单看作函数对象），设置了ld1和ld2两个参数，依照灯条中心的x坐标从左到右（opencv的坐标轴为横x竖y）。center为point2f类型的。//
		sort(lightInfos.begin(), lightInfos.end(), [](const LightDescriptor& ld1, const LightDescriptor& ld2)
		{
			return ld1.center.x < ld2.center.x;
		});

        //设一个长为lightInfos.size()，值都为-1的数组//
        //vector<int> minRightIndices(lightInfos.size(), -1);
        //遍历每一种组合

        for(size_t i = 0; i < lightInfos.size(); i++)
        {
            for(size_t j = i + 1; j < lightInfos.size(); j++)
            {

                LightDescriptor& leftLight  = lightInfos[i];
                LightDescriptor& rightLight = lightInfos[j];
                float leftangle  = leftLight.angle;
                float rightangle = rightLight.angle;

//                cout<<"leftangle?:\t"<<to_string(leftangle)<<endl;
//                cout<<"rightangle?:\t\t"<<to_string(rightangle)<<endl;

//                bool Rectangle = (leftangle != 90 && leftangle != 0 && rightLight != 90 && rightLight != 0);
                if(leftangle > 45 && rightangle < 45 && rightangle != 0.0 ){// /\ 内八型灯条对//
//                    cout<<"neiba angle?:\t"<<to_string(90.0 - leftangle + rightangle)<<endl;
//                    if(fabs(leftangle )+ fabs(rightangle) > 10)
                        continue;
                }
                else if(leftangle < 45 && rightangle > 45 && leftangle != 0.0 ){// \/ 外八型灯条对//
//                    cout<<"waiba angle?:\t"<<to_string(90.0 - rightangle + leftangle)<<endl;
//                    if(fabs(rightangle) + fabs(leftangle) > 10)
                        continue;
                }
                else{
//                    cout<<"normal angle?:\t"<<to_string(fabs(leftangle - rightangle))<<endl;
                    if(fabs(leftangle - rightangle) > 10 && fabs(leftangle - rightangle) < 90 - 10)continue;
                }
//                cout<<"leftLight.length?:\t"<<to_string(leftLight.length)<<endl;
//                cout<<"rightLight.length:\t"<<to_string(rightLight.length)<<endl;




                if(leftLight.length * 0.7 > rightLight.length ||
                   leftLight.length * 1.3 < rightLight.length )continue;

//                cout<<"pass!!!"<<endl;

                float armor_Width = fabs(leftLight.center.x - rightLight.center.x);
                if(armor_Width < 2.5 * (leftLight.width + rightLight.width))continue;

//                cout<<"angle:\t"<<fabs(leftLight.angle) + fabs(rightLight.angle)<<endl;

                float h_aver = (leftLight.length + rightLight.length)/2.0f;
                if(fabs(leftLight.center.y - rightLight.center.y) > 0.8f * h_aver)continue;

//                if(leftLight.angle * rightLight.angle < 0 && fabs(leftLight.angle) +fabs(rightLight.angle) > 18)continue;
                //计算左灯和右灯的角度差//
                float angleDiff_ = leftLight.angle - rightLight.angle;
//                /*prevent bar turned out like this / \ */
//                if(angleDiff_< -2.0f)
//                    if(leftLight.angle + 180.0f - rightLight.angle > 15.0f)continue;




#ifdef DEBUG_PRO
//                for(int i =0;i <4;i++)
//                {
//                    line(_srcImg, leftLight.points[i]+offset_roi_point, leftLight.points[(i+1)%4]+offset_roi_point, cvex::CYAN,1);
//                    line(_srcImg, rightLight.points[i]+offset_roi_point, rightLight.points[(i+1)%4]+offset_roi_point, cvex::CYAN,1);
//                }

                Point2f resPoint = offset_roi_point + Point2f((leftLight.center.x + rightLight.center.x)/2.0 , (leftLight.center.y + rightLight.center.y)/2.0);
                circle(_srcImg,resPoint,1,Scalar(0,255,0),2);
//                putText(_srcImg, "  candidate!!!", resPoint, FONT_HERSHEY_PLAIN, 1.0, Scalar(255, 255, 255), 3);//打印字体

//                imshow("lightInfos",_srcImg);  //显示筛选出的灯条对
#endif //DEBUG_PRO


		#ifdef DEBUG_DETECTION
				Mat pairImg = _debugImg.clone();
//                cv::Point p_test(320,240);
//                circle(pairImg,p_test,3,Scalar(0,255,0),-1);
				vector<RotatedRect> curLightPair{leftLight.rec(), rightLight.rec()};
                		cvex::showRectangles("debug pairing",  pairImg, pairImg,curLightPair, cvex::CYAN, 1, _roi.tl());
		#endif // DEBUG_DETECTION

                /******************************************************************************************
                *	Works for 0-5 meters situation
				*	morphologically similar: // parallel 
								 // similar height
                ******************************************************************************************/

                //计算左灯和右灯的长度差之比（越相近该值越小）
                float LenDiff_ratio = fabs(leftLight.length - rightLight.length) / max(leftLight.length, rightLight.length);
                //通过阈值筛选灯条//

                if(LenDiff_ratio > _param.light_max_height_diff_ratio_)continue;

				/*
				*	proper location: // y value of light bar close enough 
                *			        //ratio of length and width is proper
				*/
                //计算左右灯条中心距离  //
				float dis = cvex::distance(leftLight.center, rightLight.center);
                //计算左右灯条长度的均值//
                float meanLen = (leftLight.length + rightLight.length) / 2.0;
                //灯条y的差//
				float yDiff = abs(leftLight.center.y - rightLight.center.y);
                //y差值的比率//
				float yDiff_ratio = yDiff / meanLen;
                //同前//
				float xDiff = abs(leftLight.center.x - rightLight.center.x);
				float xDiff_ratio = xDiff / meanLen;
                //灯条的距离与长度的比值（也就是嫌疑装甲板长和宽的比值）//
				float ratio = dis / meanLen;
//                cout<<"yDiff_ratio\t"<<yDiff_ratio<<endl;

                //对上面各量筛选，如果y差太大（y最好越相近越好），或者x差的太小，又或者装甲板长宽比不合适就排除掉。//
                if(yDiff_ratio > _param.light_max_y_diff_ratio_ ||
                   xDiff_ratio < _param.light_min_x_diff_ratio_ )  continue;


                if(ratio > _param.armor_max_aspect_ratio_ ||
                   ratio < _param.armor_min_aspect_ratio_)  continue;

				// calculate pairs' info 
                //通过长宽比来确定是大的还是小的装x甲板//
                int armorType = ratio > _param.armor_big_armor_ratio ? BIG_ARMOR : SMALL_ARMOR;

				// calculate the rotation score
//                if(armorType == BIG_ARMOR)
//                    cout<<"This is "<<"BIG_ARMOR"<<endl;
//                if(armorType == SMALL_ARMOR)
//                    cout<<"This is "<<"SMALL_ARMOR"<<endl;

				float ratiOff = (armorType == BIG_ARMOR) ? max(_param.armor_big_armor_ratio - ratio, float(0)) : max(_param.armor_small_armor_ratio - ratio, float(0));
				float yOff = yDiff / meanLen;
                //应该是rotationScore越接近0越好，看后续用处//
				float rotationScore = -(ratiOff * ratiOff + yOff * yOff);
                //生成相应的装甲板
//                if(leftLight.angle > 83 ||leftLight.angle <7) leftLight.angle =0.0;
//                if(rightLight.angle > 83 ||rightLight.angle <7) rightLight.angle =0.0;
                ArmorDescriptor armor(leftLight, rightLight, armorType, _grayImg_of_Enemy, rotationScore, _param);
                //将获得的嫌疑装甲板放到armors中去//
				_armors.emplace_back(armor);
			}
        }



#ifdef DEBUG_TIME_CONSUME
//        chrono::high_resolution_clock::time_point find_and_filter_endTime = chrono::high_resolution_clock::now();
//        cout<<"find_and_filter_consume:   "<<to_string((static_cast<chrono::duration<double, std::milli>>(find_and_filter_endTime - find_and_filter_StartTime)).count())<<endl;
#endif //DEBUG_TIME_CONSUME

        //没找到的话。。
		if(_armors.empty())
		{
//            cout<<"There is no armor !!!!!!!!!!!!!!"<<endl;
            return _flag = ARMOR_NO;
		}

    }

#ifdef DEBUG_TIME_CONSUME
    chrono::high_resolution_clock::time_point delete_StartTime = chrono::high_resolution_clock::now();
#endif //DEBUG_TIME_CONSUME

//模板匹配方法，已弃用
//    _armors.erase(remove_if(_armors.begin(), _armors.end(), [](ArmorDescriptor& i)
//	{
//      //lamdba函数判断是不是装甲板，将装甲板中心的图片提取后让识别函数去识别，识别可以用svm或者模板匹配等
//        return !(i.isArmorPattern());
////        return 0==(i.isrealArmorPattern(_small_Armor_template,_big_Armor_template,lastEnemy));
//	}), _armors.end());

#ifdef DEBUG_TIME_CONSUME
//    chrono::high_resolution_clock::time_point delete_endTime = chrono::high_resolution_clock::now();
//    cout<<"delete_consume:   "<<to_string((static_cast<chrono::duration<double, std::milli>>(delete_endTime - delete_StartTime)).count())<<endl;
#endif //DEBUG_TIME_CONSUME

    //没有一个是装甲板的情况
	if(_armors.empty())
	{
		_targetArmor.clear();

		if(_flag == ARMOR_LOCAL)
		{
			//cout << "Tracking lost" << endl;
//			return _flag = ARMOR_LOST;
		}
//		else
//		{
//			//cout << "No armor pattern detected." << endl;
//            return _flag = ARMOR_NO;
//		}
	}

	//calculate the final score
#ifdef DEBUG_TIME_CONSUME
    chrono::high_resolution_clock::time_point calculate_StartTime = chrono::high_resolution_clock::now();
#endif //DEBUG_TIME_CONSUME

    sort(_armors.begin(), _armors.end(),
         [](const ArmorDescriptor & target1, const ArmorDescriptor & target2){
        return target1.Width < target2.Width;
    });
     _targetArmor = _armors[0];

//按照以下六个参数筛选出最终装甲板，实际运用可自行分配调试
    float temp_angle = _armors[0].angle_of_armor;
    float temp_lr_rate = _armors[0].lr_rate;
    float temp_angle_abs = _armors[0].angle_diff;
    float temp_weight = temp_angle + temp_lr_rate;
    float temp_y_diff = _armors[0].y_diff;
    float temp_Width = _armors[0].Width;
    int i;
    for (i = 0; i < _armors.size(); i++){
//        if ( _armors[i].angle_diff  < temp_angle_abs){
//            temp_angle_abs = _armors[i].angle_diff;
            if (_armors[i].angle_of_armor < temp_angle){
                temp_angle = _armors[i].angle_of_armor;
                if(_armors[i].Width >  temp_y_diff){
                    temp_y_diff = _armors[i].Width;
                    if (_armors[i].lr_rate < 1.15)
                        _targetArmor = _armors[i];
                }
            }
//        }
    }


//由于算法的缺陷（没充分利用装甲板信息识别，如装甲板上的数字），故用此算法过滤出符合最终装甲板的类型的装甲板
    int type=-1;
    Point3f aromor_i=Point3f(_targetArmor.type,_targetArmor._armor_center.x,_targetArmor._armor_center.y);
    if(aromor_i!=Point3f(0,0,0)){
        armor_history.push_back(aromor_i);
       // cout<<"armor_history\t"<<armor_history.back()<<endl;
    }

    if((armor_history[armor_history.size()-1].x+
            armor_history[armor_history.size()-2].x+
            armor_history[armor_history.size()-3].x+
            armor_history[armor_history.size()-4].x+
            armor_history[armor_history.size()-5].x+
            armor_history[armor_history.size()-6].x)
            <10 && armor_history.back().x==2)
        type =1;
    else if((armor_history[armor_history.size()-1].x+
             armor_history[armor_history.size()-2].x+
             armor_history[armor_history.size()-3].x+
             armor_history[armor_history.size()-4].x+
             armor_history[armor_history.size()-5].x)
             >7 && armor_history.back().x==1)
         type =2;

//    if(type == 2)
//        cout<<"type\t\t\t"<<type<<endl;
//    else
//        cout<<"type\t"<<type<<endl;

if(type!=-1){
    for(int j=0;j <_armors.size();j++){
        if(_armors[j].type != type)
           _armors.erase(_armors.begin()+j);
    }
}

sort(_armors.begin(), _armors.end(),
     [](const ArmorDescriptor & target1, const ArmorDescriptor & target2){
    return target1.Width < target2.Width;
});
 _targetArmor = _armors[0];

//按照以下六个参数筛选出最终装甲板，实际运用可自行分配调试
temp_angle = _armors[0].angle_of_armor;
temp_lr_rate = _armors[0].lr_rate;
temp_angle_abs = _armors[0].angle_diff;
temp_weight = temp_angle + temp_lr_rate;
temp_y_diff = _armors[0].y_diff;
temp_Width = _armors[0].Width;

for (i = 0; i < _armors.size(); i++){
//        if ( _armors[i].angle_diff  < temp_angle_abs){
//            temp_angle_abs = _armors[i].angle_diff;
        if (_armors[i].angle_of_armor < temp_angle){
            temp_angle = _armors[i].angle_of_armor;
            if(_armors[i].Width >  temp_y_diff){
                temp_y_diff = _armors[i].Width;
                if (_armors[i].lr_rate < 1.15)
                    _targetArmor = _armors[i];
            }
        }
//        }
}

if(_armors.empty())
{
//    cout<<"\t\t\ttest!!!"<<endl;
    return _flag = ARMOR_NO;
}

    _targetArmor.ArmorVertex_update(kalman());

#ifdef DEBUG_TIME_CONSUME
//    chrono::high_resolution_clock::time_point calculate_endTime = chrono::high_resolution_clock::now();
//    cout<<"calculate_consume:   "<<to_string((static_cast<chrono::duration<double, std::milli>>(calculate_endTime - calculate_StartTime)).count())<<endl;
#endif //DEBUG_TIME_CONSUME

#ifdef DEBUG_TIME_CONSUME
//    chrono::high_resolution_clock::time_point detect_endTime = chrono::high_resolution_clock::now();
//    cout<<"detect_consume:   "<<to_string((static_cast<chrono::duration<double, std::milli>>(detect_endTime -detect_StartTime)).count())<<endl;
#endif //DEBUG_TIME_CONSUME

	//update the flag status	
	_trackCnt++;

#ifdef DEBUG_PRO
    for(int i =0;i <4;i++)
    {

        line(_srcImg, _targetArmor.vertex[i]+offset_roi_point, _targetArmor.vertex[(i+1)%4]+offset_roi_point, cvex::GREEN,1);
    }

    rectangle(_srcImg, _roi, cvex::YELLOW);
    imshow("rect_line",_srcImg);//显示筛选出的最终装甲板

#endif //DEBUG_PRO

#if defined(DEBUG_DETECTION) || defined(SHOW_RESULT)
	vector<Point> intVertex;
	for(const auto& point : _targetArmor.vertex)
	{
		Point fuckPoint = point;
		intVertex.emplace_back(fuckPoint);
	}
   	cvex::showContour(_debugWindowName, _debugImg, _debugImg, intVertex, cvex::GREEN, -1, _roi.tl());
#endif //DEBUG_DETECTION || SHOW_RESULT

	return _flag = ARMOR_LOCAL;

}


//模板匹配 根据装甲板中心的图案判断是不是装甲板
//bool ArmorDescriptor::isrealArmorPattern(std::vector<cv::Mat> &small,
//                                     std::vector<cv::Mat> &big ,
//                                     LastenemyType &lastEnemy)
//{
//    //若需要判断装甲中间数字
//#ifdef IS_ARMOR
//    vector<pair<int,double>> score;
//    map<int,double> mp;
//    Mat regulatedImg=frontImg;

//    for(int i=0;i<8;i++){
//        //载入模板，模板是在初始化的时候载入ArmorDetector类，
//        //因为ArmorDescriptor与其非同类需要间接导入
//        Mat tepl=small[i];
//        Mat tepl1=big[i];
//        //模板匹配得到位置，这里没用
//        cv::Mat matchLoc;
//        //模板匹配得分
//        double value;
//        //匹配小装甲
//
//       value = cv::matchTemplate(TemplateMatch(regulatedImg, tepl, matchLoc, CV_TM_CCOEFF_NORMED);
//        mp[i+1]=value;
//        score.push_back(make_pair(i+1,value));
//        //匹配大装甲
//        value = TemplateMatch(regulatedImg, tepl1, matchLoc, CV_TM_CCOEFF_NORMED);
//        mp[i+11]=value;
//        score.push_back(make_pair(i+11,value));
//    }
//    //对该装甲与所有模板匹配后的得分进行排序
//    sort(score.begin(),score.end(), [](const pair<int,double> &a, const pair<int,double> &b)
//    {
//        return a.second > b.second;
//    });
//    //装甲中心位置
//    cv::Point2f c=(vertex[0]+vertex[1]+vertex[2]+vertex[3])/4;
//    //装甲数字即为得分最高的那个
//    int resultNum=score[0].first;
//    //得分太低认为没识别到数字
//    if(score[0].second<0.6)find_and_filter_consume:   0.558688

//    {
//        if(//与上次识别到的装甲板位置差不多，且丢失次数不超过一定值
//                std::abs(std::abs(lastEnemy.center.x)-std::abs(c.x))<10&&
//                std::abs(std::abs(lastEnemy.center.y)-std::abs(c.y))<10&&
//                lastEnemy.lostTimes<100
//                )
//        {//认为该装甲的数字与上次相同
//            lastEnemy.lostTimes++;
//            lastEnemy.center=c;
//            enemy_num=lastEnemy.num;
//            return true;
//        }
//        else
//        {//认为不是装甲
//            return false;
//        }
//    }
//    //当装甲板识别为小装甲，而得到的号码为11、22……时，说明把大装甲识别为了小装甲。
//    if(type==SMALL_ARMOR )
//    {
//        if(resultNum>10)
//        {
//            type=BIG_ARMOR;
//        }
//    }
//    enemy_num=resultNum%10;
//    lastEnemy.num=enemy_num;
//    lastEnemy.center=c;find_and_filter_consume:   0.558688

//    lastEnemy.lostTimes=0;
//    return true;
//#endif
//    //若不需要判断匹配的装甲板中的数字则整个函数直接返回true
//#ifndef IS_ARMOR
//    return true;
//#endif
//}



bool ArmorDescriptor::isArmorPattern() const
{
    // cut the central part of the armor
//    Mat regulatedImg;
//    if(type == BIG_ARMOR)
//    {
//        regulatedImg = frontImg(Rect(21, 0, 50, 50));
//    }
//    else
//    {
//        regulatedImg = frontImg;
//    }

//    resize(regulatedImg,regulatedImg, Size(regulatedImg.size().width / 2, regulatedImg.size().height / 2));
//    // copy the data to make the matrix continuous
//    Mat temp;
//    regulatedImg.copyTo(temp);
//    Mat data = temp.reshape(1, 1);

//    data.convertTo(data, CV_32FC1);

//    Ptr<SVM> svm = StatModel::load<SVM>("/home/dji/Documents/Computer_vision/2021_Robomaster_BOF/Robomaster2021-BOF-master/Armor/SVM2.xml");


//    int result = (int)svm->predict(data);
//    if(result == 1) return true;
//    else return false;
	
	
	// to test the svm, uncomment the code block above
	// and comment the code below
	return true;
}


const std::vector<cv::Point2f> ArmorDetector::getArmorVertex() const
{
    vector<cv::Point2f> realVertex;
    for(int i = 0; i < 4; i++)
	{
        //向容器内添加数据
		realVertex.emplace_back(Point2f(_targetArmor.vertex[i].x + _roi.tl().x, 
										_targetArmor.vertex[i].y + _roi.tl().y));
	}
	return realVertex;
}

int ArmorDetector::getArmorType() const
{
	return _targetArmor.type;
}


#if defined(DEBUG_DETECTION) || defined(SHOW_RESULT)
void ArmorDetector::showDebugImg() const
{
	imshow(_debugWindowName, _debugImg);
}
#endif // DEBUG_DETECTION || SHOW_RESULT
}
