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
#pragma once
#include<opencv2/opencv.hpp>
#include<array>
#include"../General/General.h"
#include<opencv2/ml.hpp>
/**************************************************************
 * DEBUG_PRETREATMENT 	only shows the image after the simple pretreatment
 * DEBUG_DETECTION	record the info of: 
 * 				    	// roi area						: yellow
 * 					// all the possible light bars				: magenta
 * 					// the possible light pairs(DEBUG_PAIR overlapped)	: cyan
 *					// the vertex of possible armor areas			: white
 * 					// the vertex of result armor areas			: green
 * SHOW_RESULT		only shows the detection result(show the vertex of armors)		: green
 * GET_ARMOR_PIC	collect the samples of armor area
 * 
 * Notice:		1. all the macro definition can be used individually
 * 			2. if you want to focus on particular part of processing, I suggest commenting all the
 * 			   unrelated part of DEBUG_DETECTION in .cpp. Or you can just rewrite the debug interactive mode.
 * 			3. remeber to change the path of pictures if using GET_ARMOR_PIC
 **************************************************************/
//#define DEBUG_PRETREATMENT

#define DEBUG_PRO
//#define DEBUG_DETECTION
//#define SHOW_RESULT

//#define DEBUG_TIME_CONSUME
//#define ARMOR_SUCCESS_RATE

//#define IS_ARMOR
//#define IS_ARMOR
//#define GET_ARMOR_PIC


namespace rm
{

/*
*	This struct store all parameters(svm excluded) of armor detection 
*/

enum Armor_Rotate_Type
{
    ALL_0 = 0,        //  ||
    SAME_SIDE = 1,    //  //
    ONLY_ONE_0 = 2,   //  \|   OR  /|
};



struct ArmorParam
{
	//Pre-treatment
	int brightness_threshold;
    int gray_threshold;
    float light_Brightness;
	int color_threshold;
	float light_color_detect_extend_ratio;

	//Filter lights
	float light_min_area;
	float light_max_angle;
	float light_min_size;
	float light_contour_min_solidity;
    float light_contour_max_solidity;
    float light_max_ratio;
    float light_min_ratio;

	//Filter pairs
	float light_max_angle_diff_;
	float light_max_height_diff_ratio_; // hdiff / max(r.length, l.length)
	float light_max_y_diff_ratio_;  // ydiff / max(r.length, l.length)
	float light_min_x_diff_ratio_;

	//Filter armor
	float armor_big_armor_ratio;
	float armor_small_armor_ratio;
	float armor_min_aspect_ratio_;
	float armor_max_aspect_ratio_;

	//other params
	float sight_offset_normalized_base;
	float area_normalized_base;
	int enemy_color;
	int max_track_num = 3000;

	/*
	*	@Brief: 为各项参数赋默认值
	*/
	ArmorParam()
	{
		//pre-treatment
        light_Brightness = -0.0;//200.0
        brightness_threshold = 15; //40
        gray_threshold = 20;    //10
		color_threshold = 40;

		light_color_detect_extend_ratio = 1.1;

		// Filter lights
        light_min_area = 10;//10
        light_max_angle = 30.0;
		light_min_size = 5.0;
        light_contour_min_solidity = 0.15;//0.5
        light_contour_max_solidity = 1.0;
        light_max_ratio = 1.5;//1.0 0.33
        light_min_ratio = 15.0; //1.0

		// Filter pairs
        	light_max_angle_diff_ = 7.0; //20
        	light_max_height_diff_ratio_ = 0.2; //0.5
        light_max_y_diff_ratio_ = 2.0; //100
		light_min_x_diff_ratio_ = 0.5; //100

		// Filter armor
		armor_big_armor_ratio = 3.2;
		armor_small_armor_ratio = 2;
		//armor_max_height_ = 100.0;
		//armor_max_angle_ = 30.0;
        armor_min_aspect_ratio_ = 1.7;//1.0
        armor_max_aspect_ratio_ = 6.5;//5.0

		//other params
		sight_offset_normalized_base = 200;
		area_normalized_base = 1000;
		enemy_color = BLUE;
	}
};

/*
*   This class describes the info of lights, including angle level, width, length, score
*/
class LightDescriptor
{
public:
    LightDescriptor() {}
	LightDescriptor(const cv::RotatedRect& light)
	{

        if(light.size.width < light.size.height){
            width = light.size.width;
            length = light.size.height;
        }
        else{
            width = light.size.height;
            length = light.size.width;
        }        

        angle = light.angle;
        angle = fabs(angle);
        if(angle == 90.0) angle = 0.0;


		center = light.center;
		area = light.size.area();
        light.points(points);

	}
	const LightDescriptor& operator =(const LightDescriptor& ld)
	{
		this->width = ld.width;
		this->length = ld.length;
		this->center = ld.center;
		this->angle = ld.angle;
		this->area = ld.area;

		return *this;
	}

	/*
	*	@Brief: return the light as a cv::RotatedRect object
	*/
	cv::RotatedRect rec() const
	{
		return cv::RotatedRect(center, cv::Size2f(width, length), angle);
	}

public:
	float width;
	float length;
	cv::Point2f center;
	float angle;
    float angle_ellip;
	float area;
    cv::Point2f points[4];
};

/*
* 	This class describes the armor information, including maximum bounding box, vertex and so on
*/
class ArmorDescriptor
{
public:

	/*
	*	@Brief: Initialize with all 0
	*/
    ArmorDescriptor();

	/*
	*	@Brief: calculate the rest of information(except for match&final score)of ArmroDescriptor based on:
			l&r light, part of members in ArmorDetector, and the armortype(for the sake of saving time)
	*	@Calls: ArmorDescriptor::getFrontImg()
	*/
	ArmorDescriptor(const LightDescriptor& lLight, const LightDescriptor& rLight, const int armorType, const cv::Mat& srcImg, const float rotationScore, ArmorParam param);

	/*
	*	@Brief: empty the object
	*	@Called :ArmorDetection._targetArmor
	*/
	void clear()
	{
		rotationScore = 0;
		sizeScore = 0;
		distScore = 0;
        finalScore = 0;
        Armor_Rotate = 0;
        for(int i = 0; i < 4; i++)
		{
			vertex[i] = cv::Point2f(0, 0);
		}
		type = UNKNOWN_ARMOR;
	}

	/*
	*	@Brief: get the front img(prespective transformation) of armor(if big, return the middle part)
	*	@Inputs: grayImg of roi
	*	@Outputs: store the front img to ArmorDescriptor's public
	*/
	void getFrontImg(const cv::Mat& grayImg);

    /*
    *	@Brief: get the front img(prespective transformation) of armor(if big, return the middle part)
    *	@Inputs: grayImg of roi
    *	@Outputs: store the front img to ArmorDescriptor's public
    */
    int ArmorVertex_update(const std::vector<cv::Point2f> imagePoints);


	/*
	*	@Return: if the centeral pattern belong to an armor
	*/
    	bool isArmorPattern() const;


        bool isrealArmorPattern(std::vector<cv::Mat> &small,
                                std::vector<cv::Mat> &big);
public:



	std::array<cv::RotatedRect, 2> lightPairs; //0 left, 1 right

	float sizeScore;		//S1 = e^(size)
	float distScore;		//S2 = e^(-offset)
	float rotationScore;		//S3 = -(ratio^2 + yDiff^2) 
    float finalScore;

    float Width;
    float Height;
    float y_diff;
    float angle_diff;
    float angle_of_armor;
    float lr_rate;

    int Armor_Rotate;

    cv::Point2f _armor_center;
    //cv::Point2f points_of_armor[4];
    std::vector<cv::Point2f> vertex; //four vertex of armor area, lihgt bar area exclued!!
    cv::Mat frontImg; //front img after prespective transformation from vertex,1 channel gray img

    //	1 -> small
    //	2 -> big
    //  0 -> unkown
	int type;
};

/*
*	This class implement all the functions of detecting the armor
*/
class ArmorDetector
{

public:
    //cv::Vec2f shoot_Armor_angle;
    int last_armor_type[10];
    //std::vector<cv::Point3f>armor_history;
    std::vector<cv::Vec2f>armor_rec;
    std::vector<cv::Point2f>armor_lp;
    /*
    *	flag for the detection result
    */
    enum ArmorFlag
    {
        ARMOR_NO = 0,		// not found
        ARMOR_LOCAL = 1		// armor found locally(in tracking mode)
    };


    enum Roi_State
    {
        GET_ROI = 1,
        ROI_SAVE=2,
        GLOBAL = 0
    };


public:
    ArmorDetector();
    ArmorDetector(const ArmorParam& armorParam);
    ~ArmorDetector(){}

    /*
    *	@Brief: Initialize the armor parameters
    *	@Others: API for client
    */
    void init(const ArmorParam& armorParam);

    /*
    *	@Brief: set the enemy's color
    *	@Others: API for client
    */
    void setEnemyColor(int enemy_color)
    {
        _enemy_color = enemy_color;
        _self_color = enemy_color == BLUE ? RED : BLUE;
    }

    /*
    *	@Brief: load image and set tracking roi
    *	@Input: frame
    *	@Others: API for client
    */
    void loadImg(const cv::Mat&  srcImg);

    /*
    *	@Brief: core of detection algrithm, include all the main detection process
    *	@Outputs: ALL the info of detection result
    *	@Return: See enum ArmorFlag
    *	@Others: API for client
    */
    int detect();

    //std::vector<int>armor_rec;
    //std::vector <cv::Point2f>armor_rec;

    /*
    *	@Brief: get the vertex of armor
    *	@Return: vector of four cv::point2f object
    *	@Notice: Order->left-top, right-top, right-bottom, left-bottom
    *	@Others: API for client
    */
    const std::vector<cv::Point2f> getArmorVertex() const;

    /*
    *	@Brief: returns the type of the armor
    *	@Return: 0 for small armor, 1 for big armor
    *	@Others: API for client
    */
    int getArmorType() const;

    /*
    *	@Brief: returns the type of the armor
    *	@Return: 0 for small armor, 1 for big armor
    *	@Others: API for client
    */
    void kalman_init();

    /*
    *	@Brief: returns the type of the armor
    *	@Return: 0 for small armor, 1 for big armor
    *	@Others: API for client
    */
    std::vector<cv::Point2f> kalman();


    cv::Vec2f kalman_angle();
    /*
    *	@Brief: returns the type of the armor
    *	@Return: 0 for small armor, 1 for big armor
    *	@Others: API for client
    */
    int Lost_Track();


//    void get_Template();

//    void Adjust_Brightness(float light_Brightness)
//    {
//        for (int r1= 0; r1 < _roiImg.rows; r1++) {
//            for (int c = 0; c < _roiImg.cols; c++) {
//                if (_roiImg.channels() == 3) {
//                    double b = _roiImg.at<cv::Vec3b>(r1, c)[0];
//                    double g = _roiImg.at<cv::Vec3b>(r1, c)[1];
//                    double r = _roiImg.at<cv::Vec3b>(r1, c)[2];

//                    _roiImg.at<cv::Vec3b>(r1, c)[0] = cv::saturate_cast<uchar>(b - light_Brightness);
//                    _roiImg.at<cv::Vec3b>(r1, c)[1] = cv::saturate_cast<uchar>(g - light_Brightness);
//                    _roiImg.at<cv::Vec3b>(r1, c)[2] = cv::saturate_cast<uchar>(r - light_Brightness);

//                }
//                else {
//                    if (_roiImg.channels() == 1) {
//                        double v = _roiImg.at<cv::Vec3b>(_roiImg.rows, _roiImg.cols)[0];
//                        _roiImg.at<uchar>(r1, c) = cv::saturate_cast<uchar>(v + light_Brightness);

//                    }
//                }
//            }
//        }

//    }
#if defined(DEBUG_DETECTION) || defined(SHOW_RESULT)
    void showDebugImg() const;
#endif // DEBUG_DETECTION || SHOW_RESULT

private:
    ArmorParam _param;
    int _enemy_color;
    int _self_color;

//    std::vector<cv::Mat> small;
//    std::vector<cv::Mat> big  ;
//    std::string Template_Location;


    cv::Rect _roi; //relative coordinates

    std::vector <int> _flag_rec;

    cv::Mat _srcImg; //source img
    cv::Mat _roiImg; //roi from the result of last frame
    cv::Mat _roiImg_save;

    cv::Mat _grayImg_of_Enemy; //gray img of roi

    int _trackCnt = 0;
    int _cnt_trackCnt = 0;


    //std::vector<cv::Point2f> point_armor;


    std::vector<ArmorDescriptor> _armors;

    ArmorDescriptor _targetArmor; //relative coordinates
    ArmorDescriptor _FinalArmor;

    int _flag;
    bool _isTracking;

    cv::KalmanFilter KF;
    cv::Mat measurement;

#if defined(DEBUG_DETECTION) || defined(SHOW_RESULT)
    std::string _debugWindowName;
    cv::Mat _debugImg;
#endif // DEBUG_DETECTION || SHOW_RESULT

#ifdef GET_ARMOR_PIC
    int _allCnt = 0;
#endif // GET_ARMOR_PIC

};

}
//class CircleQueue
//{
//public:
//    CircleQueue();
//    ~CircleQueue();
//    void addQue()
//    void delQue();
//    void clearQueue();

//private:
//    int *m_pNEWQueue;//zhizheng
//    int m_iQueueLen;//geshu
//    int *m_pQueue;
//    int *front_p;
//    int *next_p;
//    int m_iQueueCapacity;
//    int anti_armor_angle;

//}
