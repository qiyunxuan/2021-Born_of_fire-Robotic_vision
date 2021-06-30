#include"../General/opencv_extended.h"
#include"../General/numeric_rm.h"
#include<array>
#include<list>

namespace cvex
{

static std::vector<cv::Mat> small;
static std::vector<cv::Mat> big;

using cv::Mat;
using cv::MatND;
using cv::Rect;
using cv::RotatedRect;
using cv::Vec4i;
using cv::Scalar;
using cv::Point;

using cv::imshow;
using cv::waitKey;

using std::vector;

void showHist(const Mat img)
{
	Mat hist;
	int histSize = 256;
	cv::calcHist(&img, 1, 0, Mat(), hist, 1, &histSize, 0);

	Mat histImage = Mat::ones(200, 320, CV_8U) * 255;

	normalize(hist, hist, 0, histImage.rows, CV_MINMAX, CV_32F);

	histImage = Scalar::all(255);
	int binW = cvRound((double)histImage.cols / histSize);

	for(int i = 0; i < histSize; i++)
		rectangle(histImage, Point(i*binW, histImage.rows),
			Point((i + 1)*binW, histImage.rows - cvRound(hist.at<float>(i))),
			Scalar::all(0), -1, 8, 0);
	imshow("histogram", histImage);
	waitKey();
}

void rotatedRectangle(Mat& img, const RotatedRect& rec, const Scalar & color)
{
	if (&rec == nullptr) return;
	cv::Point2f rect_points[4];
	rec.points(rect_points);
	for (int j = 0; j < 4; j++)
	{
		line(img, rect_points[j], rect_points[(j + 1) % 4], color);
	}
}

void GammaCorrection(cv::Mat& src, cv::Mat& dst, float fGamma)
{
    CV_Assert(src.data);
    // accept only char type matrices
    CV_Assert(src.depth() != sizeof(uchar));

    // build look up table
    unsigned char lut[256];
    for( int i = 0; i < 256; i++ )
    {
        lut[i] = cv::saturate_cast<uchar>(pow((float)(i/255.0), fGamma) * 255.0f);
    }

    dst = src.clone();
    const int channels = dst.channels();
    switch(channels)
    {
        case 1:
            {

                cv::MatIterator_<uchar> it, end;
                for( it = dst.begin<uchar>(), end = dst.end<uchar>(); it != end; it++ )
                    //*it = pow((float)(((*it))/255.0), fGamma) * 255.0;
                    *it = lut[(*it)];

                break;
            }
        case 3:
            {

                cv::MatIterator_<cv::Vec3b> it, end;
                for( it = dst.begin<cv::Vec3b>(), end = dst.end<cv::Vec3b>(); it != end; it++ )
                {
                    //(*it)[0] = pow((float)(((*it)[0])/255.0), fGamma) * 255.0;
                    //(*it)[1] = pow((float)(((*it)[1])/255.0), fGamma) * 255.0;
                    //(*it)[2] = pow((float)(((*it)[2])/255.0), fGamma) * 255.0;
                    (*it)[0] = lut[((*it)[0])];
                    (*it)[1] = lut[((*it)[1])];
                    (*it)[2] = lut[((*it)[2])];
                }

                break;

            }
    }
}


void Adjust_Brightness(cv::Mat& src, cv::Mat& dst,float light_Brightness)
{
    dst = src.clone();
    int rows = src.rows;
    int cols = src.cols;
    float alpha = 1.0;
//    double beta = -50.0;

    for (int r1= 0; r1 < rows; r1++) {
        for (int c = 0; c < cols; c++) {
            if (src.channels() == 3) {
                double b = src.at<cv::Vec3b>(r1, c)[0];
                double g = src.at<cv::Vec3b>(r1, c)[1];
                double r = src.at<cv::Vec3b>(r1, c)[2];

                dst.at<cv::Vec3b>(r1, c)[0] = cv::saturate_cast<uchar>(b*alpha + light_Brightness);
                dst.at<cv::Vec3b>(r1, c)[1] = cv::saturate_cast<uchar>(g*alpha + light_Brightness);
                dst.at<cv::Vec3b>(r1, c)[2] = cv::saturate_cast<uchar>(r*alpha + light_Brightness);
            }
            else {
                if (src.channels() == 1) {
                    double v = src.at<cv::Vec3b>(rows, cols)[0];
                    dst.at<uchar>(r1, c) = cv::saturate_cast<uchar>(v*alpha + light_Brightness);

                }
            }
        }
    }

}



void get_Template()
{

//    const string fileName = "/home/dji/Documents/Computer_vision/2021_Robomaster_BOF/Robomaster2021-BOF-master/Analyse/Template_check.txt";
//    ofstream ofile(fileName);


    std::string Template_Location = "/home/dji/Documents/Computer_vision/2021_Robomaster_BOF/Robomaster2021-BOF-master/Template/";

//    std::vector<cv::String> image_files;
//    cv::glob(Template_Location,image_files);
    int array[16] = {1,2,3,4,5,6,7,8,11,22,33,44,55,66,77,88};
    for(int i =0;i<8;i++)
    {
        cv::Mat temp = cv::imread(Template_Location + std::to_string(array[i])+".jpg",0);
        small.emplace_back(temp);
        big.emplace_back(temp);
//        cv::imshow("check",small[i]);
//        cv::waitKey(300);
//        a.push_back(i);
    }
    //imshow("check",small[0]);

}

//void video_write()
//{
//    const string fileName = "/home/dji/Documents/Computer_vision/2021_Robomaster_BOF/Robomaster2021-BOF-master/Analyse/pre_treatment_consume.txt";
//    ofstream ofile(fileName);
//    fstream file(fileName, ios::out);  //clean context

//}


}
