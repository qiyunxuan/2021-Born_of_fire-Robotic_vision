#include"EnergyDetector.h"
#include<iostream>
#include<vector>
#include<math.h>
#include<string>
#include<chrono>

#include"../General/opencv_extended.h"
#include"../General/numeric_rm.h"

//#define HSV //是否使用HSV通道筛选颜色

namespace rm
{



const std::vector<cv::Point2f> EnergyDetector::getEnergyVertex() const
{
    vector<cv::Point2f> realVertex;
//    Point2f last_armor_vertex[4];
//    _predicted_armors_vertex.points(last_armor_vertex);
    for(int i=0;i <4;i++)
    {
        realVertex.emplace_back(Point2f(_predicted_armors_vertex[i].x,
                                        _predicted_armors_vertex[i].y));

    }
    return realVertex;
}



int EnergyDetector::markred(Mat roiImg)
{
    vector<RotatedRect> _armors;
//    imshow("roiImg!", roiImg);
    Mat Img;

    _armors.clear();
#ifdef HSV
    Mat hsvImage,dstImage1, dstImage2;
    cvtColor(srcImage, hsvImage, COLOR_BGR2HSV);//转换为HSV图
    inRange(hsvImage, Scalar(156, 43, 46), Scalar(180, 255, 255), dstImage1);//二值化图像，阈值为红色域
    inRange(hsvImage, Scalar(0, 43, 46), Scalar(10, 255, 255), dstImage2);//二值化图像，阈值为红色域
    add(dstImage1, dstImage2, Img);
    imshow("二值化图像",Img);
#endif

#ifndef HSV

    std::vector<Mat> channels;
    // 把一个3通道图像转换成3个单通道图像
    split(roiImg,channels);//分离色彩通道

    //预处理删除己方装甲板颜色

    cv::Mat _grayImg,binGrayImg;;

    cv::cvtColor(_roiImg, _grayImg, CV_BGR2GRAY);
    cv::threshold(_grayImg         , binGrayImg  , 81, 255, cv::THRESH_BINARY);

    if(_enemy_color==RED)
         Img=channels.at(2)-channels.at(0);//Get red-blue image;
    else Img=channels.at(0)-channels.at(2);//Get blue-red image;
    cv::threshold(Img, Img, 100, 255, cv::THRESH_BINARY);//110 for video

//    imshow("_grayImg!", binGrayImg);
//    imshow("_subImg!", Img);

    cv::Mat element_d = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 , 2));
    dilate(Img, Img, element_d);
//    imshow("_dilate_subImg!", Img);



    Img &= binGrayImg;
//    imshow("add!", Img);

#endif


    Mat dstImage;
    Mat element1 = getStructuringElement(MORPH_RECT,Size(3, 3));
    morphologyEx(Img, dstImage, MORPH_DILATE, element1);
//    Mat element2 = getStructuringElement(MORPH_RECT,Size(3, 3));
//    morphologyEx(dstImage, dstImage, MORPH_ERODE, element2);
//    imshow("膨胀和腐蚀",dstImage);

    vector<vector<Point>>contours;//轮廓数组
    vector<Vec4i>hierarchy; //一个参数
    Point2i armor_center; //用来存放找到的目标的中心坐标

    //提取所有轮廓并建立网状轮廓结构
    findContours(dstImage, contours, hierarchy, RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

    int contour[20] = { 0 };
    int contour_center[20] = { 0 };
    int test_contour_center = 0;
    int test_contour = 0;
      for (int i = 0; i < contours.size(); i++)//遍历检测的所有轮廓
      {
        if (hierarchy[i][3] != -1) //有内嵌轮廓，说明是一个父轮廓
        {
          contour[hierarchy[i][3]]++; //对该父轮廓进行记录
        }
        if(hierarchy[i][2] == -1) {
          contour_center[i]++;//直接识别风车中心R为轮廓中心
        }
      }

      for(int i =0;i<20;i++) test_contour_center += contour_center[i];
      for(int i =0;i<20;i++) test_contour        += contour[i];
      if(test_contour_center == 0){
          cout<<"ERROR!!! There is no center of Windmill!!!"<<endl;
          center_of_Windmill = Point2i(0,0);
          return NOT_FOUND;
      }

      if(test_contour == 0){
          cout<<"ERROR!!! There is no Windmill!!!"<<endl;
          center_of_Windmill = Point2i(0,0);
          return NOT_FOUND;
      }

//      cout<<"contours.size()  "<<contours.size()<<"\n"<<endl;

      for (int j = 0; j < contours.size(); j++)//再次遍历所有轮廓
      {
        if(contour_center[j] == 1)
        {
          double area = contourArea(contours[j]);//计算轮廓面积
//          cout<<"possible area of Energy \t"<<area<<endl;
          if(area < 80 || area > 200) continue;




//          if(area > 500 ||area <100) continue;
          RotatedRect box = minAreaRect(contours[j]); //包含该轮廓所有点
          Point2f vertex[4];
          box.points(vertex);//将左下角，左上角，右上角，右下角存入点集
          for (int i = 0; i < 4; i++)
          {
            line(_roiImg, vertex[i], vertex[(i + 1) % 4], Scalar(255,100, 0), 2, LINE_AA); //画线
          }
//          putText(srcImage, "center_of_Windmill", vertex[0], FONT_HERSHEY_SIMPLEX, 1.0, Scalar(255, 255, 255));//打印字体
          center_of_Windmill = box.center;
//          cout<<"center_of_Windmill"<<center_of_Windmill<<endl;
        }

        if (contour[j] == 1) //如果某轮廓对应数组的值为1，说明只要一个内嵌轮廓
        {
          int num = hierarchy[j][2]; //记录该轮廓的内嵌轮廓
          double area = contourArea(contours[num]);//计算轮廓面积
//          cout<<"area of armor \t"<<area<<endl;
          if(area < 250 || area > 600)   //可疑装甲板面积
              continue;
          RotatedRect box = minAreaRect(contours[num]); //包含该轮廓所有点
          Point2f vertex[4];
          box.points(vertex);//将左下角，左上角，右上角，右下角存入点集
//          for (int i = 0; i < 4; i++)
//          {
//            line(_roiImg, vertex[i], vertex[(i + 1) % 4], Scalar(255, 0, 0), 2, LINE_AA); //画线
//          }

          double _ratio;    //可疑装甲板宽高比
          if(box.size.width > box.size.height)
              _ratio = box.size.width/box.size.height;
          else
              _ratio = box.size.height/box.size.width;

          if((_ratio > 1.25) && (_ratio < 1.75))
            _armors.emplace_back(box);
//          armor_center = (vertex[0] + vertex[2]) / 2; //返回中心坐标
        }
      }

      //选出最终装甲板
      /******************************************************************************************/

      Point2f _armors_vertex[4];
      if(_armors.size() > 1){
          cout<<"There are more than one candidate armor!!!"<<_armors.size()<<endl;
          sort(_armors.begin(), _armors.end(),  [](const RotatedRect& am1, const RotatedRect& am2)
          {
              return am1.size.area() > am2.size.area();
          });
      }
      else if(_armors.size() == 0){
//          _armors.emplace_back(last_armor);
          center_of_Windmill = Point2i(0,0);
          return NOT_FOUND;
      }
//      last_armor = _armors[0];

      _armors[0].points(_armors_vertex);
      center_of_Aromr = (_armors_vertex[0] + _armors_vertex[2]) / 2;

//      putText(_roiImg, "hit it!!!", _armors_vertex[0], FONT_HERSHEY_SIMPLEX, 1.0, Scalar(255, 255, 255));//打印字体
//      cout<<"_armors.size()   "<<_armors.size()<<endl;

      if(center_of_Windmill.x * center_of_Windmill.y != 0)  //预测击打点
      {
          for(int i =0;i<4;i++){
              float xx=(_armors_vertex[i].x-center_of_Windmill.x);
              float yy=(_armors_vertex[i].y-center_of_Windmill.y);
              radius = sqrt(pow(xx,2) + pow(yy,2));
              Point2f resPoint=Point2f(_roi.tl().x + center_of_Windmill.x+xx*cos(W)+yy*sin(W),
                                       _roi.tl().y + center_of_Windmill.y-xx*sin(W)+yy*cos(W));
              _predicted_armors_vertex[i] = resPoint;
//              _predicted_armors_vertex[i] = _armors_vertex[i];
          }
          for (int i = 0; i < 4; i++)
          {
            line(_roiImg, _predicted_armors_vertex[i], _predicted_armors_vertex[(i + 1) % 4], Scalar(255, 111, 111), 2, LINE_AA); //画线
          }
      }
      /******************************************************************************************/
      return FOUND;
}

Point2i EnergyDetector::Windwill_pred(Point2i armor_center) //返回能量机关预测击打点
{
    if(center_of_Windmill.x * center_of_Windmill.y != 0)  //预测击打点
    {
        float xx=(armor_center.x-center_of_Windmill.x);
        float yy=(armor_center.y-center_of_Windmill.y);
        radius = sqrt(pow(xx,2) + pow(yy,2));
        Point2f resPoint=Point2f(center_of_Windmill.x+xx*cos(W)+yy*sin(W),
                                 center_of_Windmill.y-xx*sin(W)+yy*cos(W));
        circle(_roiImg,resPoint,1,Scalar(0,255,0),2);
//        putText(_roiImg, "  predicted_point!!!", resPoint, FONT_HERSHEY_PLAIN, 1.0, Scalar(255, 255, 255));//打印字体

//        Point2f Final_angle = diff_angle(resPoint);
//        cout<<"Final_angle  "<<Final_angle<<endl;
//        circle(srcImage,Final_angle,1,Scalar(255,255,255),10);
//        putText(srcImage, "  final_point!!!", Final_angle, FONT_HERSHEY_PLAIN, 1.0, Scalar(255, 255, 255));//打印字体

    }
}

Point2f EnergyDetector::diff_angle(Point2f resPoint)
{
    //要用PNP算法得知相关数据
    theta = atan((dist * tan(alpha * (CV_PI/180.0) - H )/ dist   ));   //Pitch     角度转弧度
    delta = atan( dist * tan(beta  * (CV_PI/180.0)      /(dist+D)));   //Yaw       角度转弧度
    return  Point2f(180 / CV_PI * delta,180 / CV_PI * theta);             //yuntai弧度转角度

}


int EnergyDetector::detect()
{
    int state_of_markred = 0;

    state_of_markred = markred(_roiImg);  //自定义函数进行识别

//    cout<<"state_of_markred:\t"<<state_of_markred<<endl;
//    cout<<"state_of_markred:\t"<<endl;

    if(state_of_markred == NOT_FOUND) return state_of_markred;

    center_of_Windmill = Point2i(_roi.tl().x + center_of_Windmill.x,_roi.tl().y + center_of_Windmill.y);
    center_of_Aromr = Point2i(_roi.tl().x + center_of_Aromr.x,_roi.tl().y + center_of_Aromr.y);

    all_center.push_back(center_of_Aromr);
//    cout<<"center_of_Aromr:\t"<<center_of_Aromr<<endl;
//    //circle(srcImage,center,1,Scalar(255,255,0),3);//标记该击打的装甲板(青色)
//    Windwill_pred(center_of_Aromr);

    imshow("效果图", _roiImg);
    return 1;
}

int EnergyDetector::detect_video()
{
    VideoCapture capture; //读入视频
    capture.open("/home/dji/Documents/Computer_vision/test/Windmill_new/energy_machinery/low_exposure_RED.mp4");

    if(!capture.isOpened())
    {
      printf("could not load vodeo successfully");
       return -1;
    }

    Point2i center; //定义矩形中心

    int    frames = capture.get(CAP_PROP_FRAME_COUNT);//zhenshu
    double fps = capture.get(CAP_PROP_FPS);
  //  double timeStamp = 0.0;

    Size size = Size(capture.get(CAP_PROP_FRAME_WIDTH),capture.get(CAP_PROP_FRAME_HEIGHT));
    Mat draw = Mat(size, CV_8UC3,Scalar(0, 0, 0));

    cout<<"frames  :"<< frames <<"\n"
        <<"fps     :"<< to_string(fps)<<"\n"
        <<"size    :"<< size <<"\n"
        <<endl;

    all_center.clear();

    center_of_Windmill = Point2i(0,0);

    while (1)
    {

      capture >> srcImage;//读入帧


  //    Mat _roiImg;
      Rect imgBound = Rect(cv::Point(0, 0), srcImage.size());
      Rect _roi;

//      if(center_of_Windmill.x * center_of_Windmill.y != 0)
//      {
//  //        record_all_centerpiece();
//          int i =65;
//          _roi = Rect(center_of_Windmill.x-radius-i,center_of_Windmill.y-radius-i ,(radius+i)*2,(radius+i)*2);
//          _roi = _roi & imgBound;
//          _roiImg = srcImage(_roi).clone();
//          rectangle(srcImage, _roi.tl(), _roi.br(), Scalar(255,255,0),2);//随机颜色
//      }
//      else{
          _roi = imgBound;
          _roiImg = srcImage.clone();
//      }

      markred(_roiImg);  //自定义函数进行识别
      center_of_Windmill = Point2i(_roi.tl().x + center_of_Windmill.x,_roi.tl().y + center_of_Windmill.y);
      center_of_Aromr = Point2i(_roi.tl().x + center_of_Aromr.x,_roi.tl().y + center_of_Aromr.y);
      all_center.push_back(center_of_Aromr);

      //circle(srcImage,center,1,Scalar(255,255,0),3);//标记该击打的装甲板(青色)
      Windwill_pred(center_of_Aromr);

  //记录所有装甲板中心像素点
  /******************************************************************************************/
  //    if(all_center.size() >1)
  //    {
  //        for(int i =1;i<all_center.size();i++)
  //        {
  //            line(srcImage,all_center[i-1],all_center[i],Scalar(255,255,0),5,CV_AA);
  //            circle(draw,all_center[i],1,Scalar(255,255,0),1);
  //            cout << "all_center[i]" << all_center[i] << "\n" << endl;
  //        }
  //    }
  /******************************************************************************************/

  //    imshow("draw", draw);
      imshow("效果图", srcImage);

      if (waitKey(30) >= 0) //按任意键退出
        break;
    }
    return 0;
}

void EnergyDetector::setEnemyColor(int enemy_color)
{
    _enemy_color = enemy_color;
    _self_color = enemy_color == BLUE ? RED : BLUE;
}

void EnergyDetector::loadImg(const cv::Mat & srcImg)
{
    srcImage = srcImg;

    Rect imgBound = Rect(cv::Point(0, 0), srcImage.size());


    line(srcImage,Point(srcImage.size().width/2-5,srcImage.size().height/2),
                 Point(srcImage.size().width/2+5,srcImage.size().height/2),
                                                             cvex::WHITE);
    line(srcImage,Point(srcImage.size().width/2,srcImage.size().height/2-5),
                 Point(srcImage.size().width/2,srcImage.size().height/2+5),
                                                             cvex::WHITE);


//    if(center_of_Windmill.x * center_of_Windmill.y != 0)
//    {
////        record_all_centerpiece();
//        int i =35;
//        _roi = Rect(center_of_Windmill.x-radius-i,center_of_Windmill.y-radius-i ,(radius+i)*2,(radius+i)*2);
//        _roi = _roi & imgBound;
//        _roiImg = srcImage(_roi).clone();
//        rectangle(srcImage, _roi.tl(), _roi.br(), Scalar(255,255,0),1);//随机颜色
//    }
//    else{
        _roi = imgBound;
        _roiImg = srcImage.clone();
//    }

    //cout<<"_trackCnt:       "<<std::to_string(_trackCnt)<<endl;

#ifdef DEBUG_DETECTION
    rectangle(_debugImg, _roi, cvex::YELLOW);
#endif // DEBUG_DETECTION
}

}
