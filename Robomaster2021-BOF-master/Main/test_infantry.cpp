#include <unistd.h>

#include"iostream"
#include<opencv2/opencv.hpp>
#include<thread>
#include<time.h>

#include"ImgProdCons.h"
#include"./Pose/AngleSolver.hpp"
#include"./Armor/ArmorDetector.h"
#include"./Energy/EnergyDetector.h"


#include <fcntl.h>

#define DEBUG
//#define USING_VIDEO
#define CAMERA_NUMBER 3

//#define serial_time_consumed

using namespace std;
using namespace cv;

static Point2i dev_recode(0.0 , 0.0);

namespace rm
{

void ImgProdCons::init()
{
    //prevent usb port from being blocked
    init_signals();

    //Initialize camera
    _videoCapturePtr->open(0,2);
    _videoCapturePtr->setVideoFormat(640, 480, true);
    _videoCapturePtr->setExposureTime(1,30);//30
    _videoCapturePtr->setFPS(100);
    _videoCapturePtr->startStream();
    _videoCapturePtr->info();

    //Initilize serial
    _serialPtr->openPort();
    _serialPtr->setDebug(true);
    int self_color=RED;
//    while(_serialPtr->setup(self_color) != Serial::OJBK)
//    {
//        sleep(1);
//    }
    cout << "I am " << (self_color == rm::BLUE ? "blue" : "red") << "." << endl;

    //Initialize angle solver
    AngleSolverParam angleParam;
    angleParam.readFile(CAMERA_NUMBER);//choose camera
    _solverPtr->init(angleParam);

    //Initialize armor detector
    ArmorParam armorParam;
    _armorDetectorPtr->init(armorParam);
    _armorDetectorPtr->setEnemyColor(self_color == rm::BLUE ? rm::RED : rm::BLUE);
    _energyDetectorPtr->setEnemyColor(self_color == rm::BLUE ? rm::RED : rm::BLUE);

    //initialize kalmanFilter
    _armorDetectorPtr->kalman_init();

    _task = Serial::AUTO_SHOOT;
}

void ImgProdCons::sense()
{
//    const string fileName = "/home/dji/Documents/Computer_vision/2021_Robomaster_BOF/Robomaster2021-BOF-master/Analyse/Serial_time_consumed.txt";
//    ofstream ofile(fileName);
//    fstream file(fileName, ios::out);  //clean context
    /* Loop for sensing */
    for(;;)
    {
        FeedBackData feedBackData;
        #ifdef serial_time_consumed
        chrono::high_resolution_clock::time_point startTime = chrono::high_resolution_clock::now();
        #endif //serial_time_consumed

        /* TODO: Handel exceptions when socket suddenly being plugged out. */
//        if(_serialPtr->tryFeedBack(feedBackData) == Serial::OJBK)
//        {
        _serialPtr->tryFeedBack(feedBackData);
                _task =      feedBackData.task_mode;
                Yaw_fed =    feedBackData.Yaw_fed /100.0;
                rail_speed = feedBackData.rail_speed / 100.0;//RPM
                rail_speed = 360 * rail_speed / 60 * (CV_PI / 180) * (127.0 / 2);// mm/s   (127*CV_PI)/60*rail_speed
                rail_speed_direction = feedBackData.rail_speed_direction;
            this_thread::sleep_for(chrono::milliseconds(3));//80
//            cout<<"This is sense!!! "<<to_string(_task)<<endl;
//        }
//        else
//        {
//            this_thread::sleep_for(chrono::milliseconds(20));
////            cout<<"There is a congestion of thread of sense!!! "<<to_string(_task)<<endl;
//        }


#ifdef serial_time_consumed
chrono::high_resolution_clock::time_point secondTime = chrono::high_resolution_clock::now();
#endif //serial_time_consumed

//        _serialPtr->feedBack(feedBackData);
//        _task = feedBackData.task_mode;
//        cout<<"Yaw_fed is\t"<<(float)(Yaw_fed)<<endl;
//        cout<<"rail_speed is\t"<<(float)(rail_speed)<<"\t";
//        cout<<"rail_speed_direction is\t"<<(float)(rail_speed_direction)<<endl;






//        chrono::high_resolution_clock::time_point secondTime = chrono::high_resolution_clock::now();
//        ofile<< (static_cast<chrono::duration<double, std::milli>>(secondTime -startTime)).count()<<endl;
#ifdef serial_time_consumed
        cout<<"serial time consumed:"<<to_string((static_cast<chrono::duration<double, std::milli>>(secondTime -startTime)).count())<<endl;
#endif //serial_time_consumed
        //_serialPtr->closePort();
    }

}

void ImgProdCons::consume()
{
    chrono::high_resolution_clock::time_point startTime = chrono::high_resolution_clock::now();
    static float detected_success_time = 0.0;
    static float picture_captured = 0.0;
    static float cnt_frame_rate = 0.0;
    float detector_success_rate = 0.0;

    const string fileName = "/home/dji/Documents/Computer_vision/2021_Robomaster_BOF/Robomaster2021-BOF-master/Analyse/rate.txt";
    ofstream ofile(fileName);
//    fstream file(fileName, ios::out);  //clean context

    /*
     * Variables for serials
     */
    ControlData controlData;
    FeedBackData feedbackdata;



    /*
     *  Variables for angle solve module
     */
    int angleFlag;
    Vec2f targetAngle;

    /* Variables for armor detector module */
    int armorFlag;
    int armorType;
    std::vector<cv::Point2f> armorVertex;
    std::vector<cv::Point2f> energyVertex;
    /*
     *  The main loop for task
     */
    Frame frame;

    for(;;)
    {
#ifndef _MSC_VER
        if (_quit_flag)
            return;
#endif
//        if(_serialPtr->getErrorCode() == Serial::SYSTEM_ERROR || !_videoCapturePtr->isOpened())
//        {
//            this_thread::sleep_for(chrono::seconds(3));
//        }

//        if(_serialPtr->tryFeedBack(feedbackdata) == Serial::OJBK){
//            _task = feedbackdata.task_mode;
//        }
//        cout<<"_task"<<to_string(_task)<<endl;


        _task=Serial::AUTO_SHOOT;
        switch(_task)
        {
        case Serial::NO_TASK:
//            cout<< "manual  "<<to_string(_task) <<endl;
            break;
//        case Serial::SMALL_BUFF:
//            cout<<"small buff"<<endl;
//            break;
        case Serial::ENERGY:
//            cout<<"ENERGY  "<<to_string(_task)<<endl;
            break;
        case Serial::AUTO_SHOOT:
//            cout<<"auto shoot  "<<to_string(_task)<<endl;
            break;
        default:
//            cout<<"unknown mode  "<<to_string(_task)<<endl;
            break;
        }

        if(!_buffer.getLatest(frame))continue;

//        cv::imshow("frame",frame.img);

        if(_task == Serial::ENERGY)
        {

//            cout<<"this is task of ENERGY!!!!"<<endl;

#ifdef USING_VIDEO
           _energyDetectorPtr->detect_video();
#endif

#ifndef USING_VIDEO
            _energyDetectorPtr->loadImg(frame.img);
            if(_energyDetectorPtr->detect()){

            energyVertex = _energyDetectorPtr->getEnergyVertex();
            _solverPtr->setTarget(energyVertex, 2);
            angleFlag = _solverPtr->solve();
            if(angleFlag != rm::AngleSolver::ANGLE_ERROR){
                targetAngle = _solverPtr->getAngle();
                controlData.pitch_dev   = (targetAngle[1]  - 3  - 2.5) * 100;  //3.5
                controlData.yaw_dev     = (targetAngle[0]  + 0.39  ) * 100;

                cout << "Distancs: "<<_solverPtr->getDistance()<<"\t"
                     <<"Deviation: "<< "["<<(float)controlData.pitch_dev/100 << " " << (float)controlData.yaw_dev/100<<"]" << endl;

                  if(_serialPtr->tryControl(controlData, chrono::milliseconds(3)) != Serial::OJBK)
                  {
//                    cout<<"not sent"<<endl;
                  }

            }
            else{
                cout<<"ANGLE_ERROR"<<endl;
                controlData.pitch_dev   = 0;
                controlData.yaw_dev     = 0;
            }
            }
            else{
                controlData.pitch_dev   = 0;
                controlData.yaw_dev     = 0;

                if(_serialPtr->tryControl(controlData, chrono::milliseconds(3)) != Serial::OJBK)
                {
//                  cout<<"not sent"<<endl;
                }
            }
#endif

        }
        else if(_task ==Serial::AUTO_SHOOT)
        {
//            _videoCapturePtr->setExposureTime(1,100);
            //isRecording = false;
            //readyTOrecord = true;

            //_solverPtr->setResolution(Size(640, 480));
#ifdef DEBUG_TIME_CONSUME
            chrono::high_resolution_clock::time_point startTime = chrono::high_resolution_clock::now();
#endif //DEBUG_TIME_CONSUME
            _armorDetectorPtr->loadImg(frame.img);
            armorFlag = _armorDetectorPtr->detect();


#ifdef DEBUG_TIME_CONSUME
            chrono::high_resolution_clock::time_point secondTime = chrono::high_resolution_clock::now();
#endif //DEBUG_TIME_CONSUME

#ifdef ARMOR_SUCCESS_RATE
            if(armorFlag == ArmorDetector::ARMOR_LOCAL) //if detector founded
            {
                detected_success_time += 1.0;
                picture_captured      += 1.0;
            }
            else
            {

                picture_captured      += 1.0;

            }

            detector_success_rate = detected_success_time / picture_captured;

            cout<<"detector_success_rate:   "<<detected_success_time<<"/"<<picture_captured<<"---->"<<detector_success_rate<<endl;
#endif //ARMOR_SUCCESS_RATE

            if(armorFlag == ArmorDetector::ARMOR_LOCAL)
            {
                controlData.gimbal_mode = 1;

                armorVertex = _armorDetectorPtr->getArmorVertex();
                armorType = _armorDetectorPtr->getArmorType();
                //cout<<"armorVertex    "<<armorVertex[0]<<endl;

                _solverPtr->setTarget(armorVertex, armorType);
                angleFlag = _solverPtr->solve();
                if(angleFlag != rm::AngleSolver::ANGLE_ERROR)
                {
                    targetAngle = _solverPtr->getAngle();

                    controlData.frame_seq   = frame.seq;
                    controlData.shoot_mode  = Serial::BURST_FIRE | Serial::LOW_SPEED;
                    controlData.pitch_dev   = targetAngle[1];
                    controlData.yaw_dev     = targetAngle[0];

                    _armorDetectorPtr->armor_rec.push_back(targetAngle);
                    _armorDetectorPtr->armor_lp.push_back(armorVertex[0]);
//                    cout<<"_armorDetectorPtr   "<<_armorDetectorPtr->armor_lp.back()<<endl;

                    double Distance=_solverPtr->getDistance();

                    float angle_compensation_value;
                    if(rail_speed != 0)
                    {
                    float Yaw_feedback;
                    if(rail_speed_direction == 0x00)
                        Yaw_feedback = Yaw_fed * (CV_PI / 180);
                    if(rail_speed_direction == 0x01)
                        Yaw_feedback = CV_PI - Yaw_fed * (CV_PI / 180);

                    angle_compensation_value = asin(sin(Yaw_feedback) * rail_speed /
                                                   (pow(23.5 * 1000,2) + pow(rail_speed,2) - 2*(23.5 * 1000)*rail_speed*cos(Yaw_feedback))
                                                    ) * (180 / CV_PI);

                    if(rail_speed_direction == 0x00)
                        angle_compensation_value = angle_compensation_value;
                    if(rail_speed_direction == 0x01)
                        angle_compensation_value = -angle_compensation_value;
                     cout<<"angle_compensation_value:\t"<<angle_compensation_value<<endl;
                    }
                    else
                        angle_compensation_value =0;


                     if(CAMERA_NUMBER == 3)
                    {
//                        controlData.pitch_dev   = (targetAngle[1] - 0.68);
//                        controlData.yaw_dev     = (targetAngle[0] + 0.69);
                        double angle_compensation_value_2=atan2(45,Distance) * (180 / CV_PI);

                        controlData.pitch_dev   = (targetAngle[1]  - 2 - angle_compensation_value_2) * 100;  //3.5
                        controlData.yaw_dev     = (targetAngle[0]  + 1.2 - angle_compensation_value  ) * 100;

//                        if(Distance<3000)
//                        {
//                            controlData.pitch_dev   = (targetAngle[1]  - 4-angle_compensation_value ) * 100;  //3.5
//                            controlData.yaw_dev     = (targetAngle[0]  + 0.59  ) * 100;

//                        }

                        dev_recode = Point2i(controlData.pitch_dev ,controlData.yaw_dev);
                    }
                    if(CAMERA_NUMBER == 6)
                    {
                        controlData.pitch_dev   = targetAngle[1] - 7.2;
                        controlData.yaw_dev     = targetAngle[0] + 2;
                    }
                    if(CAMERA_NUMBER == 7)
                    {
                        controlData.pitch_dev   = targetAngle[1] - 1.7;
                        controlData.yaw_dev     = targetAngle[0] - 0.9;
                    }

                    if(CAMERA_NUMBER == 8)
                    {
                        controlData.pitch_dev   = targetAngle[1] - 4.81;
                        controlData.yaw_dev     = targetAngle[0] -1.75;
                    }

#ifdef SENTRY
                    controlData.gimbal_mode = Serial::SERVO_MODE;
#endif



#ifdef DEBUG_TIME_CONSUME
            chrono::high_resolution_clock::time_point thirdTime = chrono::high_resolution_clock::now();
#endif //DEBUG_TIME_CONSUME


//                    cout<<"pitch_dev:"<<controlData.pitch_dev<<endl;
//                    cout<<"yaw_dev:"<<controlData.yaw_dev<<endl;
            if(_serialPtr->tryControl(controlData, chrono::milliseconds(3)) != Serial::OJBK)
            {
//              cout<<"not sent"<<endl;
            }


#ifdef DEBUG_TIME_CONSUME
            chrono::high_resolution_clock::time_point forthTime = chrono::high_resolution_clock::now();
#endif //DEBUG_TIME_CONSUME


//                    FeedBackData feedBackData;
//                    _serialPtr->trytoFeedBack(feedBackData);


//                    chrono::duration<double> time_span = chrono::duration_cast<chrono::duration<double, std::milli>>(secondTime -startTime );

#ifdef DEBUG_TIME_CONSUME
//                    cout<<"consume_of_detect:     "<<to_string((static_cast<chrono::duration<double, std::milli>>(secondTime -startTime)).count())<<endl;
//                    cout<<"consume_of_slove_angle:     "<<to_string((static_cast<chrono::duration<double, std::milli>>(thirdTime -secondTime)).count())<<endl;
//                    cout<<"consume_of_tryControl:     "<<to_string((static_cast<chrono::duration<double, std::milli>>(forthTime -thirdTime)).count())<<endl;
                    cout<<"consume_of_consume_thread:     "<<to_string((static_cast<chrono::duration<double, std::milli>>(forthTime -startTime)).count())<<endl;
//                    ofile<<to_string((static_cast<chrono::duration<double, std::milli>>(forthTime -startTime)).count())<<"\n"<<endl;
#endif //DEBUG_TIME_CONSUME


//                    _solverPtr->showAngle();
                    cout << "Distancs: "<<_solverPtr->getDistance()<<"\t"<<"Deviation: " << "["<<(float)controlData.pitch_dev/100.0f << " " << (float)controlData.yaw_dev/100.0f<<"]" << endl;
                }
                 else
                {
                    cout <<"ANGLE_ERROR"<< endl;

                }
            }
            else   //auto shoot but no target
            {
                int Max_Sparkling_Frames = 8;
                if(_armorDetectorPtr->Lost_Track() <= Max_Sparkling_Frames){
                    Vec2f angle;
                     angle  = _armorDetectorPtr->kalman_angle();
                     controlData.pitch_dev   = angle[1];
                     controlData.yaw_dev     = angle[0];
                     controlData.gimbal_mode = 1;
                }
                else if(_armorDetectorPtr->Lost_Track() > Max_Sparkling_Frames){
                    controlData.pitch_dev   = 0;
                    controlData.yaw_dev     = 0;
                    controlData.gimbal_mode = 2;
                }

                if(_serialPtr->tryControl(controlData, chrono::milliseconds(3)) != Serial::OJBK)
                {
//                  cout<<"not sent"<<endl;
                }
//                FeedBackData feedBackData;
//                _serialPtr->trytoFeedBack(feedBackData);
            }
            //cout<<"Lost_Track\t"<<_armorDetectorPtr->Lost_Track()<<endl;

//            cout/*<< "Distancs: "<<_solverPtr->getDistance() <<"  "*/<<"Deviation: " << "["<<(float)controlData.pitch_dev/100.0f << " " << (float)controlData.yaw_dev/100.0f<<"]" << endl;

#ifdef DEBUG_TIME_CONSUME

//            ofile<< (static_cast<chrono::duration<double, std::milli>>(secondTime -startTime)).count()<<"\t\t"<<to_string((int)picture_captured)<<"\t"<<to_string(detector_success_rate * 100)<<"%" <<endl;
//            cout<<to_string(detector_success_rate * 100)<<"%" <<endl;
//            ofile<<to_string(detector_success_rate * 100)<<"\n";
#endif //DEBUG_TIME_CONSUME
            //cout<< detector_success_rate * 100<<endl;
#ifdef SENTRY
            else
            {
                controlData.gimbal_mode = Serial::PATROL_AROUND;
                if(_serialPtr->tryControl(controlData, chrono::milliseconds(3)) != Serial::OJBK)
                {
                    cout<<"not sent"<<endl;
                }

            }
#endif

//            cout << "Deviation: " << targetAngle << endl;



//            if(readyTOrecord)
//            {
//                _videoCapturePtr->setVideoFormat(640, 480, true);
//                _videoCapturePtr->setExposureTime(200);

//                if(isRecording)
//                {
//                    writer << frame.img;
//                }
//                else
//                {
//                    time_t t;
//                    time(&t);
//                    const string fileName = "/home/zhouyu/Documents/file/Robomaster2018-SEU-OpenSource-master/Saved_Video/Autoshoot/" + to_string(t) + ".avi";
//                    writer.open(fileName, CV_FOURCC('M', 'J', 'P', 'G'), 25, Size(frame.img.size().width, frame.img.size().height));
//                    if(!writer.isOpened())
//                    {
//                        cout << "Capture failed." << endl;
//                        continue;
//                    }
//                    isRecording = true;
//                    cout << "Start capture. " + fileName +" created." << endl;
//                }

//            }
        }
        else
        {
            controlData.gimbal_mode = 2;
            controlData.frame_seq   = frame.seq;
            controlData.shoot_mode  = Serial::BURST_FIRE | Serial::LOW_SPEED;
            controlData.pitch_dev   = 0;
            controlData.yaw_dev     = 0;


            if(_serialPtr->tryControl(controlData, chrono::milliseconds(3)) != Serial::OJBK)
            {
//              cout<<"not sent"<<endl;
            }
//            FeedBackData feedBackData;
//            _serialPtr->trytoFeedBack(feedBackData);

            //            if(readyTOrecord)
            //            {
            //                _videoCapturePtr->setVideoFormat(640, 480, true);
            //                _videoCapturePtr->setExposureTime(200);

            //                if(isRecording)
            //                {
            //                    writer << frame.img;
            //                }
            //                else
            //                {
            //                    time_t t;
            //                    time(&t);
            //                    const string fileName = "/home/nvidia/Robomaster/Robomaster2018/" + to_string(t) + ".avi";
            //                    writer.open(fileName, CV_FOURCC('M', 'J', 'P', 'G'), 25, Size(frame.img.size().width, frame.img.size().height));
            //                    if(!writer.isOpened())
            //                    {
            //                        cout << "Capture failed." << endl;
            //                        continue;
            //                    }
            //                    isRecording = true;
            //                    cout << "Start capture. " + fileName +" created." << endl;
            //                }

            //            }
        }
        //auto t1 = chrono::high_resolution_clock::now();


        // auto t2 = chrono::high_resolution_clock::now();

#ifdef DEBUG
        if(waitKey(1) == 'q')
        {
            return;
        }
        //   cout << "Detection duration: " << (static_cast<chrono::duration<double, std::milli>>(t2 - t1)).count() << " ms" << endl;
#endif

    }
}
}
