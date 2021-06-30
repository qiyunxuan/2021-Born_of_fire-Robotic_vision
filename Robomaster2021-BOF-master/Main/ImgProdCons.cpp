#include "ImgProdCons.h"

#include <signal.h>

#include<opencv2/opencv.hpp>
#include<chrono>
#include<iostream>
#include<thread>
#include<memory>

#define DEBUG

using namespace std;
using namespace cv;

namespace rm
{

FrameBuffer::FrameBuffer(size_t size):
    _frames(size),
    _mutexs(size),
    _tailIdx(0),
    _headIdx(0),
    _lastGetTimeStamp(0.0)
{

}

bool FrameBuffer::push(const Frame& frame)
{
    const size_t newHeadIdx = (_headIdx + 1) % _frames.size();

    //try for 2ms to lock
    unique_lock<timed_mutex> lock(_mutexs[newHeadIdx],chrono::milliseconds(2));
    if(!lock.owns_lock())
    {
        return false;
    }

    _frames[newHeadIdx] = frame;
    if(newHeadIdx == _tailIdx)
    {
        _tailIdx = (_tailIdx + 1) % _frames.size();
    }
    _headIdx = newHeadIdx;
    return true;
}

bool FrameBuffer::getLatest(Frame& frame)
{
    volatile const size_t headIdx = _headIdx;

    //try for 2ms to lock
    unique_lock<timed_mutex> lock(_mutexs[headIdx],chrono::milliseconds(2));
    if(!lock.owns_lock() ||
       _frames[headIdx].img.empty() ||
       _frames[headIdx].timeStamp == _lastGetTimeStamp)
    {
        return false;
    }

    frame = _frames[headIdx];
    _lastGetTimeStamp = _frames[headIdx].timeStamp;

    return true;
}



ImgProdCons::ImgProdCons():
    _videoCapturePtr(make_unique<RMVideoCapture>()),
    _buffer(6),
    _serialPtr(make_unique<Serial>()),
    _solverPtr(make_unique<AngleSolver>()),
    _armorDetectorPtr(make_unique<ArmorDetector>()),
    _energyDetectorPtr(make_unique<EnergyDetector>()),
    _task(Serial::NO_TASK)
{}

void ImgProdCons::signal_handler(int)
{
    _quit_flag = true;
}
void ImgProdCons::init_signals(void)
{
    _quit_flag = false;
    struct sigaction sigact;
    sigact.sa_handler = signal_handler;
    sigemptyset(&sigact.sa_mask);
    sigact.sa_flags = 0;
    sigaction(SIGINT,&sigact,(struct sigaction*)NULL);
}

void ImgProdCons::produce()
{
//    const string fileName = "/home/dji/Documents/Computer_vision/2021_Robomaster_BOF/Robomaster2021-BOF-master/Analyse/produce.txt";
//    ofstream ofile(fileName);
//    fstream file(fileName, ios::out);  //clean context

    auto startTime = chrono::high_resolution_clock::now();

//    chrono::high_resolution_clock::time_point t1 = chrono::high_resolution_clock::now();
    for(;;)
    {
//        auto one = chrono::high_resolution_clock::now();
        /*
         * Can prevent the camera usb port from being blocked as possible.
         */
//        cout<<"the state of _quit_flag: "<<to_string(_quit_flag)<<endl;
//        cout<<"\tstart"<<endl;
        if (_quit_flag) return;
//        cout<<"\end"<<endl;
        /*
         * Decoding costs lots of time, so let the STM record after grab,
         * and then decode by calling 'retrieve'.
         */
        //cout<<"the state of grab: "<<to_string(!_videoCapturePtr->grab())<<endl;

        if(!_videoCapturePtr->grab()) continue;

        /*
         * Every new Mat has a new address.
         * Let the sequence number of image frame and serial frame be the same.
         */
        Mat newImg;
        uint8_t seq =0;
        double timeStamp = (static_cast<chrono::duration<double,std::milli>>(chrono::high_resolution_clock::now() - startTime)).count();

        /*
         * Tell the STM to record.
         */
//        seq++;
//        if(_serialPtr->tryRecord(seq, chrono::milliseconds(3)) != Serial::OJBK)
//        {
//            continue;
//        }

        /*
         * Decode image
         */
        if(!_videoCapturePtr->retrieve(newImg))
        {
//            cout<<"retrieve   failed!!!: "<<endl;
            continue;
        }

//        cv::imshow("newImg",newImg);
//        auto two = chrono::high_resolution_clock::now();
        /*
         * push the new image into the circular buffer
         */
//        cv::imshow("newImg",newImg);
        _buffer.push(Frame{newImg, seq, timeStamp});


        chrono::high_resolution_clock::time_point t2 = chrono::high_resolution_clock::now();
//        cout << "Capture period: " << (static_cast<chrono::duration<double, std::milli>>(t2 - t1)).count() << " ms" << endl;
//        cout << endl;
//        ofile<< (static_cast<chrono::duration<double, std::milli>>(t2 - startTime)).count() <<"   \n"<<endl;
        startTime = t2;

//        cout<<"consume_of_produce:     "<<to_string((static_cast<chrono::duration<double, std::milli>>(two -one)).count())<<endl;
    }
}


void ImgProdCons::Save_video()
{
    chrono::high_resolution_clock::time_point startTime = chrono::high_resolution_clock::now();
    int Frame_rate;

    const string fileName = "/home/bof/Documents/CV/Saved_Video/raw.txt";
    ofstream ofile(fileName);
    fstream file(fileName, ios::out);  //clean context

    VideoWriter writer;
    bool isRecording = false;
    bool readyTOrecord = true;
    Frame frame;
    for(;;)
    {
#ifndef _MSC_VER
        if (_quit_flag)
            return;
#endif
        if(!_buffer.getLatest(frame)) continue;
        if(readyTOrecord)
        {
            if(isRecording)
            {
                writer << frame.img;
                chrono::high_resolution_clock::time_point secondTime = chrono::high_resolution_clock::now();

                ofile<< (static_cast<chrono::duration<double, std::milli>>(secondTime - startTime)).count() <<"   ms\n"<<endl;
                startTime = secondTime;


//                std::cout << "It took me " << time_span.count() << " seconds.";
//                std::cout << std::endl;
#ifdef DEBUG
        if(waitKey(1) == 'p')
        {
            return;
        }
#endif
            }

            else
            {
                time_t t;
                time(&t);
                Frame_rate = 100;
                const string fileName = "/home/bof/Documents/CV/Saved_Video/" + to_string(t) + ".avi";
                writer.open(fileName, CV_FOURCC('M', 'J', 'P', 'G'), Frame_rate, Size(frame.img.size().width, frame.img.size().height));

                if(!writer.isOpened())
                {
                    cout << "SavevideoThread:Capture failed." << endl;
                    continue;
                }
                isRecording = true;
                cout << "SavevideoThread:Start capture. " + fileName +" created." << endl;
            }
        }
    }
}
}
