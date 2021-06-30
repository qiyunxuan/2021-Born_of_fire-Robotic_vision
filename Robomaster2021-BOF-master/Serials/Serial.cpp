
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
Authors:    Binyan Hu
**************************************************************/
#include "Serial.h"

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>

#include <iostream>
#include <fstream>
#include <string>
#include<string.h>
#include <stdexcept>
#include<exception>

#include "../General/General.h"

using namespace std;

namespace rm
{
Serial::Serial():
    //_serialFd(1),
    _serialFd(-1),
    _errorCode(OJBK),
    _en_debug(false),
    _lastRecordSeq(0)
{
//    static_assert(sizeof(ControlFrame) == 16, "Size of backdata is not 16");
//    static_assert(sizeof(FeedBackFrame) == 20, "Size of backdata is not be 20");

    // 初始化通讯帧结构体通用项
    _controlFrame.SOF = JetsonCommSOF;

    _controlFrame.frame_seq = 0;
    _controlFrame.shoot_mode = NO_FIRE |HIGH_SPEED;
    _controlFrame.pitch_dev = 0;
    _controlFrame.yaw_dev = 0;
    _controlFrame.gimbal_mode = 0;
    _controlFrame.rail_speed = 0;

    _controlFrame.EOF = JetsonCommEOF;
}

Serial::~Serial()
{
    tcflush(_serialFd, TCIOFLUSH);
    if (-1 == close(_serialFd))
	{
        _errorCode = SYSTEM_ERROR;
        cout << "Serial closing  failed." << endl;
    }
    else
    {
        _errorCode = OJBK;
    }
}

int Serial::openPort()
{
    _serialFd = open("/dev/CH340", O_RDWR | O_NOCTTY );
    if (_serialFd == -1)
    {        
        cout << "Open serial port failed." << endl;
        //ofile<< "Open serial port failed." <<"   \n"<<endl;
        return _errorCode = SYSTEM_ERROR;
    }
 //BaudRate:460800     Data bits:8   Parity:    Stop bits:1
    termios tOption;                                //串口配置结构体
    tcgetattr(_serialFd, &tOption);                 //获取当前设置
    cfmakeraw(&tOption);
    cfsetispeed(&tOption, B460800);                 // 接收波特率
    cfsetospeed(&tOption, B460800);                 // 发送波特率
    tcsetattr(_serialFd, TCSANOW, &tOption);
    tOption.c_cflag &= ~PARENB;                      //Clear parity enable 清除奇偶校验启用
    tOption.c_cflag &= ~CSTOPB;                      //不指明表示一位停止位
    tOption.c_cflag &= ~CSIZE;                       //这是设置c_cflag选项不按位数据位掩码
    tOption.c_cflag |= CS8;                          //设置c_cflag选项数据位为8位
    tOption.c_cflag &= ~INPCK;                       //Enable parity checking  启用奇偶校验
    tOption.c_cflag |= (B460800 | CLOCAL | CREAD);  // 设置波特率，本地连接，接收使能
    tOption.c_cflag &= ~(INLCR | ICRNL);
    tOption.c_cflag &= ~(IXON);
    tOption.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  //切换到行方式输入(不需要回车和换行发送)
    tOption.c_oflag &= ~OPOST;
    tOption.c_oflag &= ~(ONLCR | OCRNL);               //屏蔽: 回车和换行看成一个字符
    tOption.c_iflag &= ~(ICRNL | INLCR);               //屏蔽: 回车和换行看成一个字符
    tOption.c_iflag &= ~(IXON | IXOFF | IXANY);       //ignore some of restriction
    tOption.c_cc[VTIME] = 1; //1                       //只有设置为阻塞时这两个参数才有效
    tOption.c_cc[VMIN] = 1; //1
    tcflush(_serialFd, TCIOFLUSH);                  //TCIOFLUSH刷新输入、输出队列。

    cout << "Serial preparation complete." << endl;
    return _errorCode = OJBK;
}

void Serial::openMulTime()
{
    _serialFd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
}

void Serial::getState()
{
    cout<<"the state of _serialFd:      "<<to_string(_serialFd)<<endl;
}

int Serial::closePort()
{
    tcflush(_serialFd, TCIOFLUSH);
    if (-1 == close(_serialFd))
    {
        _errorCode = SYSTEM_ERROR;
        cout << "Serial closing failed." << endl;
    }
    else
    {
        _errorCode = OJBK;
    }
    return _errorCode;
}

bool Serial::isOpened() const
{
    return (_serialFd != -1);
}

void Serial::setDebug(bool en_debug)
{
    _en_debug = en_debug;
}

int Serial::setup(int& self_color)
{
    if(_en_debug)
    {
        cout << "[setup]\n";
    }

//    u_int16_t tmp_self_team;

//    _controlFrame.shoot_mode = SET_UP;
//    if(send() == OJBK)
//    {
//        if(receive() == OJBK)
//        {
//            tmp_self_team = ((_feedBackFrame.task_mode << 8) | _feedBackFrame.bullet_speed);
//            if(tmp_self_team != BLUE_TEAM && tmp_self_team != RED_TEAM)
//            {
//                return _errorCode = CORRUPTED_FRAME;
//            }

//            _controlFrame.shoot_mode = tmp_self_team;
//            if(send() == OJBK)
//            {
//                if(tmp_self_team == BLUE_TEAM)
//                {
//                    self_color = rm::BLUE;
//                }
//                else if(tmp_self_team == RED_TEAM)
//                {
//                    self_color = rm::RED;
//                }
//            }
//        }
//    }
    return _errorCode;
}

int Serial::record(u_int8_t& frame_seq)
{
    if(_en_debug)
    {
        cout << "[record]\n";
    }

    _lastRecordSeq++;
    _controlFrame.frame_seq = _lastRecordSeq;
    _controlFrame.shoot_mode = RECORD_ANGLE;
    _controlFrame.pitch_dev = 0;
    _controlFrame.yaw_dev = 0;
    frame_seq = _lastRecordSeq;
    return send();
}

int Serial::control(const ControlData& controlData)
{
    if(_en_debug)
    {
//        cout << "[control]\n";
    }
    _controlFrame = pack(controlData);
    return send();
}

int Serial::feedBack(FeedBackData& feedBackData)
{
    if(_en_debug)
    {
//        cout << "[request]\n";
    }

//    _controlFrame.shoot_mode = REQUEST_TRANS;
//    if(send() == OJBK)
//    {
        if (receive() == OJBK)
        {
            feedBackData = unpack(_feedBackFrame);

            return OJBK;
        }
//    }
    return _errorCode;
}

int Serial::getErrorCode() const
{
    return _errorCode;
}


void Serial::print(const ControlFrame &ct)
{
    cout<<"ControlFrame:"<<endl;
    cout<<hex<<(unsigned int)ct.SOF<<endl;
    cout<<dec<<(unsigned int)ct.frame_seq<<endl;
    cout<<hex<<(unsigned int)ct.shoot_mode<<endl;
    cout<<dec<<              ct.pitch_dev<<endl;
    cout<<dec<<              ct.yaw_dev<<endl;
    cout<<dec<<         (int)ct.rail_speed<<endl;
    cout<<hex<<(unsigned int)ct.gimbal_mode<<endl;
    cout<<hex<<(unsigned int)ct.EOF<<endl;
}

void Serial::print(const FeedBackFrame &fb)
{
    cout<<"FeedBackFrame:"<<endl;
//    cout<<hex<<(unsigned int)fb.SOF<<endl;
//    cout<<dec<<(unsigned int)fb.frame_seq<<endl;
    cout<<hex<<(unsigned int)fb.task_mode<<endl;
//    cout<<dec<<(unsigned int)fb.bullet_speed<<endl;
//    cout<<dec<<(unsigned int)fb.rail_pos<<endl;
//    cout<<dec<<(unsigned int)fb.shot_armor<<endl;
//    cout<<dec<<(unsigned int)fb.remain_HP<<endl;
//    for(int i = 0;i<11;i++)
//    {
//        cout<<dec<<(unsigned int)(fb.reserved[i])<<", ";
//    }
    cout<<endl;
//    cout<<hex<<(unsigned int)fb.EOF<<endl;
}

Serial::ControlFrame Serial::pack(const ControlData& ctrl)
{
    return ControlFrame
    {
        JetsonCommSOF,
        ctrl.frame_seq,
        ctrl.shoot_mode,
        ctrl.pitch_dev,
        ctrl.yaw_dev,
        ctrl.rail_speed,
        ctrl.gimbal_mode,
        JetsonCommEOF
    };
}

FeedBackData Serial::unpack(const Serial::FeedBackFrame& fb)
{
    return FeedBackData
    {
        fb.task_mode,
        ((uint16_t)(fb.Yaw_fed_h << 8 | fb.Yaw_fed_l )),
        ((uint16_t)(fb.rail_speed_h << 8 | fb.rail_speed_l )),
        fb.rail_speed_direction
//        fb.bullet_speed,
//        fb.rail_pos,
//        fb.shot_armor,
//        fb.remain_HP
    };
}

int Serial::send()
{
    //    Linux原始串口通信与Linux终端通信不同，
    //    终端通信都是以'\n'换行符作为一次通信的结束符，而原始串口没有这个约定.
    //    打开串口后，最好清除一下串口接收和发送缓冲区：
    tcflush(_serialFd, TCOFLUSH); //清除一下接收和发送缓冲区
    int sendCount;

    try
    {
       sendCount  = write(_serialFd, &_controlFrame, sizeof(ControlFrame));

//       cout<<"           sendCount:"<<sendCount<<endl;
    }
    catch(exception e)
    {
        cout << e.what() << endl;
        return _errorCode = SYSTEM_ERROR;
    }

    if (sendCount == -1)
	{
        if (_en_debug)
        {
            cout << "\tSerial sending failed. Frame sequence: " << (int)_controlFrame.frame_seq << endl;
        }
        _errorCode = READ_WRITE_ERROR;
	}
    else if (sendCount < static_cast<int>(sizeof(ControlFrame)))
	{
        if (_en_debug)
        {
            cout << "\tSerial sending failed. "<< sizeof(ControlFrame) - sendCount <<
                    " bytes unsent. Frame sequence: " << (int)_controlFrame.frame_seq << endl;
        }
        _errorCode = READ_WRITE_ERROR;
    }
    else
    {
//        if (_en_debug)
//        {
//            cout << "\tSerial sending succeeded. " << "Frame sequence: "
//                 << (int)_controlFrame.frame_seq << endl;
//        }

#ifdef SERIAL_PRINT
        getState();
        cout<<"the state of sendCount:      "<<to_string(sendCount)<<endl;
        print(_controlFrame);
#endif
        _errorCode = OJBK;
    }

    return _errorCode;
}

int Serial::receive()
{
    memset(&_feedBackFrame,0,sizeof(_feedBackFrame));//feedbackframe == 0
    tcflush(_serialFd, TCIFLUSH);

    int readCount = 0;


    readCount  = read(_serialFd, ((unsigned char *)(&_feedBackFrame)), sizeof(FeedBackFrame));

    if (readCount == -1)
    {
        if (_en_debug)
        {
            cout << "\tSerial reading failed."<< endl;
//            perror("read");
        }
        _errorCode = READ_WRITE_ERROR;
    }
    else if (readCount < static_cast<int>(sizeof(FeedBackFrame)))
    {
        if (_en_debug)
        {
            cout << "\tSerial reading failed. not enough"<< endl;
        }
        _errorCode = READ_WRITE_ERROR;
    }
    else
    {
        if (_feedBackFrame.SOF != JetsonCommSOF || _feedBackFrame.EOF != JetsonCommEOF)
        {
            if (_en_debug)
            {
//                cout << "\tFeed back frame SOF or EOF is not correct. SOF: " << (int)_feedBackFrame.SOF << " ,EOF: " << (int)_feedBackFrame.EOF << endl;
            }
            return _errorCode = CORRUPTED_FRAME;
        }
        else
        {
            if (_en_debug)
            {
    //            cout << "\tSerial receiving succeeded. " << "SOF: " << (int)_feedBackFrame.SOF << " ,EOF: " << (int)_feedBackFrame.EOF << endl;
//                cout << "\tSerial receiving succeeded. AND  TASK  IS    "<< to_string(_feedBackFrame.task_mode)<< endl;

//                cout << "\tSerial receiving succeeded. AND  Yaw_fed  IS    "
//                     << ((uint16_t)(_feedBackFrame.Yaw_fed_h << 8 | _feedBackFrame.Yaw_fed_l )/ 100.0)<< endl;

//                cout << "\tSerial receiving succeeded. AND  Yaw_fed  IS    "
//                     << ((uint16_t)(_feedBackFrame.rail_speed_h << 8 | _feedBackFrame.rail_speed_l )/ 100.0)<< endl;
//                  cout << "\tSerial receiving succeeded. AND  rail_speed_direction  IS    "
//                       << to_string(_feedBackFrame.rail_speed_direction)<< endl;
            }
        }
    #ifdef SERIAL_PRINT
            print(_feedBackFrame);
    #endif
            return _errorCode = OJBK;
    }
}


}
