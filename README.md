# Born of Fire战队21赛季步兵视觉 
本项目为南京航空航天大学金城学院Born of Fire战队21赛季步兵视觉源码，用于队内新队员的起步教学。在对计算机视觉领域相关知识有了较熟悉的理解后可以承上启下，开点新坑（机器视觉、深度学习）。
其次感谢东南大学以及北京理工大学珠海学院的19赛季开源代码和DJI官方的开源，对于本套代码的编写在思路上提供了很大的帮助。  
代码内部可能还存在部分问题，内部由于存在部分新旧代码以及模式切换代码冗余可能造成阅读困难深感抱歉，如有疑问欢迎联系相关技术人员讨论。  
联系方式：周煜 QQ:374957510  丁嘉伟 QQ:
&emsp;&emsp;&emsp;&emsp;&emsp; 
<!-- TOC -->

- [Born of Fire战队21季步兵视觉开源](#Born of Fire战队21季步兵视觉开源)
  - [1.主要功能模块介绍和效果展示](#1主要功能模块介绍和效果展示)
  - [2. 依赖工具和编译环境](#2-依赖工具和编译环境)
  - [3. 编译与调试方式](#3-编译与调试方式)
  - [4. 功能模块具体实现方案](#4-功能模块具体实现方案)
    - [装甲检测：](#装甲检测)
    - [大符识别：](#大符识别)
    - [大符击打：](#大符击打)
  - [5. 整体架构](#5-整体架构)
  - [6 未来展望](#6未来展望)

<!-- /TOC -->
## 1.主要功能模块介绍和效果展示 
该代码主要分为六个部分，分别为装甲识别、角度结算、大小符的识别与击打以及相机驱动和串口通信等其他内容。以下是对于主要功能模块的介绍。  
- 装甲检测：相机为KS2A543，采用640*480，相机可跑满，大概可稳定识别5m内目标，根据本赛季视觉大概击打3m目标的战术定位来说可满足要求。  
- 大符识别与击打：打符与装甲板识别为同一个相机，放置于云台，于队内测试击打效果尚可，可触发小符效果。   
## 2. 依赖工具和编译环境
- ubuntu 16.04
- QT Community 5.8.0
- opencv 3.4.5
- gcc 5.4.0
- C++14准

## 3. 编译与调试方式
[RM环境配置总结](https://blog.csdn.net/monszho/article/details/108577042)  
在类AngleSolverParam函数readFile中更改xml文件路径。

## 4. 功能模块具体实现方案
  ### 装甲检测：
  - 利用颜色权重进行颜色分割，灰度二值化，将灰度二值图与颜色分割图做差，去除干扰，并对最终二值图进行形态学及滤波操作。
  - 寻找轮廓并进行最小矩形法得到候选灯条集合。
  - 灯条两两组合得到候选灯条对。
  - 装甲集合进行二次排序去除不符合条件的装甲。 
  - 第三次筛选装甲集合，将装甲集合与上几帧识别装甲对比，以累计的方式确定此次识别的装甲板类型并二次排序筛选出最终装甲板。

### 大符识别：

  - 灰度二值化及形态学和滤波操作，得到完整的风车二值图像。
  - 寻找全部轮廓。
  - 利用椭圆拟合和特征进行筛选找到疑似扇叶
  - 利用轮廓层级关系和内轮廓特征筛选扇叶，找到待击打扇叶。  

### 大符击打：
  - 单目测距得到相机坐标系下相对坐标，并记录。
  - 直接识别R标识得到空间圆心。
  - 利用圆心和圆弧上点计算风车平面相对相机平面的法向量。
  - 计算预测点，绝对转相对，计算相对云台偏转角。  

## 5. 整体架构
（1）文件树：
```
Robomaster2021-BOF-master/  
├─Armor                           //装甲板头识别功能
│      ArmorDetector.cpp/hpp
│      README.md
│
├─Driver                          //相机驱动
│      RMVideoCapture.cpp/hpp     //usb相机接口
│
├─Energy                          //能量机关
│      EnergyDetector.cpp/hpp   
│
├─General                         //各种外拓展API，用处不大
│        
├─Main                            //主函数入口
│      ImgProdCons.cpp/hpp         //多线程定义
│      test_infantry.cpp  
│      main.cpp                   //主函数，创造线程
│        
├─Pose                             //位姿检测
│      AngleSolver.cpp/hpp           //位姿检测功能  
│      angle_solver_params.xml       //相机参数
│      README.md                  
│        
├─Serials                            //串通讯
│       Serial.cpp/hpp   
│
```
（2）整体思路：  
&emsp;&emsp;软件的执行开始首先创建四个线程，分别是图像生产和处理线程，与stm32数据的接收和发送以及录制视频线程。对于图像的生产和处理线程是通过生产者和消费者算法控制图像的异步生产和消费，提高运行速度；在自瞄图像处理的过程中得到装甲端点的像素坐标以及是否连续识别相同装甲类型等基本信息，将基本信息传入角度解算函数通过pnp姿态估计得到目标在相机坐标系下的坐标，然后再根据当前目标状态（第一次发现该目标、连续识别相同目标和掉帧缓冲）执行不同的应对方案，这是该软件解决不同状态的核心思路。就移动预测方案举例，当第一次发现该目标时将各个保留值进行初始化，连续识别则连续迭代滤波计算预测量，掉帧缓冲则按上一帧位置和速度继续估算目标位置。通过从stm32接受数据确定当前是否需要切换到击打能力机关模式。  
 
## 7. 未来展望
1. 相机方案的优化：
由于之前一直用的是无驱相机KS2A543，和队内新买的海康和大华相比，虽然分辨率都是640X480,但是帧率上有6～8倍的差距。在NUC上运行程序时，达到了处理一张图片耗时只用1.3MS左右，队内买的海康与大恒工业相机帧率为800Hz左右，可以完美跑满，而KS2A543的帧率只有100Hz。在实战中，无驱相机的打击效果还是稍有滞后（也有没写预测的原因在里面）。虽然能用，但谁不想用更好的装备呢（= =）。海康的驱动我已经写好了，后人只需要往里面填充新的识别代码即可。
2.识别算法优化：
当前项目的所有识别功能都是在摄像头超低曝光的前提下进行的，这样虽然可以大大减少环境光的干扰，但是也丧失了很多赛场信息，如：在超低曝光下，即使Gamma值再怎么调也看不到装甲板上的数字、录制视频线程录制出来的视频都是特别特别黑的，只能看到赛场上车的红蓝灯条，赛后分析价值不高，等等。所以希望能调整曝光值来解决这一问题。
3. 移动预测和反陀螺功能：
当前项目并没有有效的移动预测和反陀螺功能，赛前也有对此进行代码编写及测试，但是效果并不理想。希望能设计出更科学有效的算法解决这一问题。
4. 能量机关：
由于21赛季中，风车的打击效果实在是差（纯纯的没识别到），原因也是在学校时队内作出的风车和实际场地上的有差距导致风车打击效果的不理想。虽说在赛场上有熟悉场地调试的环节，但是时间过于少，不应指望其来对风车功能进行调试。希望新的赛季，在有充足的调试时间的情况下，视觉组能和硬件组作出更紧密的联系。
5. 调试：
本项目并没有QT的UI界面以供调试人员可以对项目功能进行动态的调试，导致每次的修改参数都需要重新运行代码来看参数调试后的效果，调试时间成本太高。
6 培养：
视觉组虽然搞得更多的是上层软件项目，但是由于比赛的特殊性，需要与电控和硬件的紧密联系，所以最好还是需要具备有一定的硬件基础知识。（如：串口通讯）毕竟学习一门专业知识和实际操作哪里都有门路，而RM平台最终目标是培养一个复合型人才。
