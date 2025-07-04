#ifndef CAMERA_HPP
#define CAMERA_HPP
#include "ros/ros.h"
#include <stdio.h>
#include <pthread.h>
#include <opencv2/opencv.hpp>
#include "MvErrorDefine.h"
#include "CameraParams.h"
#include "MvCameraControl.h"

namespace camera
{
//********** define ************************************/
#define MAX_IMAGE_DATA_SIZE (4 * 2048 * 3072)
    //********** frame ************************************/
    cv::Mat frame;
    ros::Time frametime;
    //********** frame_empty ******************************/
    bool frame_empty = 0;
    //********** mutex ************************************/
    pthread_mutex_t mutex;
    //********** CameraProperties config ************************************/
    enum CamerProperties
    {
        CAP_PROP_FRAMERATE_ENABLE,  //帧数可调
        CAP_PROP_FRAMERATE,         //帧数
        CAP_PROP_BURSTFRAMECOUNT,   //外部一次触发帧数
        CAP_PROP_HEIGHT,            //图像高度
        CAP_PROP_WIDTH,             //图像宽度
        CAP_PROP_EXPOSURE_TIME,     //曝光时间
        CAP_PROP_GAMMA_ENABLE,      //伽马因子可调
        CAP_PROP_GAMMA,             //伽马因子
        CAP_PROP_GAINAUTO,          //亮度
        CAP_PROP_SATURATION_ENABLE, //饱和度可调
        CAP_PROP_SATURATION,        //饱和度
        CAP_PROP_OFFSETX,           //X偏置
        CAP_PROP_OFFSETY,           //Y偏置
        CAP_PROP_TRIGGER_MODE,      //外部触发
        CAP_PROP_TRIGGER_SOURCE,    //触发源
        CAP_PROP_LINE_SELECTOR      //触发线

    };

    //^ *********************************************************************************** //
    //^ ********************************** Camera Class************************************ //
    //^ *********************************************************************************** //
    class Camera
    {
    public:
        //********** 构造函数  ****************************/
        Camera(ros::NodeHandle &node);
        //********** 析构函数  ****************************/
        ~Camera();
        //********** 原始信息转换线程 **********************/
        static void *HKWorkThread(void *p_handle);

        //********** 输出摄像头信息 ***********************/
        bool PrintDeviceInfo(MV_CC_DEVICE_INFO *pstMVDevInfo);
        //********** 设置摄像头参数 ***********************/
        bool set(camera::CamerProperties type, float value);
        //********** 恢复默认参数 *************************/
        bool reset();
        //********** 读图10个相机的原始图像 ********************************/
        void ReadImg(cv::Mat &image, ros::Time& time);

        int get_width();
        int get_height();
        int get_Offset_x();
        int get_Offset_y();
        boost::array<double, 9> get_cameraMatrix();
        std::vector<double> get_distCoeffs();

    private:
        //********** handle ******************************/
        void *handle;
        std::vector<void *> handlelist;
        //********** nThreadID ******************************/
        pthread_t nThreadID;
        std::vector<pthread_t> nThreadIDlist;
        //********** yaml config ******************************/
        int nRet;
        int width;
        int height;
        int Offset_x;
        int Offset_y;
        bool FrameRateEnable;
        int FrameRate;
        int BurstFrameCount;
        int ExposureTime;
        bool GammaEnable;
        float Gamma;
        int GainAuto;
        bool SaturationEnable;
        int Saturation;
        int TriggerMode;
        int TriggerSource;
        int LineSelector;

        int downsample;

        bool    calibrate_enable_;
        cv::Mat cameraMatrix_;  // 相机内参
        cv::Mat distCoeffs_;     // 相机畸变校正参数

    };
    //^ *********************************************************************************** //

    //^ ********************************** Camera constructor************************************ //
    Camera::Camera(ros::NodeHandle &node)
    {
        handle = NULL;
        cameraMatrix_ = cv::Mat::zeros(3, 3, CV_64F);
        distCoeffs_ = cv::Mat::zeros(5, 1, CV_64F);
        ROS_INFO("==========load camera params=============");
        //********** 读取待设置的摄像头参数 第三个参数是默认值 yaml文件未给出该值时生效 ********************************/
        node.param("width", width, 3072);
        node.param("height", height, 2048);
        node.param("FrameRateEnable", FrameRateEnable, true);
        node.param("FrameRate", FrameRate, 10);
        node.param("BurstFrameCount", BurstFrameCount, 1); // 一次触发采集的次数
        node.param("ExposureTime", ExposureTime, 50000);
        node.param("GammaEnable", GammaEnable, true);
        node.param("Gamma", Gamma, (float)0.7);
        node.param("GainAuto", GainAuto, 2);
        node.param("SaturationEnable", SaturationEnable,false);
        node.param("Saturation", Saturation, 128);
        node.param("Offset_x", Offset_x, 0);
        node.param("Offset_y", Offset_y, 0);
        node.param("TriggerMode", TriggerMode, 1);
        node.param("TriggerSource", TriggerSource, 2);
        node.param("LineSelector", LineSelector, 1);

        node.param("downsample", downsample, 2);
        
        node.param("CalibrateEnable", calibrate_enable_, false);
        if(calibrate_enable_) {
            double fx, fy, cx, cy, k1, k2, p1, p2, k3;
            node.param("CameraMatrix/fx", fx, 1.);
            node.param("CameraMatrix/fy", fy, 1.);
            node.param("CameraMatrix/cx", cx, 0.);
            node.param("CameraMatrix/cy", cy, 0.);
            cameraMatrix_.at<double>(0,0) = fx;
            cameraMatrix_.at<double>(1,1) = fy;
            cameraMatrix_.at<double>(0,2) = cx;
            cameraMatrix_.at<double>(1,2) = cy;
            cameraMatrix_.at<double>(2,2) = 1.;
            // 打印内参
            std::cout << "cameraMatrix_: " << cameraMatrix_ << std::endl;

            node.param("DistCoeffs/k1", k1, 0.);
            node.param("DistCoeffs/k2", k2, 0.);
            node.param("DistCoeffs/p1", p1, 0.);
            node.param("DistCoeffs/p2", p2, 0.);
            node.param("DistCoeffs/k3", k3, 0.);
            distCoeffs_.at<double>(0,0) = k1;
            distCoeffs_.at<double>(1,0) = k2;
            distCoeffs_.at<double>(2,0) = p1;
            distCoeffs_.at<double>(3,0) = p2;
            distCoeffs_.at<double>(4,0) = k3;
            // 打印畸变系数
            std::cout << "distCoeffs_: " << distCoeffs_ << std::endl;
        }

        //********** 初始化SDK ********************************/
        // nRet = MV_CC_Initialize();
        // if (MV_OK != nRet)
        // {
        //     printf("Initialize SDK fail! nRet [0x%x]\n", nRet);
        //     exit(-1);
        // }
        uint32_t sdk_v = MV_CC_GetSDKVersion();
        printf("SDK Version: v%d.%d.%d.%d\n", (sdk_v&0xFF000000)>>24, (sdk_v&0x00FF0000)>>16, (sdk_v&0x0000FF00)>>8, sdk_v&0x000000FF);

        //********** 枚举设备 ********************************/
        MV_CC_DEVICE_INFO_LIST stDeviceList;
        memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
        nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
        if (MV_OK != nRet)
        {
            printf("MV_CC_EnumDevices fail! nRet [%x]\n", nRet);
            exit(-1);
        }

        int deviceID = 0;
        bool hasdevice = false;
        for (int i = 0; i < stDeviceList.nDeviceNum; i++)
        {
            printf("[device %d]:\n", i);
            MV_CC_DEVICE_INFO *pDeviceInfo = stDeviceList.pDeviceInfo[i];
            if (NULL == pDeviceInfo)
            {
                break;
            }
            PrintDeviceInfo(pDeviceInfo);
        }
        //********** 选择设备并创建句柄 *************************/
        nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[deviceID]);
        if (MV_OK != nRet)
        {
            printf("MV_CC_CreateHandle fail! nRet [%x]\n", nRet);
            exit(-1);
        }

        // 打开设备
        //********** frame **********/
        nRet = MV_CC_OpenDevice(handle);
        if (MV_OK != nRet)
        {
            printf("MV_CC_OpenDevice fail! nRet [%x]\n", nRet);
            exit(-1);
        }

        //设置 yaml 文件里面的配置
        this->set(CAP_PROP_FRAMERATE_ENABLE, FrameRateEnable);
        if (FrameRateEnable)
            this->set(CAP_PROP_FRAMERATE, FrameRate);
        this->set(CAP_PROP_BURSTFRAMECOUNT, BurstFrameCount);
        this->set(CAP_PROP_HEIGHT, height);
        this->set(CAP_PROP_WIDTH, width);
        this->set(CAP_PROP_OFFSETX, Offset_x);
        this->set(CAP_PROP_OFFSETY, Offset_y);
        this->set(CAP_PROP_EXPOSURE_TIME, ExposureTime);
        this->set(CAP_PROP_GAMMA_ENABLE, GammaEnable);
        if (GammaEnable)
            this->set(CAP_PROP_GAMMA, Gamma);
        this->set(CAP_PROP_GAINAUTO, GainAuto);

        this->set(CAP_PROP_TRIGGER_MODE, TriggerMode);
        this->set(CAP_PROP_TRIGGER_SOURCE, TriggerSource);
        this->set(CAP_PROP_LINE_SELECTOR, LineSelector);

        // MV_CC_SetBoolValue(handle, "DigitalShiftEnable", true);
        // float fValue = 20.0000;
        // MV_CC_SetFloatValue(handle, "DigitalShift", fValue);

        nRet = MV_CC_SetEnumValue(handle, "DecimationHorizontal", downsample);
        if (MV_OK == nRet)
        {
            printf("set DecimationHorizontal OK! value=%f\n", (float)downsample);
        }else
        {
            printf("Set DecimationHorizontal Failed! nRet = [%x]\n\n", nRet);
        }

        // nRet = MV_CC_SetEnumValue(handle, "DecimationVertical", downsample);
        // if (MV_OK == nRet)
        // {
        //     printf("set DecimationVertical OK! value=%f\n", (float)downsample);
        // }else
        // {
        //     printf("Set DecimationVertical Failed! nRet = [%x]\n\n", nRet);
        // }

        nRet = MV_CC_SetBoolValue(handle, "AutoSCPD", 1);
        if (MV_OK == nRet)
        {
            printf("set AutoSCPD OK! value=%f\n", 1.0);
        }else
        {
            printf("Set AutoSCPD Failed! nRet = [%x]\n\n", nRet);
        }

        //********** frame **********/
        //白平衡 非自适应（给定参数0） 关闭
        nRet = MV_CC_SetEnumValue(handle, "BalanceWhiteAuto", 0);
        if (MV_OK == nRet)
        {
            printf("set BalanceRatio OK! value=%f\n",0.0 );
        }
        else
        {
            printf("Set BalanceRatio Failed! nRet = [%x]\n\n", nRet);
        }
        this->set(CAP_PROP_SATURATION_ENABLE, SaturationEnable);
        if (SaturationEnable)
            this->set(CAP_PROP_SATURATION, Saturation);
        //软件触发
        // ********** frame **********/
        nRet = MV_CC_SetEnumValue(handle, "TriggerMode", 0);
        if (MV_OK == nRet)
        {
            printf("set TriggerMode OK!\n");
        }
        else
        {
            printf("MV_CC_SetTriggerMode fail! nRet [%x]\n", nRet);
        }

        //********** 图像格式 **********/
        // 0x01100003:Mono10
        // 0x010C0004:Mono10Packed
        // 0x01100005:Mono12
        // 0x010C0006:Mono12Packed
        // 0x01100007:Mono16
        // 0x02180014:RGB8Packed
        // 0x02100032:YUV422_8
        // 0x0210001F:YUV422_8_UYVY
        // 0x01080008:BayerGR8 
        // 0x01080009:BayerRG8
        // 0x0108000A:BayerGB8
        // 0x0108000B:BayerBG8
        // 0x0110000e:BayerGB10
        // 0x01100012:BayerGB12
        // 0x010C002C:BayerGB12Packed
        nRet = MV_CC_SetEnumValue(handle, "PixelFormat", 0x01080008); // BayerRG8
        if (MV_OK == nRet)
        {
            printf("set PixelFormat OK ! value = BayerRG8\n");
        }
        else
        {
            printf("MV_CC_SetPixelFormat fail! nRet [%x]\n", nRet);
        }
        MVCC_ENUMVALUE t = {0};
        //********** frame **********/

        nRet = MV_CC_GetEnumValue(handle, "PixelFormat", &t);
        if (MV_OK == nRet)
        {
            printf("PixelFormat :%d!\n", t.nCurValue); // 35127316
        }
        else
        {
            printf("get PixelFormat fail! nRet [%x]\n", nRet);
        }
        // 开始取流
        //********** frame **********/

        nRet = MV_CC_StartGrabbing(handle);
        if (MV_OK != nRet)
        {
            printf("MV_CC_StartGrabbing fail! nRet [%x]\n", nRet);
            exit(-1);
        }
        //初始化互斥量
        nRet = pthread_mutex_init(&mutex, NULL);
        if (nRet != 0)
        {
            perror("pthread_create failed\n");
            exit(-1);
        }
        //********** frame **********/

        nRet = pthread_create(&nThreadID, NULL, HKWorkThread, handle);

        if (nRet != 0)
        {
            printf("thread create failed.ret = %d\n", nRet);
            exit(-1);
        }
    }

    //^ ********************************** Camera constructor************************************ //
    Camera::~Camera()
    {
        int nRet;
        //********** frame **********/

        pthread_join(nThreadID, NULL);

        //********** frame **********/

        nRet = MV_CC_StopGrabbing(handle);

        if (MV_OK != nRet)
        {
            printf("MV_CC_StopGrabbing fail! nRet [%x]\n", nRet);
            exit(-1);
        }
        printf("MV_CC_StopGrabbing succeed.\n");
        // 关闭设备
        //********** frame **********/

        nRet = MV_CC_CloseDevice(handle);

        if (MV_OK != nRet)
        {
            printf("MV_CC_CloseDevice fail! nRet [%x]\n", nRet);
            exit(-1);
        }
        printf("MV_CC_CloseDevice succeed.\n");
        // 销毁句柄
        //********** frame **********/

        nRet = MV_CC_DestroyHandle(handle);

        if (MV_OK != nRet)
        {
            printf("MV_CC_DestroyHandle fail! nRet [%x]\n", nRet);
            exit(-1);
        }
        printf("MV_CC_DestroyHandle succeed.\n");
        // 销毁互斥量
        pthread_mutex_destroy(&mutex);
    }

    //^ ********************************** Camera constructor************************************ //
    bool Camera::set(CamerProperties type, float value)
    {
        switch (type)
        {
        case CAP_PROP_FRAMERATE_ENABLE:
        {
            //********** frame **********/

            nRet = MV_CC_SetBoolValue(handle, "AcquisitionFrameRateEnable", value);

            if (MV_OK == nRet)
            {
                printf("set AcquisitionFrameRateEnable OK! value=%f\n",value);
            }
            else
            {
                printf("Set AcquisitionFrameRateEnable Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_FRAMERATE:
        {
            //********** frame **********/

            nRet = MV_CC_SetFloatValue(handle, "AcquisitionFrameRate", value);

            if (MV_OK == nRet)
            {
                printf("set AcquisitionFrameRate OK! value=%f\n",value);
            }
            else
            {
                printf("Set AcquisitionFrameRate Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_BURSTFRAMECOUNT:
        {
            //********** frame **********/

            nRet = MV_CC_SetIntValue(handle, "AcquisitionBurstFrameCount", value);

            if (MV_OK == nRet)
            {
                printf("set AcquisitionBurstFrameCount OK!\n");
            }
            else
            {
                printf("Set AcquisitionBurstFrameCount Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_HEIGHT:
        {
            //********** frame **********/

            nRet = MV_CC_SetIntValue(handle, "Height", value); //图像高度

            if (MV_OK == nRet)
            {
                printf("set Height OK!\n");
            }
            else
            {
                printf("Set Height Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_WIDTH:
        {
            //********** frame **********/

            nRet = MV_CC_SetIntValue(handle, "Width", value); //图像宽度

            if (MV_OK == nRet)
            {
                printf("set Width OK!\n");
            }
            else
            {
                printf("Set Width Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_OFFSETX:
        {
            //********** frame **********/

            nRet = MV_CC_SetIntValue(handle, "OffsetX", value); //图像宽度

            if (MV_OK == nRet)
            {
                printf("set Offset X OK!\n");
            }
            else
            {
                printf("Set Offset X Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_OFFSETY:
        {
            //********** frame **********/

            nRet = MV_CC_SetIntValue(handle, "OffsetY", value); //图像宽度

            if (MV_OK == nRet)
            {
                printf("set Offset Y OK!\n");
            }
            else
            {
                printf("Set Offset Y Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_EXPOSURE_TIME:
        {
            //********** frame **********/

            nRet = MV_CC_SetFloatValue(handle, "ExposureTime", value); //曝光时间

            if (MV_OK == nRet)
            {
                printf("set ExposureTime OK! value=%f\n",value);
            }
            else
            {
                printf("Set ExposureTime Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_GAMMA_ENABLE:
        {
            //********** frame **********/

            nRet = MV_CC_SetBoolValue(handle, "GammaEnable", value); //伽马因子是否可调  默认不可调（false）

            if (MV_OK == nRet)
            {
                printf("set GammaEnable OK! value=%f\n",value);
            }
            else
            {
                printf("Set GammaEnable Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_GAMMA:
        {
            //********** frame **********/

            nRet = MV_CC_SetFloatValue(handle, "Gamma", value); //伽马越小 亮度越大

            if (MV_OK == nRet)
            {
                printf("set Gamma OK! value=%f\n",value);
            }
            else
            {
                printf("Set Gamma Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_GAINAUTO:
        {
            //********** frame **********/
            nRet = MV_CC_SetEnumValue(handle, "GainAuto", value); //增益模式

            if (MV_OK == nRet)
            {
                printf("set GainAuto OK! value=%f\n",value);
            }
            else
            {
                printf("Set GainAuto Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_SATURATION_ENABLE:
        {
            //********** frame **********/

            nRet = MV_CC_SetBoolValue(handle, "SaturationEnable", value); //饱和度是否可调 默认不可调(false)

            if (MV_OK == nRet)
            {
                printf("set SaturationEnable OK! value=%f\n",value);
            }
            else
            {
                printf("Set SaturationEnable Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_SATURATION:
        {
            //********** frame **********/

            nRet = MV_CC_SetIntValue(handle, "Saturation", value); //饱和度 默认128 最大255

            if (MV_OK == nRet)
            {
                printf("set Saturation OK! value=%f\n",value);
            }
            else
            {
                printf("Set Saturation Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }

        case CAP_PROP_TRIGGER_MODE:
        {

            nRet = MV_CC_SetEnumValue(handle, "TriggerMode", value); //饱和度 默认128 最大255

            if (MV_OK == nRet)
            {
                printf("set TriggerMode OK!\n");
            }
            else
            {
                printf("Set TriggerMode Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_TRIGGER_SOURCE:
        {

            nRet = MV_CC_SetEnumValue(handle, "TriggerSource", value); //饱和度 默认128 最大255255

            if (MV_OK == nRet)
            {
                printf("set TriggerSource OK!\n");
            }
            else
            {
                printf("Set TriggerSource Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_LINE_SELECTOR:
        {

            nRet = MV_CC_SetEnumValue(handle, "LineSelector", value); //饱和度 默认128 最大255

            if (MV_OK == nRet)
            {
                printf("set LineSelector OK!\n");
            }
            else
            {
                printf("Set LineSelector Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }

        default:
            return 0;
        }
        return nRet;
    }

    //^ ********************************** Camera constructor************************************ //
    bool Camera::reset()
    {
        nRet = this->set(CAP_PROP_FRAMERATE_ENABLE, FrameRateEnable);
        nRet = this->set(CAP_PROP_FRAMERATE, FrameRate) || nRet;
        // nRet = this->set(CAP_PROP_BURSTFRAMECOUNT, BurstFrameCount) || nRet;
        nRet = this->set(CAP_PROP_HEIGHT, height) || nRet;
        nRet = this->set(CAP_PROP_WIDTH, width) || nRet;
        nRet = this->set(CAP_PROP_OFFSETX, Offset_x) || nRet;
        nRet = this->set(CAP_PROP_OFFSETY, Offset_y) || nRet;
        nRet = this->set(CAP_PROP_EXPOSURE_TIME, ExposureTime) || nRet;
        nRet = this->set(CAP_PROP_GAMMA_ENABLE, GammaEnable) || nRet;
        nRet = this->set(CAP_PROP_GAMMA, Gamma) || nRet;
        nRet = this->set(CAP_PROP_GAINAUTO, GainAuto) || nRet;
        nRet = this->set(CAP_PROP_SATURATION_ENABLE, SaturationEnable) || nRet;
        nRet = this->set(CAP_PROP_SATURATION, Saturation) || nRet;
        nRet = this->set(CAP_PROP_TRIGGER_MODE, TriggerMode) || nRet;
        nRet = this->set(CAP_PROP_TRIGGER_SOURCE, TriggerSource) || nRet;
        nRet = this->set(CAP_PROP_LINE_SELECTOR, LineSelector) || nRet;
        return nRet;
    }

    //^ ********************************** PrintDeviceInfo ************************************ //
    bool Camera::PrintDeviceInfo(MV_CC_DEVICE_INFO *pstMVDevInfo)
    {
        if (NULL == pstMVDevInfo)
        {
            printf("%s\n", "The Pointer of pstMVDevInfoList is NULL!");
            return false;
        }
        if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE)
        {
            uint32_t ip = pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp;
            
            printf("%s %d.%d.%d.%d\n", "nCurrentIp:", (ip&0xFF000000)>>24, (ip&0xFF0000)>>16, (ip&0xFF00)>>8, (ip&0xFF));   //当前IP
            printf("%s %s\n", "chUserDefinedName:", pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName); //用户定义名
            printf("%s %s\n", "chSerialNumber:", pstMVDevInfo->SpecialInfo.stGigEInfo.chSerialNumber);
            printf("%s %s\n\n", "chModelName:", pstMVDevInfo->SpecialInfo.stGigEInfo.chModelName);
        }
        else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE)
        {
            printf("UserDefinedName:%s\n\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
        }
        else
        {
            printf("Not support.\n");
        }
        return true;
    }

    //^ ********************************** Camera constructor************************************ //
    void Camera::ReadImg(cv::Mat &image, ros::Time& time)
    {

        pthread_mutex_lock(&mutex);
        time = frametime;
        if (frame_empty)
        {
            image = cv::Mat();
        }
        else
        {
            image = camera::frame.clone();
            frame_empty = 1;
        }
        pthread_mutex_unlock(&mutex);
    }

    //^ ********************************** HKWorkThread1 ************************************ //
    void *Camera::HKWorkThread(void *p_handle)
    {
        double start;
        int nRet;
        unsigned char *m_pBufForDriver = (unsigned char *)malloc(sizeof(unsigned char) * MAX_IMAGE_DATA_SIZE);
        unsigned char *m_pBufForSaveImage = (unsigned char *)malloc(MAX_IMAGE_DATA_SIZE);
        MV_FRAME_OUT_INFO_EX stImageInfo = {0};
        MV_CC_PIXEL_CONVERT_PARAM stConvertParam = {0};
        cv::Mat tmp;
        int image_empty_count = 0; //空图帧数
        while (ros::ok())
        {
            start = static_cast<double>(cv::getTickCount());
            nRet = MV_CC_GetOneFrameTimeout(p_handle, m_pBufForDriver, MAX_IMAGE_DATA_SIZE, &stImageInfo, 15);
            if (nRet != MV_OK)
            {
                if (++image_empty_count > 100)
                {
                    ROS_INFO("The Number of Faild Reading Exceed The Set Value!\n");
                    exit(-1);
                }
                continue;
            }
            image_empty_count = 0; //空图帧数
            //转换图像格式为BGR8

            // stConvertParam.nWidth = 3072;                               //ch:图像宽 | en:image width
            // stConvertParam.nHeight = 2048;                              //ch:图像高 | en:image height

            stConvertParam.nWidth = stImageInfo.nWidth;                               //ch:图像宽 | en:image width
            stConvertParam.nHeight = stImageInfo.nHeight;                              //ch:图像高 | en:image height
            stConvertParam.pSrcData = m_pBufForDriver;                  //ch:输入数据缓存 | en:input data buffer
            stConvertParam.nSrcDataLen = MAX_IMAGE_DATA_SIZE;           //ch:输入数据大小 | en:input data size
            stConvertParam.enDstPixelType = PixelType_Gvsp_BGR8_Packed; //ch:输出像素格式 | en:output pixel format                      //! 输出格式 RGB
            stConvertParam.pDstBuffer = m_pBufForSaveImage;             //ch:输出数据缓存 | en:output data buffer
            stConvertParam.nDstBufferSize = MAX_IMAGE_DATA_SIZE;        //ch:输出缓存大小 | en:output buffer size
            stConvertParam.enSrcPixelType = stImageInfo.enPixelType;    //ch:输入像素格式 | en:input pixel format                       //! 输入格式 RGB
            MV_CC_ConvertPixelType(p_handle, &stConvertParam);
            pthread_mutex_lock(&mutex);
            // camera::frame = cv::Mat(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC3, m_pBufForSaveImage).clone(); //tmp.clone();
            // cv::Mat bayerImage = cv::Mat(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC1, m_pBufForSaveImage).clone(); //tmp.clone();
            // cv::Mat bgrImage(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC3);
            // cv::cvtColor(bayerImage, bgrImage, CV_BayerBG2BGR);
            // camera::frame = bgrImage.clone();

            // cv::Mat rgbImage = cv::Mat(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC3, m_pBufForSaveImage).clone();
            // cv::Mat sampleUp;
            // cv::resize(rgbImage, sampleUp, cv::Size(stImageInfo.nWidth*4, stImageInfo.nHeight*4));
            // camera::frame = sampleUp.clone();
            camera::frame = cv::Mat(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC3, m_pBufForSaveImage).clone();
            camera::frametime = ros::Time::now();

            frame_empty = 0;
            pthread_mutex_unlock(&mutex);
            double time = ((double)cv::getTickCount() - start) / cv::getTickFrequency();
            //*************************************testing img********************************//
            // std::cout << "HK_camera,Time:" << time << "\tFPS:" << 1 / time << std::endl;
            // cv::imshow("HK vision",frame);
            // cv::waitKey(1);
        }
        free(m_pBufForDriver);
        free(m_pBufForSaveImage);
        return 0;
    }

    int Camera::get_width() {return width;}
    int Camera::get_height() {return height;}
    int Camera::get_Offset_x() {return Offset_x;}
    int Camera::get_Offset_y() {return Offset_y;}

    boost::array<double, 9> Camera::get_cameraMatrix() {
        boost::array<double, 9> r = {0};
        if(calibrate_enable_) {
            r.at(0) = cameraMatrix_.at<double>(0, 0);
            r.at(2) = cameraMatrix_.at<double>(0, 2);
            r.at(4) = cameraMatrix_.at<double>(1, 1);
            r.at(5) = cameraMatrix_.at<double>(1, 2);
            r.at(8) = cameraMatrix_.at<double>(2, 2);
        }
        return r;
    }

    std::vector<double> Camera::get_distCoeffs() {
        std::vector<double> r;
        if(calibrate_enable_) {
            r.push_back(distCoeffs_.at<double>(0, 0));
            r.push_back(distCoeffs_.at<double>(1, 0));
            r.push_back(distCoeffs_.at<double>(2, 0));
            r.push_back(distCoeffs_.at<double>(3, 0));
            r.push_back(distCoeffs_.at<double>(4, 0));
        }
        return r;
    }

} // namespace camera
#endif
