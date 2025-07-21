/// \file
/// \~chinese
/// \brief 多相机拉流处理示例
/// \example MultipleCamera.cpp
/// \~english
/// \brief Multiple cameras grab stream sample
/// \example MultipleCamera.cpp

//**********************************************************************
// 本Demo为简单演示SDK的使用，没有附加修改相机IP的代码，在运行之前，请使
// 用相机客户端修改相机IP地址的网段与主机的网段一致。
// This Demo shows how to use GenICam API(C) to write a simple program.
// Please make sure that the camera and PC are in the same subnet before running
// the demo. If not, you can use camera client software to modify the IP address
// of camera to the same subnet with PC.
//**********************************************************************
#include "IMVApi.h"
#include "IMVDefines.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/time.h"
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <cmath>
#include <cv_bridge/cv_bridge.h>
#include <fstream>
#include <image_transport/image_transport.h>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Header.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

using namespace boost::asio;

#define MAX_DEVICE_NUM 16

std::mutex mtxTime;
static double TIME_NOW = 0.0;
ros::Publisher pubIMU;
ros::Publisher pubImages[MAX_DEVICE_NUM];

// 参数
// 接收到图像后，转换为的图像格式
IMV_EPixelType convertFormat = gvspPixelMono8;

bool exit_flag = false;
void SignalHandler(int signal) {
    if (signal == SIGINT) { // 捕捉 Ctrl + C 触发的 SIGINT 信号
        fprintf(stderr, "\nReceived Ctrl+C, exiting...\n");
        exit_flag = true; // 设置退出标志
    }
}

void SetupSignalHandler() {
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = SignalHandler; // 设置处理函数
    sigemptyset(&sigIntHandler.sa_mask);      // 清空信号屏蔽集
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);
}

class TIMESYNC_TCPClient { // 用于处理时间同步和IMU数据的TCP客户端

    static constexpr int MAX_MSG_LEN = 512;

public:
    TIMESYNC_TCPClient(const std::string &ip, int port)
        : ip_address_(ip)
        , port_(port)
        , tcp_stream_() {
        connect();
    }

    void connect() {
        ip::tcp::endpoint end_point(ip::address::from_string(ip_address_), port_);
        std::cout << "Connecting to server: " << end_point << std::endl;
        tcp_stream_.connect(end_point);
        if (!tcp_stream_) {
            std::cerr << "Failed to connect to server: " << end_point << std::endl;
            exit(EXIT_FAILURE);
        }
        std::cout << "Connected to server " << end_point << std::endl;
    }

    void timestamp_proc() { // 处理时间戳数据timestamp_proc
        std::cout << "start timestamp_proc" << std::endl;
        while (!exit_flag && tcp_stream_.getline(buf_, MAX_MSG_LEN)) {
            auto msg = std::string(buf_);
            std::cout << buf_ << std::endl;
            if (boost::starts_with(msg, "#CAMERA")) {
                std::vector<std::string> tokens;
                boost::split(tokens, msg, boost::is_any_of(","));
                if (tokens.size() < 3) {
                    std::cout << "Invalid message format: " << msg << std::endl;
                    return;
                }
                int week       = std::stoi(tokens[1]);
                double seconds = std::stod(tokens[2]);
                {
                    std::lock_guard<std::mutex> lock(mtxTime);
                    TIME_NOW = week * 604800 + seconds; // 604800 seconds in a week
                    printf("Timestamp received: %.6f\n", TIME_NOW);
                }
            }
        }
        std::cout << "Timestamp server connection closed." << std::endl;
    }

    void imu_proc() { // 处理IMU数据
        std::cout << "start imu_proc" << std::endl;
        while (!exit_flag && tcp_stream_.getline(buf_, MAX_MSG_LEN)) {
            auto msg = std::string(buf_);

            if (boost::starts_with(msg, "#RAWIMUXA")) {
                std::vector<std::string> tokens;
                boost::split(tokens, msg, boost::is_any_of(","));
                if (tokens.size() < 16) {
                    std::cout << "Invalid IMU message format: " << msg << std::endl;
                    return;
                }
                int week       = std::stoi(tokens[1]);
                double seconds = std::stod(tokens[2]);
                int flag       = std::stoi(tokens[3]);

                auto p2_31 = pow(2, 31);
                auto p2_16 = pow(2, 16);

                double acc_fac  = 400.0 / p2_31; // 增量
                double gyro_fac = 2160.0 / p2_31;
                acc_fac         = 1.25 / p2_16 * 9.8 / 1000.0;
                gyro_fac        = 0.1 / p2_16 * M_PI / 180.0;

                double dvz = std::stoi(tokens[4]) * acc_fac;
                double dvy = std::stoi(tokens[5]) * acc_fac;
                double dvx = std::stoi(tokens[6]) * acc_fac;

                double daz = std::stoi(tokens[7]) * gyro_fac;
                double day = std::stoi(tokens[8]) * gyro_fac;
                double dax = std::stoi(tokens[9]) * gyro_fac;

                double odom1_fac = 100.0 / p2_31; // 里程计转换因子
                double odom2_fac = 360.0 / p2_31; // 里程计转换因子

                int apd1 = std::stoi(tokens[10]) * odom1_fac;
                int bpd1 = std::stoi(tokens[11]) * odom1_fac;
                int dir1 = std::stoi(tokens[12]) * odom1_fac;

                int apd2 = std::stoi(tokens[13]) * odom2_fac;
                int bpd2 = std::stoi(tokens[14]) * odom2_fac;
                int dir2 = std::stoi(tokens[15]) * odom2_fac;

                // 发布IMU消息
                sensor_msgs::Imu imu_msg;
                imu_msg.header.stamp    = ros::Time(week * 604800 + seconds);
                imu_msg.header.frame_id = "imu_frame";

                imu_msg.linear_acceleration.x = dvx;
                imu_msg.linear_acceleration.y = dvy;
                imu_msg.linear_acceleration.z = dvz;
                imu_msg.angular_velocity.x    = dax;
                imu_msg.angular_velocity.y    = day;
                imu_msg.angular_velocity.z    = daz;

                pubIMU.publish(imu_msg);

                // TODO odometry
            }
        }
        std::cout << "IMU server connection closed." << std::endl;
    }

public:
    std::string ip_address_;
    int port_;

    char buf_[MAX_MSG_LEN];

    ip::tcp::iostream tcp_stream_;
};

typedef struct _CameraInfo {
    IMV_HANDLE handle;
    IMV_DeviceInfo info;

    int carmeraId; // camera index
} CameraInfo;

typedef struct _MultipleCamera {
    unsigned int nOpenCameraNum;
    CameraInfo cameraInfo[MAX_DEVICE_NUM];
} MultipleCamera;

// 图片转化
// Image convert
static cv::Mat imageConvert(CameraInfo *pCameraInfo, IMV_Frame frame, IMV_EPixelType convertFormat, const char *&cvPixelFormatStr) {
    IMV_HANDLE devHandle = pCameraInfo->handle;
    IMV_PixelConvertParam stPixelConvertParam;
    unsigned char *pDstBuf        = NULL;
    unsigned int nDstBufSize      = 0;
    int ret                       = IMV_OK;
    FILE *hFile                   = NULL;
    const char *pFileName         = NULL;
    const char *pConvertFormatStr = NULL;

    cvPixelFormatStr = "mono8";
    int cvType       = CV_8UC1;

    switch (convertFormat) {
        case gvspPixelRGB8:
            nDstBufSize       = sizeof(unsigned char) * frame.frameInfo.width * frame.frameInfo.height * 3;
            pFileName         = (const char *) "convertRGB8.bin";
            pConvertFormatStr = (const char *) "RGB8";
            cvPixelFormatStr  = "rgb8";
            cvType            = CV_8UC3;
            break;

        case gvspPixelBGR8:
            nDstBufSize       = sizeof(unsigned char) * frame.frameInfo.width * frame.frameInfo.height * 3;
            pFileName         = (const char *) "convertBGR8.bin";
            pConvertFormatStr = (const char *) "BGR8";
            cvPixelFormatStr  = "bgr8";
            cvType            = CV_8UC3;
            break;
        case gvspPixelBGRA8:
            nDstBufSize       = sizeof(unsigned char) * frame.frameInfo.width * frame.frameInfo.height * 4;
            pFileName         = (const char *) "convertBGRA8.bin";
            pConvertFormatStr = (const char *) "BGRA8";
            cvPixelFormatStr  = "bgra8";
            cvType            = CV_8UC4;
            break;
        case gvspPixelMono8:
        default:
            nDstBufSize       = sizeof(unsigned char) * frame.frameInfo.width * frame.frameInfo.height;
            pFileName         = (const char *) "convertMono8.bin";
            pConvertFormatStr = (const char *) "Mono8";
            cvPixelFormatStr  = "mono8";
            cvType            = CV_8UC1;
            break;
    }

    pDstBuf = (unsigned char *) malloc(nDstBufSize);
    if (NULL == pDstBuf) {
        printf("malloc pDstBuf failed!\n");
        return cv::Mat(); // 返回空的cv::Mat对象
    }

    // 图像转换成BGR8
    // convert image to BGR8
    memset(&stPixelConvertParam, 0, sizeof(stPixelConvertParam));
    stPixelConvertParam.nWidth          = frame.frameInfo.width;
    stPixelConvertParam.nHeight         = frame.frameInfo.height;
    stPixelConvertParam.ePixelFormat    = frame.frameInfo.pixelFormat;
    stPixelConvertParam.pSrcData        = frame.pData;
    stPixelConvertParam.nSrcDataLen     = frame.frameInfo.size;
    stPixelConvertParam.nPaddingX       = frame.frameInfo.paddingX;
    stPixelConvertParam.nPaddingY       = frame.frameInfo.paddingY;
    stPixelConvertParam.eBayerDemosaic  = demosaicNearestNeighbor;
    stPixelConvertParam.eDstPixelFormat = convertFormat;
    stPixelConvertParam.pDstBuf         = pDstBuf;
    stPixelConvertParam.nDstBufSize     = nDstBufSize;

    ret = IMV_PixelConvert(devHandle, &stPixelConvertParam);
    if (IMV_OK == ret) {
        cv::Mat srcImage(stPixelConvertParam.nHeight, stPixelConvertParam.nWidth, cvType, stPixelConvertParam.pDstBuf);
        return srcImage; // 返回转换后的图像
    } else {
        if (pDstBuf) {
            free(pDstBuf);
            pDstBuf = NULL;
        }
        printf("image convert to %s failed! ErrorCode[%d]\n", pConvertFormatStr, ret);
    }

    return cv::Mat();
}

// 数据帧回调函数
// Data frame callback function
static void onGetFrame(IMV_Frame *pFrame, void *pUser) {
    CameraInfo *pCameraInfo = (CameraInfo *) pUser;

    if (pFrame == NULL) {
        printf("pFrame is NULL\n");
        return;
    }

    if (pCameraInfo == NULL) {
        printf("pCameraInfo is NULL\n");
        return;
    }

    const char *encoding = "mono8"; // 图像编码格式
    cv::Mat srcImage     = imageConvert(pCameraInfo, *pFrame, convertFormat, encoding);

    // cv::imshow(std::to_string(pCameraInfo->carmeraId), srcImage);
    // {
    //   double now = ros::Time::now().toSec();
    //   cv::imwrite(std::to_string(TIME_NOW) + "cam" +
    //                   std::to_string(pCameraInfo->carmeraId) + "&" +
    //                   std::to_string(now) + ".png",
    //               srcImage); // 保存图像
    // }
    ///
    /// 彩色：bgr8 灰度：mono8
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), encoding, srcImage).toImageMsg();

    {
        std::lock_guard<std::mutex> lock(mtxTime);
        msg->header.stamp = ros::Time(TIME_NOW);
    }

    msg->header.frame_id = pCameraInfo->info.cameraKey;
    pubImages[pCameraInfo->carmeraId].publish(msg);

    std::cout << pubImages[pCameraInfo->carmeraId].getTopic() << " published: " << msg->header.stamp.toSec() << std::endl;

    // printf("cameraKey[%s] frame blockId = %llu\n", pCameraInfo->info.cameraKey,
    //        pFrame->frameInfo.blockId);

    return;
}

static int setLineTriggerConf(IMV_HANDLE devHandle) {
    int ret = IMV_OK;

    // 设置触发源为外部触发
    // Set trigger source to Line1
    ret = IMV_SetEnumFeatureSymbol(devHandle, "TriggerSource", "Line1");
    if (IMV_OK != ret) {
        printf("Set triggerSource value failed! ErrorCode[%d]\n", ret);
        return ret;
    }

    // 设置触发器
    // Set trigger selector to FrameStart
    ret = IMV_SetEnumFeatureSymbol(devHandle, "TriggerSelector", "FrameStart");
    if (IMV_OK != ret) {
        printf("Set triggerSelector value failed! ErrorCode[%d]\n", ret);
        return ret;
    }

    // 设置触发模式
    // Set trigger mode to On
    ret = IMV_SetEnumFeatureSymbol(devHandle, "TriggerMode", "On");
    if (IMV_OK != ret) {
        printf("Set triggerMode value failed! ErrorCode[%d]\n", ret);
        return ret;
    }

    // 设置外触发为上升沿（下降沿为FallingEdge）
    // Set trigger activation to RisingEdge(FallingEdge in opposite)
    ret = IMV_SetEnumFeatureSymbol(devHandle, "TriggerActivation", "RisingEdge");
    if (IMV_OK != ret) {
        printf("Set triggerActivation value failed! ErrorCode[%d]\n", ret);
        return ret;
    }

    return ret;
}

// ***********开始： 这部分处理与SDK操作相机无关，用于显示设备列表 ***********
// ***********BEGIN: These functions are not related to API call and used to
// display device info***********
static void displayDeviceInfo(IMV_DeviceList deviceInfoList) {
    IMV_DeviceInfo *pDevInfo = NULL;
    unsigned int cameraIndex = 0;
    char vendorNameCat[11];
    char cameraNameCat[16];

    // 打印Title行
    // Print title line
    printf("\nIdx Type Vendor     Model      S/N             DeviceUserID    IP "
           "Address    \n");
    printf("---------------------------------------------------------------------"
           "---------\n");

    for (cameraIndex = 0; cameraIndex < deviceInfoList.nDevNum; cameraIndex++) {
        pDevInfo = &deviceInfoList.pDevInfo[cameraIndex];
        // 设备列表的相机索引  最大表示字数：3
        // Camera index in device list, display in 3 characters
        printf("%-3d", cameraIndex + 1);

        // 相机的设备类型（GigE，U3V，CL，PCIe）
        // Camera type
        switch (pDevInfo->nCameraType) {
            case typeGigeCamera:
                printf(" GigE");
                break;
            case typeU3vCamera:
                printf(" U3V ");
                break;
            case typeCLCamera:
                printf(" CL  ");
                break;
            case typePCIeCamera:
                printf(" PCIe");
                break;
            default:
                printf("     ");
                break;
        }

        // 制造商信息  最大表示字数：10
        // Camera vendor name, display in 10 characters
        if (strlen(pDevInfo->vendorName) > 10) {
            memcpy(vendorNameCat, pDevInfo->vendorName, 7);
            vendorNameCat[7] = '\0';
            strcat(vendorNameCat, "...");
            printf(" %-10.10s", vendorNameCat);
        } else {
            printf(" %-10.10s", pDevInfo->vendorName);
        }

        // 相机的型号信息 最大表示字数：10
        // Camera model name, display in 10 characters
        printf(" %-10.10s", pDevInfo->modelName);

        // 相机的序列号 最大表示字数：15
        // Camera serial number, display in 15 characters
        printf(" %-15.15s", pDevInfo->serialNumber);

        // 自定义用户ID 最大表示字数：15
        // Camera user id, display in 15 characters
        if (strlen(pDevInfo->cameraName) > 15) {
            memcpy(cameraNameCat, pDevInfo->cameraName, 12);
            cameraNameCat[12] = '\0';
            strcat(cameraNameCat, "...");
            printf(" %-15.15s", cameraNameCat);
        } else {
            printf(" %-15.15s", pDevInfo->cameraName);
        }

        // GigE相机时获取IP地址
        // IP address of GigE camera
        if (pDevInfo->nCameraType == typeGigeCamera) {
            printf(" %s", pDevInfo->DeviceSpecificInfo.gigeDeviceInfo.ipAddress);
        }

        printf("\n");
    }

    return;
}

int openCamera(MultipleCamera *pMultipleCameraInfo, ros::NodeHandle &nh, image_transport::ImageTransport &it) {
    unsigned int index         = 0;
    int ret                    = IMV_OK;
    unsigned int openCameraNum = 0;

    IMV_HANDLE devHandle = NULL;

    if (!pMultipleCameraInfo) {
        printf("pMultipleCameraInfo is NULL!\n");
        return IMV_INVALID_PARAM;
    }

    for (index = 0; index < pMultipleCameraInfo->nOpenCameraNum; index++) {
        devHandle = NULL;

        // 创建设备句柄
        // Create Device Handle
        ret = IMV_CreateHandle(&devHandle, modeByIndex, (void *) &index);
        if (IMV_OK != ret) {
            printf("Create devHandle1 failed! index[%u], ErrorCode[%d]\n", index, ret);
            continue;
        }

        // 获取设备信息
        // Get device information
        ret = IMV_GetDeviceInfo(devHandle, &pMultipleCameraInfo->cameraInfo[openCameraNum].info);
        if (IMV_OK != ret) {
            printf("Get device info failed! index[%u], ErrorCode[%d]\n", index, ret);
        }

        // 打开相机
        // Open camera
        ret = IMV_Open(devHandle);
        if (IMV_OK != ret) {
            printf("Open camera1 failed! cameraKey[%s], ErrorCode[%d]\n", pMultipleCameraInfo->cameraInfo[openCameraNum].info.cameraKey, ret);

            // 销毁设备句柄
            // Destroy Device Handle
            IMV_DestroyHandle(devHandle);
            continue;
        }

        // 设置外部触发配置
        // Set external trigger config
        ret = setLineTriggerConf(devHandle);
        if (IMV_OK != ret) {
            continue;
        }

        pMultipleCameraInfo->cameraInfo[openCameraNum].handle    = devHandle;
        pMultipleCameraInfo->cameraInfo[openCameraNum].carmeraId = openCameraNum;
        pubImages[openCameraNum]                                 = nh.advertise<sensor_msgs::Image>("cam" + std::to_string(openCameraNum), 1);

        openCameraNum++;
    }

    pMultipleCameraInfo->nOpenCameraNum = openCameraNum;

    return IMV_OK;
}

int startGrabbing(MultipleCamera *pMultipleCameraInfo) {
    unsigned int index = 0;
    int ret            = IMV_OK;

    if (!pMultipleCameraInfo) {
        printf("pMultipleCameraInfo is NULL!\n");
        return IMV_INVALID_PARAM;
    }

    for (index = 0; index < pMultipleCameraInfo->nOpenCameraNum; index++) {
        // 注册数据帧回调函数
        // Register data frame callback function
        ret = IMV_AttachGrabbing(pMultipleCameraInfo->cameraInfo[index].handle, onGetFrame, (void *) &pMultipleCameraInfo->cameraInfo[index]);
        if (IMV_OK != ret) {
            printf("Attach grabbing failed! cameraKey[%s], ErrorCode[%d]\n", pMultipleCameraInfo->cameraInfo[index].info.cameraKey, ret);
            continue;
        }

        // 开始拉流
        // Start grabbing
        ret = IMV_StartGrabbing(pMultipleCameraInfo->cameraInfo[index].handle);
        if (IMV_OK != ret) {
            printf("Start grabbing failed! EcameraKey[%s], ErrorCode[%d]\n", pMultipleCameraInfo->cameraInfo[index].info.cameraKey, ret);
        }
    }

    return IMV_OK;
}

int stopGrabbing(MultipleCamera *pMultipleCameraInfo) {
    unsigned int index = 0;
    int ret            = IMV_OK;

    if (!pMultipleCameraInfo) {
        printf("pMultipleCameraInfo is NULL!\n");
        return IMV_INVALID_PARAM;
    }

    for (index = 0; index < pMultipleCameraInfo->nOpenCameraNum; index++) {
        // 判断设备是否正在拉流
        // Check whether device is grabbing or not
        if (IMV_IsGrabbing(pMultipleCameraInfo->cameraInfo[index].handle)) {
            // 停止拉流
            // Stop grabbing
            ret = IMV_StopGrabbing(pMultipleCameraInfo->cameraInfo[index].handle);
            if (IMV_OK != ret) {
                printf("Stop grabbing failed! EcameraKey[%s], ErrorCode[%d]\n", pMultipleCameraInfo->cameraInfo[index].info.cameraKey, ret);
            }
        }
    }

    return IMV_OK;
}

int closeDevice(MultipleCamera *pMultipleCameraInfo) {
    unsigned int index = 0;
    int ret            = IMV_OK;

    if (!pMultipleCameraInfo) {
        printf("pMultipleCameraInfo is NULL!\n");
        return IMV_INVALID_PARAM;
    }

    for (index = 0; index < pMultipleCameraInfo->nOpenCameraNum; index++) {
        // 判断设备是否已打开
        // Check whether device is opened or not
        if (IMV_IsOpen(pMultipleCameraInfo->cameraInfo[index].handle)) {
            // 关闭相机
            // Close camera
            ret = IMV_Close(pMultipleCameraInfo->cameraInfo[index].handle);
            if (IMV_OK != ret) {
                printf("Close grabbing failed! EcameraKey[%s], ErrorCode[%d]\n", pMultipleCameraInfo->cameraInfo[index].info.cameraKey, ret);
            }
        }

        // 销毁设备句柄
        // Destroy Device Handle
        ret = IMV_DestroyHandle(pMultipleCameraInfo->cameraInfo[index].handle);
        if (IMV_OK != ret) {
            printf("Destroy device Handle failed! EcameraKey[%s], ErrorCode[%d]\n", pMultipleCameraInfo->cameraInfo[index].info.cameraKey, ret);
        }
    }

    return IMV_OK;
}

// ***********结束： 这部分处理与SDK操作相机无关，用于显示设备列表 ***********
// ***********END: These functions are not related to API call and used to
// display device info***********

int main(int argc, char **argv) {
    ros::init(argc, argv, "multi_cam_node");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    ros::Rate loop_rate(100);
    pubIMU = nh.advertise<sensor_msgs::Imu>("/imu0", 1);

    SetupSignalHandler();

    std::cout.precision(19);

    // 设置相机时间和IMU的IP地址和端口
    TIMESYNC_TCPClient cam_node("169.254.0.122", 5001);
    std::thread time_thread(&TIMESYNC_TCPClient::timestamp_proc, &cam_node);
    TIMESYNC_TCPClient imu_node("169.254.0.122", 5003);
    std::thread imu_thread(&TIMESYNC_TCPClient::imu_proc, &imu_node);

    if (!cam_node.tcp_stream_ || !imu_node.tcp_stream_) {
        std::cerr << "Failed to connect to the time sync or IMU server." << std::endl;
        return -1;
    }

    while (!exit_flag && TIME_NOW == 0.0) {
        printf("Waiting for timestamp...\n");
        usleep(100 * 1000); // 等待时间戳被接收
    }

    if (exit_flag) {
        std::cout << "Exiting due to signal." << std::endl;
        return 0;
    }

    int ret = IMV_OK;
    MultipleCamera multipleCameraInfo;
    IMV_DeviceList deviceInfoList;

    memset(&multipleCameraInfo, 0, sizeof(multipleCameraInfo));

    // 发现设备
    // discover camera
    ret = IMV_EnumDevices(&deviceInfoList, interfaceTypeAll);
    if (IMV_OK != ret) {
        printf("Enumeration devices failed! ErrorCode[%d]\n", ret);
        getchar();
        return -1;
    }

    if (deviceInfoList.nDevNum < 1) {
        printf("no camera\n");
        getchar();
        return -1;
    }

    // 打印相机基本信息（序号,类型,制造商信息,型号,序列号,用户自定义ID,IP地址）
    // Print camera info (Index, Type, Vendor, Model, Serial number, DeviceUserID,
    // IP Address)
    displayDeviceInfo(deviceInfoList);

    usleep(2000 * 1000);

    multipleCameraInfo.nOpenCameraNum = deviceInfoList.nDevNum;

    do {
        // 打开相机
        // Open camera
        ret = openCamera(&multipleCameraInfo, nh, it);
        if (IMV_OK != ret) {
            break;
        }

        if (multipleCameraInfo.nOpenCameraNum == 0) {
            printf("no camera is opened!\n");
            break;
        }

        // 开始拉流
        // Start grabbing
        ret = startGrabbing(&multipleCameraInfo);
        if (IMV_OK != ret) {
            break;
        }

        while (!exit_flag && ros::ok()) {
            ros::spinOnce();
            loop_rate.sleep();
        }

        // 停止拉流
        // Stop grabbing
        ret = stopGrabbing(&multipleCameraInfo);
        if (IMV_OK != ret) {
            break;
        }

    } while (false);

    // 关闭相机
    // Close camera
    closeDevice(&multipleCameraInfo);

    std::cout << "All cameras closed." << std::endl;

    time_thread.join();
    imu_thread.join();

    return 0;
}