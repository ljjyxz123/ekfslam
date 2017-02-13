#ifndef MAKER_BINOCULAR_H
#define MAKER_BINOCULAR_H
#include <opencv2/opencv.hpp>
#include <libusb-1.0/libusb.h>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <cyusb.h>

#define FALSE   0
#define false   0
#define TRUE    1
#define true    1

typedef unsigned char u8;


class makerbinocular {
public:
    makerbinocular();
    
    ~makerbinocular();
    
    void init();
    
    void get_frame(cv::Mat & left_image,  cv::Mat & right_image);
    
    bool is_initialized() {return initialized;}
    int buffer_size;
    unsigned char * datain;
    cyusb_handle *h1=NULL;
    
private:
    
    libusb_device ** devs;
    libusb_context *contex = NULL;

   
    
    struct libusb_device_descriptor desc;
    struct libusb_config_descriptor * config;
    
    libusb_device *device;
    libusb_device_handle *dev_handle;

//    unsigned char * datain;
    
    u8 bulk_ep_in;
//    int buffer_size;

//    std::fstream fs("/home/leo/下载/DUAL_CAM_GLOBAL_V2用户资料/Driver/Linux/bin");
//    std::fstream fs;

    bool initialized;
  };

#endif
