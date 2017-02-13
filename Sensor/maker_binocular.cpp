#include "maker_binocular.h"
//#include "DataProcess.h"
//#include "DataCapture.h"
#include <cyusb.h>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <iomanip>
#include <boost/concept_check.hpp>

//extern CDataCapture* m_pDataCapture;
//extern fstream time_fs;
using namespace std;

makerbinocular::makerbinocular()
{
    init();
}

makerbinocular::~makerbinocular()
{

    libusb_free_device_list(devs, 1);
    libusb_close(dev_handle);
    
    delete datain;
}


void makerbinocular::init()
{
    initialized = false;
    // init required data

    
    
    int r;
    int err;
    ssize_t cnt;

#if 1
    // initialize a library session
    r = libusb_init(&contex);
//    cnt= cyusb_open();
//    std::cout <<  "device " << r << std::endl;
    if (cnt < 0)
    {
        std::cout <<  "Init error!" <<  std::endl;
    }

    // set verbosity level to 3
    libusb_set_debug(contex,  3);
//    h1=cyusb_gethandle(0);
//    r=cyusb_kernel_driver_active(h1,0);
//    if (r < 0)
 //   {
 //       std::cout <<  "cyusb_kernel_driver_active error!" <<  std::endl;
 //   }
//    r = cyusb_claim_interface(h1, 0);
//    if (r < 0)
//    {
//        std::cout <<  "cyusb_claim_interface error!" <<  std::endl;
//    }
    
    cnt = libusb_get_device_list(contex,  &devs);

    if (cnt < 0)
    {
        std::cout <<  "Get devices error" <<  std::endl;
    }
    else
    {
        std::cout <<  "Get " <<  cnt <<  " devices in total" <<  std::endl;
    }

    // found cypress usb device
    bool idVendorAndProductfound =  false;
    for (int i = 0; i < cnt; i++)
    {
        device = devs[i];
        err = libusb_get_device_descriptor(device, &desc);
//        err = cyusb_get_device_descriptor(h1, &desc);

        if (err < 0)
        {
            std::cout <<  "failed to get desc" <<  std::endl;
            return; 
        }


        if (desc.idVendor ==  0x04b4 && desc.idProduct ==  0x1005)
        {
            std::cout <<  "============================================" << std::endl;
            printf("Found cypress usb device: idVendor 0x%04x idProduct: 0x%04x\r\n", desc.idVendor,desc.idProduct);
            std::cout <<  "============================================" <<  std::endl;
            idVendorAndProductfound = true;
            break;
        }
        if (desc.idVendor ==  0x2014 && desc.idProduct ==  0x0117)
        {
        std::cout <<  "============================================" << std::endl;
        printf("Found cypress usb device: idVendor 0x%04x idProduct: 0x%04x\r\n", desc.idVendor,desc.idProduct);
        std::cout <<  "============================================" <<  std::endl;
        idVendorAndProductfound = true;
        break;
        }
    }

    if (idVendorAndProductfound ==  false)
    {
        std::cout <<  "============================================" <<  std::endl;
        std::cout <<  "Error: Can not found the device, please check the idVendor and idProduct!" <<  std::endl;
        std::cout <<  "============================================" <<  std::endl;
        return ;
    }
    // get cypress usb config desc
    libusb_get_config_descriptor(device,  0,  &config);
//    cyusb_get_config_descriptor(h1,0,&config);
    // 

    if ((int)config->bNumInterfaces > 1)
    std::cout <<  "too many interfaces" <<  std::endl;

    const struct libusb_interface * inter;
    const struct libusb_interface_descriptor * interdesc;
    const struct libusb_endpoint_descriptor * epdesc;

    inter = &config->interface[0];
    interdesc = &inter->altsetting[0];

    if ((int) interdesc->bNumEndpoints > 2)
    std::cout <<  "too many endpoints" <<  std::endl;

    for (int j = 0; j < interdesc->bNumEndpoints; j++)
    {
        epdesc = &interdesc->endpoint[j];

        if ((epdesc->bEndpointAddress) & 0x80)
        {
            bulk_ep_in = epdesc->bEndpointAddress;
            printf("Hints: Get Built in endpoint: 0x%02x\n" ,  epdesc->bEndpointAddress);
            printf("Max packetsize is %d \n",  epdesc->wMaxPacketSize);
        }
    }
#endif
//    err = libusb_open(device, &dev_handle);
    err = cyusb_open();
    if (err < 0)
    {
        printf("open device failed\n");
//        libusb_free_device_list(devs, 1);
//        libusb_close(dev_handle);
        cyusb_close();
        return ;
    }
    h1=cyusb_gethandle(0);
    printf("h1 get vendor %d \n",cyusb_getvendor(h1));

/*    if (libusb_kernel_driver_active(dev_handle,  0) ==  1)
    {
        printf("Kernel Driver Active\n" );
        if (libusb_detach_kernel_driver(dev_handle,  0 ) ==  0)
        printf("Kernal Driver Detached\n");
    }
    */
if(cyusb_kernel_driver_active(h1,0)!=0)
 {
    printf("kernel driver active, exitting");
    cyusb_close();
    return;
}

//    err = libusb_claim_interface(dev_handle,  0);
    err = cyusb_claim_interface(h1, 0);
    if (err != 0)
    {
        printf("can not claim interface");
//        libusb_free_device_list(devs, 1);
//        libusb_close(dev_handle);
        cyusb_close();
        return; 
    }
    
    initialized = true;
}


void makerbinocular::get_frame(cv::Mat &left_image, cv::Mat &right_image)
{
//    buffer_size = 640 * 480 * 2 + 2048;
    buffer_size = (1284*480+200)*2;
    datain = new u8[buffer_size];
    bool b_header=false,b_imu=false;
  
//    int high_temp=0;
    
    if (left_image.rows !=  480 |  left_image.cols !=  640 |  right_image.rows !=  480 |  right_image.cols !=  640)
    {
        std::cout <<  left_image.rows <<  left_image.cols <<  std::endl;
        std::cout <<  "Error: the image size should be: 640 x 480" <<  std::endl;
        
    }
    int transferd;


    
//    fs << "hello,world" << std::endl;

    unsigned char * pcS = (u8*) (datain + 32);
   
    
//    printf("init over\n");
//    int error = libusb_bulk_transfer(dev_handle, bulk_ep_in, datain, buffer_size, &transferd, 1000);
    int error = cyusb_bulk_transfer(h1, 0x86, datain, buffer_size, &transferd,1000);
//  std::cout <<  transferd <<  std::endl;
//    fs<<transferd << std::endl;
//    std::cout<<std::hex<<datain;
//    printf("cyusb_bulk_transfer over\n");
//    m_pDataCapture->Open(NULL,480,640);
//memcpy(m_pInData+m_iCount,lpData,dwSize);
/*
    for(int i=0;i<transferd;i++)
    if(datain[i]==0x33&&datain[i+1]==0xcc&&datain[i+14]==0x22&&datain[i+15]==0xdd)
    {
        unsigned char imTimeStamp[4];
        long int timestamp=0;
        imTimeStamp[0]=datain[i+10];
		imTimeStamp[1]=datain[i+11];
		imTimeStamp[2]=datain[i+12];
		imTimeStamp[3]=datain[i+13];
        timestamp=(long)((imTimeStamp[0]<<24)+(imTimeStamp[1]<<16)+(imTimeStamp[2]<<8)+(imTimeStamp[3]));
        printf("%ld 0x%02x%02x%02x%02x\n",timestamp,imTimeStamp[0],imTimeStamp[1],imTimeStamp[2],imTimeStamp[3]);
        time_fs<<timestamp<<endl;
    }

*/

//    m_pDataCapture->Input(datain,transferd);
//    printf("get number %d \n",transferd);

    std::cout << "."<<endl;
//    delete datain;
//    printf(".");

    

    if (transferd ==  0)
    {
        std::cout <<  "============================================" <<  std::endl;
        std::cout <<  "Warning: No data received ! Please check the buld endpoint address" <<  std::endl;
        std::cout <<  "============================================" <<  std::endl;
    }

#if 0
    if (error == 0) 
    {
         std::cout << "received " <<  std::endl;

        // frame header
        if (((*datain) ==  0x01) & (*(datain+1) ==  0xfe) & (*(datain+2) ==  0x01) & (*(datain + 3) ==  0xfe))
        {
            //std::cout <<  "get frame header" <<  std::endl;
        }
        
        /*
        else
        {
            std::cout << "========================================" <<  std::endl;
            std::cout <<  "Warning: Frame header error!" <<  std::endl;
            std::cout << "=======================================" <<  std::endl;
            usleep(30000);
            return;
        }
        */

        int cnt_y,  cnt_x;
        for (cnt_y  = 0; cnt_y < 480; cnt_y++)
        {
            for (cnt_x = 0; cnt_x < 640; cnt_x++)
            {
                // left image
                left_image.at<uchar>(479 - cnt_y, cnt_x) = *(pcS + cnt_y * 1280 + cnt_x * 2);

                // right image
                right_image.at<uchar>(479 - cnt_y, cnt_x) = *(pcS + cnt_y * 1280 + cnt_x * 2 + 1);
            }
        }

    }
#endif
    

}
