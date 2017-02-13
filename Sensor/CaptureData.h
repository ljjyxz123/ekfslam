#ifndef _CaptureData_h_
#define _CaptureData_h_
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
//#include <fstream.h>
#include "CMutex.h"
#include "MsgQueue.h"
#include "maker_binocular.h"


//#include "CMsgQueue.h"
//#include "CMutex.h"
//#include "CCountingSem.h"
typedef struct {
    unsigned int        DataNum;
    long long           ImuTimeStamp[600];
    float               ImuValue[600][6];
    long long           ImgTimeStamp;
    unsigned char       LeftImgBuffer[640*480];
    unsigned char       RightImgBuffer[640*480];
} USBElements;

typedef struct {
    bool                WeightFlag=0;
    float               WeightValue=0.6;
    float               LastImuValue[6];
} WeightElements;

class CaptureData
{
	public:
		CaptureData();
		~CaptureData();

        int Get_CaptureData(USBElements *ImuCapture,long long OutImgTimeStamp);

        int Imu_num;


        unsigned char * Imu_char_data;
        USBElements Imu_data;

        unsigned char * Img_data;
        CMutex p_Mutex;
        int transferd;
        int imu_strlen=180;

    private:
        MsgQueue *Thread_q=new MsgQueue;
        pthread_t threadIdA;
        pthread_t threadIdB;
        pthread_t threadIdC;
//        ofstream ImuFile("/home/leo/project/usb_camera_imu/disruptor/imu.txt",ios::out);
//        fstream LeftImgFile;
//        fstream RightImgFile;
        FILE* ImuFile;
        FILE* ImgFile;
        FILE* LeftImgFile;
        FILE* RightImgFile;
        long long ImgTimestamp;
        float GValue=9.81;
        WeightElements weight_data;

        void ShowCaptureData(unsigned char*MessageBuffer,int messagebuffer_len,unsigned long *timestamp_last,int run_time);
        
        static void* _RunThreadA(void *arg);
        static void* _RunThreadB(void *arg);
        static void* _RunThreadC(void *arg);
        int CaptureData_main();

};


#endif