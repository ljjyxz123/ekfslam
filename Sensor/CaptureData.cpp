
//#include "DataProcess.h"
//#include "DataCapture.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <assert.h>
#include <pthread.h>
#include <iostream>
#include <cyusb.h>
#include <opencv2/opencv.hpp>
//#include<fiostream.h>

#include "MsgQueue.h"
#include "maker_binocular.h"
#include "CaptureData.h"
#include "CMutex.h"

//extern CDataCapture* m_pDataCapture;
//extern fstream time_fs;
using namespace std;




CaptureData::CaptureData()
{
    
    Imu_char_data=new unsigned char[180];
    Img_data=new unsigned char[480*1280];
//    Thread_q = new MsgQueue;

    imu_strlen=180;
    weight_data.WeightFlag=0;
    weight_data.WeightValue=0.6;
    Imu_num=0;
    Imu_data.DataNum=0;
    memset(Imu_data.ImuTimeStamp,0,sizeof(Imu_data.ImuTimeStamp));
    memset(Imu_data.ImuValue,0,sizeof(Imu_data.ImuValue));
//    imu_fp=fopen("/home/leo/project/usb_camera_imu/disruptor/imu.txt","w+");
//    img_fp=fopen("/home/leo/project/usb_camera_imu/disruptor/img.txt","w+");
//    printf("CaptureData_main\n");
//    ImuFile=fopen("/home/leo/project/usb_camera_imu/disruptor_3/imu.txt","w+");
//    fprintf(ImuFile,"start");
//    fclose(ImuFile);

    CaptureData_main();

//    LeftImgFile=fopen("/home/leo/project/usb_camera_imu/disruptor/LeftImg.txt","w+");
//    RightImgFile=fopen("/home/leo/project/usb_camera_imu/disruptor/RightImg.txt","w+");
}

CaptureData::~CaptureData()
{
    if(Imu_char_data!=NULL)
    {

        delete[]Imu_char_data;
    }
    if(Img_data!=NULL)
    {
        delete[]Img_data;
    }
//    fclose(ImuFile);


}

int CaptureData::Get_CaptureData(USBElements *ImuCapture,long long OutImgTimeStamp)
{
//    FILE *imu_fp;
//    FILE *img_fp;
    int imu_datanum=0,imu_datanum_diff=0;
    long long           ImuTimeStamp_tmp[10];
    float               ImuValue_tmp[10][6];
    int i=0,j=0;
    assert(ImuCapture);
    //assert(ImgCapture);
//    imu_fp=fopen("/home/leo/project/usb_camera_imu/disruptor/imu.txt","w+");
//    img_fp=fopen("/home/leo/project/usb_camera_imu/disruptor/img.txt","w+");
//    printf("#############Imu_num is %d %d\n",Imu_num,Imu_num*imu_strlen*sizeof(long long));
    p_Mutex.Lock();
//    Imu_data.DataNum=Imu_num;
    memcpy(ImuCapture,&Imu_data,sizeof(Imu_data));
//    ImuCapture->DataNum=Imu_num-10;
//    if(ImuCapture->DataNum==0)
//    {
//        return 1;
//    }
//    ImuCapture->ImgTimeStamp=Imu_data.ImgTimeStamp;
    if(ImuCapture->ImgTimeStamp==0)
    {
        return 1;
    }
//    ImuCapture->ImgTimeStamp=Imu_data.ImgTimeStamp;
//    cout<<"ImuCapture->ImgTimeStamp"<<ImuCapture->ImgTimeStamp<<endl;

    ImuCapture->DataNum=Imu_num-10;
//    imu_datanum=i;
//    imu_datanum_diff=Imu_num-i;
    imu_datanum_diff =10;
//    printf("ImuCapture[0] is %lld,%f,%f\n",ImuCapture[0],ImuCapture[1],Imu_data[1]);
//    printf("#########ImgTimestamp %lld#######\n",ImuCapture->ImuTimeStamp[0]);
//    cout<<"&&&&&&&&&&&&&ImgTimeStamp ="<<ImuCapture->ImgTimeStamp<<"&&&&&&&&&&"<<Imu_num<<endl;
//    memcpy(ImgCapture,&ImgTimestamp,sizeof(ImgTimestamp));
    OutImgTimeStamp = ImgTimestamp;
//    cout<<"######ImgTimestamp  "<<ImgTimestamp<<endl;
    //memcpy(ImgCapture,Img_data,480*1280);
    p_Mutex.UnLock();

//    printf("%s [%d]\n",ImuCapture,sizeof(ImuCapture));
//    fprintf(img_fp,"%s",Img_data);
//    memset(Imu_char_data,0,sizeof(ImuCapture));

    for(i=0;i<imu_datanum_diff;i++)
    {
        Imu_data.ImuTimeStamp[i]=Imu_data.ImuTimeStamp[Imu_data.DataNum-imu_datanum_diff+i];
        for(j =0;j<6;j++)
            Imu_data.ImuValue[i][j]=Imu_data.ImuValue[Imu_data.DataNum-imu_datanum_diff+i][j];
    }
    
    Imu_num=10;
    Imu_data.DataNum=10;
    
//    memset(Imu_data.ImuTimeStamp+10,0,sizeof(Imu_data.ImuTimeStamp));
//    memset(Imu_data.ImuValue+10,0,sizeof(Imu_data.ImuValue));

    return ImuCapture->DataNum;

}





void CaptureData::ShowCaptureData(unsigned char*MessageBuffer,int messagebuffer_len,unsigned long *timestamp_last,int run_time)
{
    unsigned char imTimeStamp[4];
    char datatoIMU[2];
	unsigned int timestamp=0; 
    unsigned long long timestamp_f=0; 
    unsigned char* m_pOutData=new unsigned char[480*1280];
    unsigned char* m_pOutDataLeft=new unsigned char[480*640];
    unsigned char* m_pOutDataRight=new unsigned char[480*640];
    unsigned int high_index=0,high_temp=0;
    short gx=0,gy=0,gz=0,tm=0,ax=0,ay=0,az=0;
    float GyroValue[3],AccelValue[3];
    char LeftImgFilePath[512];
    char RightImgFilePath[512];
//    cout<<"##############ShowCaptureData start";


    for(int j=0;j<messagebuffer_len;j++)
    {
        if(MessageBuffer[j]==0x33&&MessageBuffer[j+1]==0xcc&&MessageBuffer[j+14]==0x22&&MessageBuffer[j+15]==0xdd)
        {
//            cout<<"ShowCaptureData start---1"<<"j="<<j<<endl;   
            imTimeStamp[0]=MessageBuffer[j+10];
            imTimeStamp[1]=MessageBuffer[j+11];
            imTimeStamp[2]=MessageBuffer[j+12];
            imTimeStamp[3]=MessageBuffer[j+13];
            timestamp=(unsigned int)((imTimeStamp[0]<<24)+(imTimeStamp[1]<<16)+(imTimeStamp[2]<<8)+(imTimeStamp[3]));
    //					memcpy(p_msg,imTimeStamp,sizeof(imTimeStamp));
//            printf("%d******%d\n",timestamp,timestamp-(*timestamp_last));
            ImgTimestamp=(long long)timestamp*100000;
            Imu_data.ImgTimeStamp=ImgTimestamp;
            if((timestamp-(*timestamp_last))!=500)
            {
                cout<<"##########################################*$$$*$*$$$$$#error##################################"<<timestamp<<"#######"<<*timestamp_last<<endl;
                *timestamp_last=timestamp;
//                return ;
            }
            *timestamp_last=timestamp;
//            LeftImgFile=fopen("/home/leo/project/usb_camera_imu/disruptor_3/Left/timestamp.txt","a+");
//            RightImgFile=fopen("/home/leo/project/usb_camera_imu/disruptor_3/Right/timestamp.txt","a+");
            timestamp_f=(long long)timestamp*100000;
//            sprintf(LeftImgFilePath,"/home/leo/project/usb_camera_imu/disruptor_3/Left/%lld.jpg",timestamp_f);
//            sprintf(RightImgFilePath,"/home/leo/project/usb_camera_imu/disruptor_3/Right/%lld.jpg",timestamp_f);
//            fprintf(LeftImgFile, "%lld.jpg  %lld \n",timestamp_f,timestamp_f);
//            fprintf(RightImgFile, "%lld.jpg  %lld \n",timestamp_f,timestamp_f);
            j=j+16;
//            fclose(LeftImgFile);
//            fclose(RightImgFile);

//            ImuFile<<timestamp<<endl;
        }
    //                            printf("3333 MessageBuffer\n");

        if(MessageBuffer[j]==0x66&&MessageBuffer[j+1]==0xdd&&MessageBuffer[j+182]==0x44&&MessageBuffer[j+183]==0xbb)
        {
            
            memcpy(Imu_char_data,MessageBuffer+j+2,imu_strlen);


            p_Mutex.Lock();
//            ImuFile=fopen("/home/leo/project/usb_camera_imu/disruptor_3/imu/imu.txt","a+");
            

            for (int k = 0; k < imu_strlen; k+=18) {
                imTimeStamp[0]=Imu_char_data[k];
                imTimeStamp[1]=Imu_char_data[k+1];
                imTimeStamp[2]=Imu_char_data[k+2];
                imTimeStamp[3]=Imu_char_data[k+3];
                timestamp=(unsigned int)((imTimeStamp[0]<<24)+(imTimeStamp[1]<<16)+(imTimeStamp[2]<<8)+(imTimeStamp[3]));
				Imu_data.ImuTimeStamp[Imu_num]=(long long)timestamp*100000;
//                cout<<"#########ImuTimeStamp ="<<Imu_data.ImuTimeStamp[Imu_num]<<"######"<<Imu_num<<endl;
                Imu_data.ImuValue[Imu_num][3]=GyroValue[0]=(short)((Imu_char_data[k+12]<<8)+Imu_char_data[k+13])*3.14159/(65.5*180.0);
                Imu_data.ImuValue[Imu_num][4]=GyroValue[1]=(short)((Imu_char_data[k+14]<<8)+Imu_char_data[k+15])*3.14159/(65.5*180.0);
                Imu_data.ImuValue[Imu_num][5]=GyroValue[2]=(short)((Imu_char_data[k+16]<<8)+Imu_char_data[k+17])*3.14159/(65.5*180.0);
                Imu_data.ImuValue[Imu_num][0]=AccelValue[0]=(short)((Imu_char_data[k+4]<<8)+Imu_char_data[k+5])/16384.0*GValue;
                Imu_data.ImuValue[Imu_num][1]=AccelValue[1]=(short)((Imu_char_data[k+6]<<8)+Imu_char_data[k+7])/16384.0*GValue;
                Imu_data.ImuValue[Imu_num][2]=AccelValue[2]=(short)((Imu_char_data[k+8]<<8)+Imu_char_data[k+9])/16384.0*GValue;
//                printf("%f %f %f %f %f %f %f\n",Imu_data[0],Imu_data[1],Imu_data[2],Imu_data[3],Imu_data[4],Imu_data[5],Imu_data[6]);

//                printf(" %f %f %f %f %f %f\n",AccelValue[0],AccelValue[1],AccelValue[2],GyroValue[0],GyroValue[1],GyroValue[2]);
                timestamp_f=(long long)timestamp*100000;

/*             *****************加权滤波***********
                if(weight_data.WeightFlag==0)
                {
                    for(int kk=0;kk<6;kk++)
                    {
                        weight_data.LastImuValue[kk]=Imu_data.ImuValue[Imu_num][kk];
                    }
                    weight_data.WeightFlag=1;
                }
                if(weight_data.WeightFlag==1)
                {
                    for(int kk=0;kk<6;kk++)
                    {
                       Imu_data.ImuValue[Imu_num][kk]=Imu_data.ImuValue[Imu_num][kk]*weight_data.WeightValue+weight_data.LastImuValue[kk]*(1-weight_data.WeightValue);
                       weight_data.LastImuValue[kk]=Imu_data.ImuValue[Imu_num][kk];
                    }
                }********************************/
//               fprintf(ImuFile, "%lld %f %f %f %f %f %f\n",timestamp_f,Imu_data.ImuValue[Imu_num][3],Imu_data.ImuValue[Imu_num][4],Imu_data.ImuValue[Imu_num][5],Imu_data.ImuValue[Imu_num][0],Imu_data.ImuValue[Imu_num][1],Imu_data.ImuValue[Imu_num][2]);	
//                printf("%lld %f %f %f %f %f %f\r",timestamp_f,GyroValue[0],GyroValue[1],GyroValue[2],AccelValue[0],AccelValue[1],AccelValue[2]);
//                printf("%lld %f %f %f %f %f %f\n",timestamp_f,Imu_data.ImuValue[Imu_num][3],Imu_data.ImuValue[Imu_num][4],Imu_data.ImuValue[Imu_num][5],Imu_data.ImuValue[Imu_num][0],Imu_data.ImuValue[Imu_num][1],Imu_data.ImuValue[Imu_num][2]);
                Imu_num++;
            }
//            printf("%lld %f %f %f %f %f %f\n",Imu_data[0],Imu_data[1],Imu_data[2],Imu_data[3],Imu_data[4],Imu_data[5],Imu_data[6]);
//            fclose(ImuFile);
            p_Mutex.UnLock();

            Imu_data.DataNum=Imu_num;
//            printf("imu len %d\n",Imu_num);
            if(Imu_num>=599)
            {
                Imu_num=0;
            }
            j+=184;
        }
        if(MessageBuffer[j]==0x55&&MessageBuffer[j+1]==0xaa)
        {
            
            
            high_index=MessageBuffer[j+2];
            high_index=high_index<<8;
            high_index+=MessageBuffer[j+3];
            
    //							printf("high_index is %d  0x%02x%02x len %d\n",high_index,MessageBuffer[j+2],MessageBuffer[j+3],j-last_j);
    //							last_j=j;
    //                                printf("%d",high_index);
            memcpy(m_pOutData+high_index*1280,MessageBuffer+j+4,1280);
            memcpy(m_pOutDataLeft+high_index*640,MessageBuffer+j+4,640);
            memcpy(m_pOutDataRight+high_index*640,MessageBuffer+j+4+640,640);
            
            j=j+1283;
            
    //								printf("*******high_index is %d\n",high_index);
            
            if(high_index==479)
            {
    //										printf("high_temp is %d,high_index is %d\n",high_temp,high_index);
            
//                cv::Mat frame(480,1280,CV_8UC1,m_pOutData);
//		namedWindow("Left", 0);
//		namedWindow("Right", 0);
//            	cv::Mat frame(480,640,CV_8UC1,m_pOutDataRight);
//            	cv::Mat frame1(480,640,CV_8UC1,m_pOutDataLeft);
//                cv::imwrite(LeftImgFilePath, frame);
//                cv::imwrite(RightImgFilePath, frame1);
                //cv::Mat colored(g_height,g_width,CV_8UC3);
                //cv::applyColorMap(frame,colored,cv::COLORMAP_JET)
//                cv::imshow("Right",frame1);
//            	cv::imshow("Left",frame);
                //cv::imshow("color",colored);
//                cv::waitKey(1);
                p_Mutex.Lock();
                Imu_data.ImgTimeStamp=ImgTimestamp;
                memcpy(Imu_data.LeftImgBuffer,m_pOutDataLeft,480*640);
                memcpy(Imu_data.RightImgBuffer,m_pOutDataRight,480*640);
                p_Mutex.UnLock();
//                memset(m_pOutData,0,480*1280);
/*
                cout<<"delete[3]"<<endl;
                delete[]m_pOutData;
                cout<<"delete[4]"<<endl;
                delete[]m_pOutDataLeft;
                cout<<"delete[5]"<<endl;
                delete[]m_pOutDataRight;
                cout<<"delete[6]"<<endl;*/
//                memset(MessageBuffer,0,2048000);
                high_temp=high_index;
            }        

        }

    }
    delete[]m_pOutData;
    delete[]m_pOutDataLeft;
    delete[]m_pOutDataRight;
//    cout<<"ShowCaptureData end"<<endl;
}

void* CaptureData::_RunThreadA(void* arg)
{
    unsigned int code=0;
    unsigned char* p_msg ;
    unsigned char imTimeStamp[4];
	unsigned long timestamp=0;
    CaptureData *CaptureData_this=(CaptureData *)arg;
	
	 makerbinocular m_makerbinocular;
//	m_makerbinocular.buffer_size = 1233408;
	m_makerbinocular.buffer_size = 1024000;
//	m_makerbinocular.buffer_size = 512;
    m_makerbinocular.datain = new u8[m_makerbinocular.buffer_size];
	p_msg=m_makerbinocular.datain;
  
    while(1)
    {
//        m_makerbinocular.datain = new u8[m_makerbinocular.buffer_size];
//	    p_msg=m_makerbinocular.datain;
		int error = cyusb_bulk_transfer(m_makerbinocular.h1, 0x86, m_makerbinocular.datain, m_makerbinocular.buffer_size, &(CaptureData_this->transferd),1000);
//		p_msg=m_makerbinocular.datain;
        
        for(int i=0;i<(CaptureData_this->transferd);i++)
        {
            if(m_makerbinocular.datain[i]==0x33&&m_makerbinocular.datain[i+1]==0xcc&&m_makerbinocular.datain[i+14]==0x22&&m_makerbinocular.datain[i+15]==0xdd)
            {
//					unsigned char imTimeStamp[4];
//					unsigned long timestamp=0;
                imTimeStamp[0]=m_makerbinocular.datain[i+10];
                imTimeStamp[1]=m_makerbinocular.datain[i+11];
                imTimeStamp[2]=m_makerbinocular.datain[i+12];
                imTimeStamp[3]=m_makerbinocular.datain[i+13];
                timestamp=(unsigned long)((imTimeStamp[0]<<24)+(imTimeStamp[1]<<16)+(imTimeStamp[2]<<8)+(imTimeStamp[3]));
//					memcpy(p_msg,imTimeStamp,sizeof(imTimeStamp));
//                printf("#########send[%d][%d][%d]\n",timestamp,i,code);
//					printf(">>>>>>>%s is Running....send data to message queue... %s\n",p_thread_name,p_msg);
                
//					p_msg_send->sendMsg(code, p_msg);
//					code++;
        //				time_fs<<timestamp<<endl;
//                sprintf(p_msg,"%ld",timestamp);
            }
        }	
//        printf(">>>>>>>A is Running....send data to message queue...\n");
        CaptureData_this->Thread_q->sendMsg(code,(void *)p_msg);
        code++;
//        sleep(1);
//        thread_flag=1;
    }
}
void* CaptureData::_RunThreadB(void* arg)
{
    unsigned int code=0;
    void* p_msg;
    unsigned char imTimeStamp[4];
	unsigned long timestamp=0,timestamp_last=0;
    unsigned char msg_data[2048000],msg_data_temp[1024000];
	int last_i=0,last_j=0,lastmsg_lan=0;
	int message_len=0,last_message_len=0;
	int messagebuffer_len=0;
	unsigned int high_index=0,high_temp=0;
	int img_flag=0,step_1=0;
    CaptureData *CaptureData_this=(CaptureData *)arg;
    
    int Frist_RUN=0;
    int ret =0;
    int i=0;
	
	unsigned char *MessageBuffer=new unsigned char[2048000];

//    sleep(1);
//    printf("_RunThreadB \n");
    while(1)
    {
//        while(thread_flag==0);

//        thread_flag=0;
        ret=CaptureData_this->Thread_q->recvMsg(code,p_msg);
//        cout<<"ret ="<<ret<<endl;

//        printf("<<<<<<<%s is Running....recv data to message queue...\n",(char*)arg,(char*)p_msg);
        memcpy(msg_data+lastmsg_lan,p_msg,1024000);
        delete[]p_msg;

        for(i=0;i<(CaptureData_this->transferd+lastmsg_lan);i++)
        {
            
            if((msg_data[i]==0x33)&&(msg_data[i+1]==0xcc)&&(msg_data[i+14]==0x22)&&(msg_data[i+15]==0xdd))
            {
//					unsigned char imTimeStamp[4];
//					unsigned long timestamp=0;
//					high_temp=0;
                img_flag=1;

                imTimeStamp[0]=msg_data[i+10];
                imTimeStamp[1]=msg_data[i+11];
                imTimeStamp[2]=msg_data[i+12];
                imTimeStamp[3]=msg_data[i+13];
                timestamp=(unsigned long)((imTimeStamp[0]<<24)+(imTimeStamp[1]<<16)+(imTimeStamp[2]<<8)+(imTimeStamp[3]));
//                cout<<"########timestamp :"<<timestamp<<"$$$$$$$$$"<<i<<endl;



//                last_i=i;
                i+=923719;
                Frist_RUN++;
//                if(Frist_RUN>=3)
//                Frist_RUN=3;
                if(i>(CaptureData_this->transferd+lastmsg_lan))
                {
                    lastmsg_lan=CaptureData_this->transferd+lastmsg_lan-i+923719;
                    memset(msg_data_temp,0,sizeof(msg_data_temp));
                    memcpy(msg_data_temp,msg_data+i-923719,lastmsg_lan);
                    memset(msg_data,0,sizeof(msg_data));
                    memcpy(msg_data,msg_data_temp,lastmsg_lan);
                }
                if((i)<=(CaptureData_this->transferd+lastmsg_lan))
                {
                    memcpy(MessageBuffer,msg_data+i-923719,923720);
                    CaptureData_this->ShowCaptureData(MessageBuffer,923720,&timestamp_last,Frist_RUN);

                }

//					timestamp=(unsigned long)((MessageBuffer[10]<<24)+(MessageBuffer[11]<<16)+(MessageBuffer[12]<<8)+(MessageBuffer[13]));
//					printf("MessageBuffer timerstamp %d  messagebuffer 0x%02x%02x%02x%02x\n",timestamp,MessageBuffer[10],MessageBuffer[1],MessageBuffer[12],MessageBuffer[13]);


            }
        }
//		cout<<"#########for (i) end"<<endl;	
    }
}
void* CaptureData::_RunThreadC(void* arg)
{
    USBElements Imu_buffer;
    unsigned char Img_Leftbuffer[640*480];
    unsigned char Img_Rigntbuffer[640*480];
    unsigned char Img_buffer[480*1280];
    long long Img_timestamp=0;
    CaptureData *CaptureData_this=(CaptureData *)arg;
    while(1)
    {
        sleep(2);
//        printf("Get_CaptureData\n");
        CaptureData_this->Get_CaptureData(&Imu_buffer,Img_timestamp);
        printf("Datanum %d,ImuTimeStamp %lld,ImuValue[0] id %f,ImgTimeStamp %lld\n",Imu_buffer.DataNum,Imu_buffer.ImuTimeStamp[0],Imu_buffer.ImuValue[0][0],Imu_buffer.ImgTimeStamp);
    }
}
int CaptureData::CaptureData_main()
{

    
    
    char input[] = "ThreadA";
    char inputB[] = "ThreadB";
//    setMsgQueue(Thread_q);
    //(void*)&input
//    printf("CaptureData_main start\n");
    int retA = pthread_create(&threadIdA, NULL, _RunThreadA, this);
//    sleep(10);
    int retB = pthread_create(&threadIdB, NULL, _RunThreadB, this);

//    int retC = pthread_create(&threadIdC, NULL, _RunThreadC, this);
/*
    if(retB != 0)
    {
        cout<< "create thread error"<<endl;
    }
 */
//    cout<<"main thread running"<<endl;
//    pthread_join(threadIdA,NULL);
//    pthread_join(threadIdB,NULL);
//    pthread_join(threadIdC,NULL);
 
    return 0;
}
