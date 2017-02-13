#include <stdio.h>
#include <stdlib.h>

#include <string.h>
#include <unistd.h>

#include "MsgQueue.h"
#include "Sem.h"

//#include "COperatingSystemFactory.h"
extern int transferd;

MsgQueue::MsgQueue()
{

//    m_mutex = new pthread_mutex_t;
//    pthread_mutexattr_init(&m_mtex_attr);
//    pthread_mutex_init(&m_mutex, &m_mtex_attr);
//	m_mutex=COperatingSystemFactory::newMutex("Msg Mutex");
	p_sem=new CCountingSem();


}



MsgQueue::~MsgQueue()
{

}





bool MsgQueue::recvMsg(unsigned int &m_msg_code,void *&p_msg)
{

 	bool result;
    	Elements queue_element;
	p_sem->Get();

//	p_mutex->Lock();
   
	if (m_queue.empty()) {
	    
//		p_mutex->UnLock();
//        printf("empty\n");
	    	return 2;
	    
	}


	queue_element = m_queue.front();
	m_queue.pop_front();
    

	m_msg_code = queue_element.msg_code;
	p_msg = queue_element.p_message;
//    printf("recvMsg [%d]\n",queue_element.msg_code);
//	p_mutex->UnLock();

    return true;


}




bool MsgQueue::sendMsg(unsigned int m_msg_code,void *p_msg)
{
	bool result;
    	Elements queue_element;
	void *p_data;
	unsigned char *data;
	unsigned char imTimeStamp[4];
	int timestamp;

	queue_element.p_message=new unsigned char[1024000];

	queue_element.msg_code=m_msg_code;
	memcpy(queue_element.p_message,p_msg,1024000);
//	queue_element.p_message=p_msg;
//	p_mutex->Lock();

	m_queue.push_back(queue_element);
//   printf("sendMsg [%d]\n",m_msg_code);
//	p_mutex->UnLock();
	p_sem->Post();

	


    return true;


}


