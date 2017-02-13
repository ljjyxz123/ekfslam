
#ifndef _MsgQueue_h_
#define _MsgQueue_h_


#include <deque>
#include <pthread.h>
#include <semaphore.h>

#include "Sem.h"
//#include "CMsgQueue.h"
//#include "CMutex.h"
//#include "CCountingSem.h"
typedef struct {
    unsigned int        msg_code;
    void                *p_message;
} Elements;

class MsgQueue
{
	public:
		MsgQueue();
		~MsgQueue();

		bool recvMsg(unsigned int &m_msg_code,void *&p_msg);
		bool sendMsg(unsigned int m_msg_code,void *p_msg);

	private:

		 std::deque<Elements>            m_queue;

		 
//		CMutex	*p_mutex;
//		CCountingSem *p_sem;
//        pthread_mutex_t             m_mutex;
//        pthread_mutexattr_t         m_mtex_attr;
        CCountingSem                *p_sem;
//        sem_t		m_sem;

};


#endif