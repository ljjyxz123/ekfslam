

#ifndef _Sem_h_
#define _Sem_h_
#include <semaphore.h>
/********************************************************************************************
*
*File name: CCountingSem.h
*Author: Wu Yinghao
*Version: 0.0.1
*Date: 2013.5.20
*E-Mail: wyh817@gmail.com
*Description: this file is base class of CountingSem
*
********************************************************************************************/

class CCountingSem
{
	public:
		typedef enum {
       	 kTimeout,
        	 kForever
    		}Mode;                     


		CCountingSem();
		~CCountingSem();

		 //get the sem and sem -1
		 bool Get(Mode mode = kForever, unsigned long timeoutMS = 0);

		 //post the sem and sem +1
    	 bool Post(void);
    private:

		sem_t		sem;

};









#endif



