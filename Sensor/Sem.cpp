

#include "Sem.h"





CCountingSem::CCountingSem()
{
    sem_init(&sem, 0, 0);
}



CCountingSem::~CCountingSem()
{


}

bool CCountingSem::Get(Mode mode , unsigned long )
{
    
	 if(sem_wait(&sem)==0)
	 	return true;
	 else
	 	return false;


}


bool CCountingSem::Post()
{
	sem_post(&sem);
    	return true;

}

