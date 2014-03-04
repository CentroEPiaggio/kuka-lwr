//  ---------------------- Doxygen info ----------------------
//! \file TimerThreadMain.cpp
//!
//! \brief
//! Implementation file for the timer thread of the class FastResearchInterface
//!
//! \details
//! The class FastResearchInterface provides a basic low-level interface
//! to the KUKA Light-Weight Robot IV For details, please refer to the file
//! FastResearchInterface.h.
//! \n
//! \n
//! <b>GNU Lesser Public License</b>
//! \n
//! This file is part of the Fast Research Interface Library.
//! \n\n
//! The Fast Research Interface Library is free software: you can redistribute
//! it and/or modify it under the terms of the GNU General Public License
//! as published by the Free Software Foundation, either version 3 of the
//! License, or (at your option) any later version.
//! \n\n
//! The Fast Research Interface Library is distributed in the hope that it
//! will be useful, but WITHOUT ANY WARRANTY; without even the implied 
//! warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See
//! the GNU General Public License for more details.
//! \n\n
//! You should have received a copy of the GNU General Public License
//! along with the Fast Research Interface Library. If not, see 
//! http://www.gnu.org/licenses.
//! \n
//! \n
//! Stanford University\n
//! Department of Computer Science\n
//! Artificial Intelligence Laboratory\n
//! Gates Computer Science Building 1A\n
//! 353 Serra Mall\n
//! Stanford, CA 94305-9010\n
//! USA\n
//! \n
//! http://cs.stanford.edu/groups/manips\n
//!
//! \date November 2011
//!
//! \version 1.0
//!
//!	\author Torsten Kroeger, tkr@stanford.edu
//!
//!
//!
//  ----------------------------------------------------------
//   For a convenient reading of this file's source code,
//   please use a tab width of four characters.
//  ----------------------------------------------------------


#include <FastResearchInterface.h>
#include <pthread.h>
#include <stdio.h>
#include <OSAbstraction.h>


#ifdef _NTO_



#include <sys/neutrino.h>
#include <sched.h>


#define	TIMER_PULSE					(_PULSE_CODE_MINAVAIL)



// ****************************************************************
// TimerThreadMain()
//
void* FastResearchInterface::TimerThreadMain(void *ObjectPointer)
{
	int								OurChannelID	=	0
								,	ReceptionID		=	0;

	timer_t 						TimerID;

	struct sigevent 				Event;

	struct itimerspec 				Timer;

	struct _pulse  					PulseMsg;

	FastResearchInterface			*ThisObjectPtr		=	(FastResearchInterface*)ObjectPointer;

	OurChannelID				=	ChannelCreate(0); //create communication channel

	// Initialize event data structure
	// attach the timer to the channel OurChannelID
	Event.sigev_notify			=	SIGEV_PULSE;
	Event.sigev_coid			=	ConnectAttach(0, 0, OurChannelID, _NTO_SIDE_CHANNEL, 0);
	Event.sigev_priority		=	getprio(0);
	Event.sigev_code			=	TIMER_PULSE;

	timer_create(CLOCK_REALTIME, &Event, &TimerID);

	// Configure the timer
	Timer.it_value.tv_sec		=	0L;
	Timer.it_value.tv_nsec		=	(long int)(1000000000.0 * ThisObjectPtr->CycleTime);	// wait one cycle time interval before start
	Timer.it_interval.tv_sec	=	0L;
	Timer.it_interval.tv_nsec	=	(long int)(1000000000.0 * ThisObjectPtr->CycleTime);

	pthread_mutex_lock(&(ThisObjectPtr->MutexForThreadCreation));
	ThisObjectPtr->ThreadCreated	=	true;
	pthread_mutex_unlock(&(ThisObjectPtr->MutexForThreadCreation));

	pthread_cond_signal(&(ThisObjectPtr->CondVarForThreadCreation));

	// Start the timer
	timer_settime(TimerID, 0, &Timer, NULL);

	pthread_mutex_lock(&(ThisObjectPtr->MutexForCondVarForTimer));

	while(!(ThisObjectPtr->TerminateTimerThread))
	{
		pthread_mutex_unlock(&(ThisObjectPtr->MutexForCondVarForTimer));

		ReceptionID	= MsgReceive(OurChannelID, &PulseMsg, sizeof(PulseMsg), NULL);

		pthread_mutex_lock(&(ThisObjectPtr->MutexForCondVarForTimer));
		if (ReceptionID == 0)
		{
			ThisObjectPtr->TimerFlag = true;
			pthread_cond_signal(&(ThisObjectPtr->CondVarForTimer));
		}
	}
	pthread_mutex_unlock(&(ThisObjectPtr->MutexForCondVarForTimer));

	if (timer_delete(TimerID) != 0)
    {
		ThisObjectPtr->OutputConsole->printf("FastResearchInterface::TimerThreadMain(): ERROR, cannot delete timer...\n");
    }

	pthread_exit(NULL);
}


#endif // _NTO_


#ifndef _NTO_



// ****************************************************************
// TimerThreadMain()
//
void* FastResearchInterface::TimerThreadMain(void *ObjectPointer)
{
	int								OurChannelID	=	0
								,	ReceptionID		=	0;

	FastResearchInterface			*ThisObjectPtr	=	(FastResearchInterface*)ObjectPointer;

	pthread_mutex_lock(&(ThisObjectPtr->MutexForThreadCreation));
	ThisObjectPtr->ThreadCreated	=	true;
	pthread_mutex_unlock(&(ThisObjectPtr->MutexForThreadCreation));

	pthread_cond_signal(&(ThisObjectPtr->CondVarForThreadCreation));

	pthread_mutex_lock(&(ThisObjectPtr->MutexForCondVarForTimer));

	while(!(ThisObjectPtr->TerminateTimerThread))
	{
		pthread_mutex_unlock(&(ThisObjectPtr->MutexForCondVarForTimer));

		delay(1);

		pthread_mutex_lock(&(ThisObjectPtr->MutexForCondVarForTimer));
		if (ReceptionID == 0)
		{
			ThisObjectPtr->TimerFlag = true;
			pthread_cond_signal(&(ThisObjectPtr->CondVarForTimer));
		}
	}
	pthread_mutex_unlock(&(ThisObjectPtr->MutexForCondVarForTimer));

	pthread_exit(NULL);
	
	return (NULL);
}


#endif // WIN32