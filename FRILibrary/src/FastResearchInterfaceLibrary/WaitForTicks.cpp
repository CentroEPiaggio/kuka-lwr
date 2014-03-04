//  ---------------------- Doxygen info ----------------------
//! \file WaitForTicks.cpp
//!
//! \brief
//! Implementation file for the simple timer methods of the class FastResearchInterface
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
#include <errno.h>
#include <OSAbstraction.h>




// ****************************************************************
// WaitForKRCTick()
//
int FastResearchInterface::WaitForKRCTick(const unsigned int &TimeoutValueInMicroSeconds)
{
	int		ReturnValue		=	EOK;

#ifdef _NTO_

	struct timespec			TimeoutValue
						,	CurrentTime;

	if (TimeoutValueInMicroSeconds > 0)
	{
		clock_gettime(CLOCK_REALTIME, &CurrentTime);

		TimeoutValue.tv_nsec	=	(CurrentTime.tv_nsec + (TimeoutValueInMicroSeconds * 1000)) % 1000000000;
		TimeoutValue.tv_sec		=	CurrentTime.tv_sec + (TimeoutValueInMicroSeconds % 1000000)
										+ (CurrentTime.tv_nsec + (TimeoutValueInMicroSeconds * 1000)) / 1000000000;

		pthread_mutex_lock(&(this->MutexForControlData));
		while (!this->NewDataFromKRCReceived)
		{

			ReturnValue	=	 pthread_cond_timedwait(	&(this->CondVarForDataReceptionFromKRC)
			           	 	                        ,	&(this->MutexForControlData)
			           	 	                        ,	&TimeoutValue							);
		}
		this->NewDataFromKRCReceived	=	false;
		pthread_mutex_unlock(&(this->MutexForControlData));

		return(ReturnValue);
	}

#endif

	pthread_mutex_lock(&(this->MutexForControlData));
	while (!this->NewDataFromKRCReceived)
	{

		ReturnValue	=	 pthread_cond_wait(		&(this->CondVarForDataReceptionFromKRC)
		           	 	                   	,	&(this->MutexForControlData)			);
	}
	this->NewDataFromKRCReceived	=	false;
	pthread_mutex_unlock(&(this->MutexForControlData));


	return(ReturnValue);
}



// ****************************************************************
// WaitForTimerTick()
//
int FastResearchInterface::WaitForTimerTick(const unsigned int &TimeoutValueInMicroSeconds)
{
	int		ReturnValue		=	0;

#ifdef _NTO_

	struct timespec			TimeoutValue
						,	CurrentTime;

	if (TimeoutValueInMicroSeconds > 0)
	{
		clock_gettime(CLOCK_REALTIME, &CurrentTime);

		TimeoutValue.tv_nsec	=	(CurrentTime.tv_nsec + (TimeoutValueInMicroSeconds * 1000)) % 1000000000;
		TimeoutValue.tv_sec		=	CurrentTime.tv_sec + (TimeoutValueInMicroSeconds % 1000000)
										+ (CurrentTime.tv_nsec + (TimeoutValueInMicroSeconds * 1000)) / 1000000000;

		pthread_mutex_lock(&(this->MutexForCondVarForTimer));
		while (!this->TimerFlag)
		{

			ReturnValue	=	 pthread_cond_timedwait(	&(this->CondVarForTimer)
			           	 	                        ,	&(this->MutexForCondVarForTimer)
			           	 	                        ,	&TimeoutValue							);
		}
		this->TimerFlag	=	false;
		pthread_mutex_unlock(&(this->MutexForCondVarForTimer));

		return(ReturnValue);
	}

#endif

	pthread_mutex_lock(&(this->MutexForCondVarForTimer));
	while (!this->TimerFlag)
	{

		ReturnValue	=	 pthread_cond_wait(		&(this->CondVarForTimer)
		           	 	                   	,	&(this->MutexForCondVarForTimer)			);
	}

	this->TimerFlag	=	false;
	pthread_mutex_unlock(&(this->MutexForCondVarForTimer));

	return(ReturnValue);
}
