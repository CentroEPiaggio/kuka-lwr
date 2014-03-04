//  ---------------------- Doxygen info ----------------------
//! \file GetUDPCommunicationData.cpp
//!
//! \brief
//! Implementation file for several methods of the class FastResearchInterface for
//! providing data about the UDP communication channel
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
#include <friComm.h>


// ****************************************************************
// GetFRICycleTime()
//
float FastResearchInterface::GetFRICycleTime(void)
{
	float		ReturnValue		=	0.0;

	pthread_mutex_lock(&(this->MutexForControlData));
	ReturnValue	=	this->ReadData.intf.desiredCmdSampleTime;
	pthread_mutex_unlock(&(this->MutexForControlData));

	return(ReturnValue);
}


// ****************************************************************
// GetCommunicationTimingQuality()
//
int FastResearchInterface::GetCommunicationTimingQuality(void)
{
	int		ReturnValue		=	0;

	pthread_mutex_lock(&(this->MutexForControlData));
	ReturnValue	=	this->ReadData.intf.quality;
	pthread_mutex_unlock(&(this->MutexForControlData));

	return(ReturnValue);
}


// ****************************************************************
// GetUDPAnswerRate()
//
float FastResearchInterface::GetUDPAnswerRate(void)
{
	float		ReturnValue		=	0.0;

	pthread_mutex_lock(&(this->MutexForControlData));
	ReturnValue	=	this->ReadData.intf.stat.answerRate;
	pthread_mutex_unlock(&(this->MutexForControlData));

	return(ReturnValue);
}


// ****************************************************************
// GetUDPLatencyInSeconds()
//
float FastResearchInterface::GetUDPLatencyInSeconds(void)
{
	float		ReturnValue		=	0.0;

	pthread_mutex_lock(&(this->MutexForControlData));
	ReturnValue	=	this->ReadData.intf.stat.latency;
	pthread_mutex_unlock(&(this->MutexForControlData));

	return(ReturnValue);
}


// ****************************************************************
// GetUDPJitterInSeconds()
//
float FastResearchInterface::GetUDPJitterInSeconds(void)
{
	float		ReturnValue		=	0.0;

	pthread_mutex_lock(&(this->MutexForControlData));
	ReturnValue	=	this->ReadData.intf.stat.jitter;
	pthread_mutex_unlock(&(this->MutexForControlData));

	return(ReturnValue);
}


// ****************************************************************
// GetUDPPackageLossRate()
//
float FastResearchInterface::GetUDPPackageLossRate(void)
{
	float		ReturnValue		=	0.0;

	pthread_mutex_lock(&(this->MutexForControlData));
	ReturnValue	=	this->ReadData.intf.stat.missRate;
	pthread_mutex_unlock(&(this->MutexForControlData));

	return(ReturnValue);
}


// ****************************************************************
// GetNumberOfMissedUDPPackages()
//
unsigned int FastResearchInterface::GetNumberOfMissedUDPPackages(void)
{
	unsigned int		ReturnValue		=	0;

	pthread_mutex_lock(&(this->MutexForControlData));
	ReturnValue	=	this->ReadData.intf.stat.missCounter;
	pthread_mutex_unlock(&(this->MutexForControlData));

	return(ReturnValue);
}


// ****************************************************************
// GetValueOfKRCSequenceCounter()
//
unsigned int FastResearchInterface::GetValueOfKRCSequenceCounter(void)
{
	unsigned int		ReturnValue		=	0;

	pthread_mutex_lock(&(this->MutexForControlData));
	ReturnValue	=	this->ReadData.head.sendSeqCount;
	pthread_mutex_unlock(&(this->MutexForControlData));

	return(ReturnValue);
}
