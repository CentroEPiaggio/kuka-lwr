//  ---------------------- Doxygen info ----------------------
//! \file KRCCommunicationThreadMain.cpp
//!
//! \brief
//! Implementation file for the thread of the class FastResearchInterface that
//! communicates with the KRC unit
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
#include <sched.h>
#include <string.h>
#include <stdio.h>
#include <friudp.h>
#include <friComm.h>
#include <OSAbstraction.h>
#include <iostream>


#ifdef WIN32// \ToDo Make this clean through the OSAbstraction
#include <Windows.h>	
#endif



// ****************************************************************
// KRCCommunicationThreadMain()
//
void* FastResearchInterface::KRCCommunicationThreadMain(void *ObjectPointer)
{
	int								SequenceCounter					=	0
								,	ResultValue						=	0;

	float							ZeroVector[LBR_MNJ];

	//friUdp 							KRC;

	tFriMsrData 					LocalReadData;

	tFriCmdData						LocalCommandData;



	FastResearchInterface			*ThisObjectPtr					=	(FastResearchInterface*)ObjectPointer;

	friUdp 							KRC(ThisObjectPtr->PORT, ThisObjectPtr->IP);
	

	memset(ZeroVector, 0x0, LBR_MNJ * sizeof(float));
	
#ifdef WIN32
	// \ToDo Make this clean through the OSAbstraction
	SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_HIGHEST);
#endif	

	pthread_mutex_lock(&(ThisObjectPtr->MutexForThreadCreation));
	ThisObjectPtr->ThreadCreated	=	true;
	pthread_mutex_unlock(&(ThisObjectPtr->MutexForThreadCreation));

	pthread_cond_signal(&(ThisObjectPtr->CondVarForThreadCreation));

	for(;;)
	{
		// receive data from the KRC unit
		ResultValue	=	KRC.Recv(&LocalReadData);

		if (ResultValue != 0)
		{
			ThisObjectPtr->OutputConsole->printf("FastResearchInterface::KRCCommunicationThreadMain(): ERROR during the reception of a UDP data package.\n");
		}

		pthread_mutex_lock(&(ThisObjectPtr->MutexForControlData));

		ThisObjectPtr->NewDataFromKRCReceived	=	true;
		ThisObjectPtr->ReadData				=	LocalReadData;

		if (ThisObjectPtr->TerminateKRCCommunicationThread)
		{
			pthread_mutex_unlock(&(ThisObjectPtr->MutexForControlData));
			break;
		}

		LocalCommandData	=	ThisObjectPtr->CommandData;

		SequenceCounter++;
		LocalCommandData.head.sendSeqCount	=	SequenceCounter;
		LocalCommandData.head.reflSeqCount	=	LocalReadData.head.sendSeqCount;
		LocalCommandData.head.datagramId	=	FRI_DATAGRAM_ID_CMD;
		LocalCommandData.head.packetSize	=	sizeof(tFriCmdData);

		pthread_mutex_unlock(&(ThisObjectPtr->MutexForControlData));

		pthread_cond_broadcast(&(ThisObjectPtr->CondVarForDataReceptionFromKRC));

		// send data to KRC unit
		ResultValue						=	KRC.Send(&LocalCommandData);

		if (ResultValue != 0)
		{
			ThisObjectPtr->OutputConsole->printf("FastResearchInterface::KRCCommunicationThreadMain(): ERROR during the sending of a UDP data package.\n");
		}

		pthread_mutex_lock(&(ThisObjectPtr->MutexForLogging));

		if (ThisObjectPtr->LoggingIsActive)
		{
			pthread_mutex_unlock(&(ThisObjectPtr->MutexForLogging));
			ThisObjectPtr->DataLogger->AddEntry(		LocalReadData
			                                 	,	LocalCommandData);
		}
		else
		{
			pthread_mutex_unlock(&(ThisObjectPtr->MutexForLogging));
		}
	}

	pthread_exit(NULL);
	
	return (NULL);
}


