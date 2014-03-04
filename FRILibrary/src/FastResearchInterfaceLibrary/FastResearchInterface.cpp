//  ---------------------- Doxygen info ----------------------
//! \file FastResearchInterface.cpp
//!
//! \brief
//! Implementation file for the class FastResearchInterface
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
#include <Console.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <errno.h>
#include <stdarg.h>
#include <OSAbstraction.h>
#include <iostream>



#define MAX_ROBOT_NAME_LENGTH				256
#define MAX_OUTPUT_PATH_LENGTH				256
#define MAX_FILE_NAME_LENGTH				256
#define SIZE_OF_ROBOT_STATE_STRING			4096
#define MAX_IP_NUMBER_LENGHT				16


// ****************************************************************
// Constructor
//
FastResearchInterface::FastResearchInterface(const char *InitFileName)
{
	int					ParameterCount						=	0
					,	FuntionResult						=	0;

	struct sched_param		SchedulingParamsKRCCommunicationThread
						,	SchedulingParamsTimerThread
						,	SchedulingParamsMainThread;

	pthread_attr_t			AttributesKRCCommunicationThread
						,	AttributesTimerThread;

	this->RobotName				=	new char[MAX_ROBOT_NAME_LENGTH];
	this->LoggingPath			=	new char[MAX_OUTPUT_PATH_LENGTH];
	this->LoggingFileName		=	new char[MAX_FILE_NAME_LENGTH];
	this->RobotStateString		=	new char[SIZE_OF_ROBOT_STATE_STRING];
	this->IP 					= 	new char[MAX_IP_NUMBER_LENGHT];
	this->PORT 					=	49938; //default
	char IP_DEFAULT[12] = "192.168.0.10";

	memset((void*)(this->RobotName)				, 0x0	, MAX_ROBOT_NAME_LENGTH			* sizeof(char));
	memset((void*)(this->LoggingPath)			, 0x0	, MAX_OUTPUT_PATH_LENGTH		* sizeof(char));
	memset((void*)(this->LoggingFileName)		, 0x0	, MAX_FILE_NAME_LENGTH			* sizeof(char));
	memset((void*)(this->RobotStateString)		, 0x0	, SIZE_OF_ROBOT_STATE_STRING	* sizeof(char));
	memset((void*)(this->IP)					, 0x0 	, MAX_IP_NUMBER_LENGHT			* sizeof(char));
	memcpy(this->IP, IP_DEFAULT, 12*sizeof(char));
	

	ParameterCount	=	this->ReadInitFile(InitFileName);


	if (ParameterCount == -1)
	{
		fprintf(stderr, "FastResearchInterface::FastResearchInterface(): ERROR, an initialization file name is required! QUITTING.\n");
		getchar();
		exit(EXIT_FAILURE); // terminates the process
	}

	if (ParameterCount < 9)
	{
		fprintf(stderr, "FastResearchInterface::FastResearchInterface(): ERROR, initialization file \'%s\' is incomplete. Only %d parameters are given. QUITTING.\n", InitFileName, ParameterCount);
		getchar();
		exit(EXIT_FAILURE); // terminates the process
	}

	this->OutputConsole						=	new Console(PriorityOutputConsoleThread);

	this->TerminateTimerThread				=	false;
	this->TerminateKRCCommunicationThread	=	false;
	this->TimerFlag							=	false;
	this->NewDataFromKRCReceived			=	false;
	this->LoggingIsActive					=	false;
	this->ThreadCreated						=	false;	

	this->LoggingState						=	FastResearchInterface::WriteLoggingDataFileCalled;

	this->CurrentControlScheme				=	FastResearchInterface::JOINT_POSITION_CONTROL;

	memset((void*)(&(this->CommandData))	, 0x0	,										sizeof(tFriCmdData)	);
	memset((void*)(&(this->ReadData))		, 0x0	,										sizeof(tFriMsrData)	);

	pthread_mutex_init(&(this->MutexForControlData				), NULL);
	pthread_mutex_init(&(this->MutexForCondVarForTimer			), NULL);
	pthread_mutex_init(&(this->MutexForLogging					), NULL);
	pthread_mutex_init(&(this->MutexForThreadCreation			), NULL);

	pthread_cond_init(&(this->CondVarForTimer					), NULL);
	pthread_cond_init(&(this->CondVarForDataReceptionFromKRC	), NULL);
	pthread_cond_init(&(this->CondVarForThreadCreation			), NULL);

	this->OutputConsole->printf("Fast Research Interface: Using initialization file \"%s\".\n", InitFileName);

	// Thread configuration

	// get default information

	// set thread attributes to default values:
	//		detachstate			PTHREAD_CREATE_JOINABLE
	//		schedpolicy			PTHREAD_INHERIT_SCHED
	//		schedparam			Inherited from parent thread
	//		contentionscope		PTHREAD_SCOPE_SYSTEM
	//		stacksize			4K bytes
	//		stackaddr			NULL

	// Set the priorities from the initialization file.
	SchedulingParamsKRCCommunicationThread.sched_priority	=	PriorityKRCCommunicationThread						;
	SchedulingParamsTimerThread.sched_priority				=	PriorityTimerThread									;
	SchedulingParamsMainThread.sched_priority				=	PriorityMainThread									;

	pthread_attr_init(&AttributesKRCCommunicationThread															)	;
	pthread_attr_init(&AttributesTimerThread																	)	;

	// Set the thread scheduling policy attribute to round robin
	// default is OTHER which equals RR in QNX 6.5.0.
	pthread_attr_setschedpolicy(&AttributesKRCCommunicationThread	,	SCHED_FIFO								)	;
	pthread_attr_setschedpolicy(&AttributesTimerThread				,	SCHED_FIFO								)	;

	// Set the thread's inherit-scheduling attribute to explicit
	// otherwise, the scheduling parameters Cannot be changed (e.g., priority)
	pthread_attr_setinheritsched(&AttributesKRCCommunicationThread	,	PTHREAD_EXPLICIT_SCHED					)	;
	pthread_attr_setinheritsched(&AttributesTimerThread				,	PTHREAD_EXPLICIT_SCHED					)	;

	pthread_attr_setschedparam(&AttributesKRCCommunicationThread	,	&SchedulingParamsKRCCommunicationThread	)	;
	pthread_attr_setschedparam(&AttributesTimerThread				,	&SchedulingParamsTimerThread			)	;

	// this is supposed to become the message thread
	// i.e. we have to set our own scheduling parameters
	this->MainThread = pthread_self();
	pthread_setschedparam(this->MainThread, SCHED_FIFO, &SchedulingParamsMainThread);
	
#ifdef WIN32

	if(!SetPriorityClass(GetCurrentProcess(), HIGH_PRIORITY_CLASS))
	{
		this->OutputConsole->printf("FastResearchInterface::FastResearchInterface(): ERROR, could not set process priority.\n");
	} 

#endif


	FuntionResult	=	pthread_create(		&TimerThread
										,	&AttributesTimerThread
										,	&TimerThreadMain
										,	this);

	if (FuntionResult != EOK)
	{
		this->OutputConsole->printf("FastResearchInterface::FastResearchInterface(): ERROR, could not start the timer thread (Result: %d).\n", FuntionResult);
		getchar();
		exit(EXIT_FAILURE); // terminates the process
	}
	
	while (!ThreadCreated)
	{
		pthread_cond_wait (&(this->CondVarForThreadCreation), &(this->MutexForThreadCreation));
	}

	ThreadCreated	=	false;
	pthread_mutex_unlock(&(this->MutexForThreadCreation));	

	FuntionResult	=	pthread_create(		&KRCCommunicationThread
										,	&AttributesKRCCommunicationThread
										,	&KRCCommunicationThreadMain
										,	this);
										
	if (FuntionResult != EOK)
	{
		this->OutputConsole->printf("FastResearchInterface::FastResearchInterface(): ERROR, could not start the KRC communication thread (Result: %d).\n", FuntionResult);
		getchar();
		exit(EXIT_FAILURE); // terminates the process
	}
	
	pthread_mutex_lock(&(this->MutexForThreadCreation));

	while (!ThreadCreated)
	{
		pthread_cond_wait (&(this->CondVarForThreadCreation), &(this->MutexForThreadCreation));
	}

	ThreadCreated	=	false;
	pthread_mutex_unlock(&(this->MutexForThreadCreation));

	if (strlen(this->LoggingPath) > 0)
	{
		if (strcmp(&(this->LoggingPath[strlen(this->LoggingPath) - 1]), OS_FOLDER_SEPARATOR) != 0)
		{
			strcat(this->LoggingPath, OS_FOLDER_SEPARATOR);
		}
	}
	else
	{
		sprintf(this->LoggingPath, ".%s", OS_FOLDER_SEPARATOR);
	}

	if (strlen(this->LoggingFileName) == 0)
	{
		sprintf(this->LoggingFileName, "FRI.dat");
	}

	this->DataLogger	=	new DataLogging(	this->RobotName
											,	this->LoggingPath
											,	this->LoggingFileName
											,	this->NumberOfLoggingFileEntries);
}


// ****************************************************************
// Destructor
//
FastResearchInterface::~FastResearchInterface(void)
{
	int		ResultValue		=	0;

	// End the timer thread
	pthread_mutex_lock(&(this->MutexForCondVarForTimer));
	this->TerminateTimerThread = true;
	pthread_mutex_unlock(&(this->MutexForCondVarForTimer));
	pthread_join(this->TimerThread, NULL);

	// Bad workaround for the KUKA friUDP class, which unfortunately
	// does not use select() for sockets in order to enable timeouts.
	//
	// For the case, no UDP connection to the KRC could be
	// established, we have to wake up the communication thread
	// by sending a signal as it is still waiting for a message
	// from the KRC.
	
	pthread_mutex_lock(&(this->MutexForControlData));
	this->NewDataFromKRCReceived = false;
	pthread_mutex_unlock(&(this->MutexForControlData));

	delay(1 + 2 * (unsigned int)(this->CycleTime));

	pthread_mutex_lock(&(this->MutexForControlData));
	if (this->NewDataFromKRCReceived)
	{
		pthread_mutex_unlock(&(this->MutexForControlData));

		if (this->IsMachineOK())
		{
			this->StopRobot();
		}

		// The KRL program running on the robot controller waits for a
		// integer value change at position 15 (i.e., 16 in KRL).
		// A value of 30 or 40 (depending on the value of
		// this->GetKRLIntValue(14), that is, $FRI_TO_INT[15]
		// lets the KRL program call friClose() and
		// terminate itself.

		this->SetKRLIntValue(15, this->GetKRLIntValue(14));

		// wait for the next data telegram of the KRC unit
		pthread_mutex_lock(&(this->MutexForControlData));
		this->NewDataFromKRCReceived	=	false;
		pthread_mutex_unlock(&(this->MutexForControlData));
		ResultValue	=	this->WaitForKRCTick(((unsigned int)(this->CycleTime * 3000000.0)));

		if (ResultValue != EOK)
		{
			this->OutputConsole->printf("FastResearchInterface::~FastResearchInterface(): ERROR, the KRC unit does not respond anymore. Probably the FRI was closed or there is no UDP connection anymore.");
			getchar();
			pthread_kill(this->KRCCommunicationThread, SIGTERM);
		}

		// wait until the KRL program ended
		// (no UDP communication possible anymore)
		delay(800);

		// End the KRC communication thread
		pthread_mutex_unlock(&(this->MutexForControlData));
		// The UDP communication is working --> normal shutdown
		this->TerminateKRCCommunicationThread = true;
		pthread_mutex_unlock(&(this->MutexForControlData));
	}
	else
	{
		// No UDP communication to KRC and the communication
		// thread is blocked --> we wake him up manually
		this->TerminateKRCCommunicationThread = true;
		pthread_mutex_unlock(&(this->MutexForControlData));
		pthread_kill(this->KRCCommunicationThread, SIGTERM);
	}
	
#ifndef WIN32
	pthread_join(this->KRCCommunicationThread, NULL);
#endif

	if (this->LoggingState != FastResearchInterface::WriteLoggingDataFileCalled)
	{
		this->WriteLoggingDataFile();
	}

	delete[]	this->RobotName			;
	delete[]	this->LoggingPath		;
	delete[]	this->LoggingFileName	;
	delete[]	this->RobotStateString	;
	delete		this->DataLogger		;
	delete		this->OutputConsole		;
}


// ****************************************************************
// printf()
//
int FastResearchInterface::printf(const char* Format,...)
{
	int			Result		=	0;

    va_list		ListOfArguments;

	va_start(ListOfArguments, Format);

	Result = this->OutputConsole->printf(Format, ListOfArguments);

	va_end(ListOfArguments);

	return(Result);
}

