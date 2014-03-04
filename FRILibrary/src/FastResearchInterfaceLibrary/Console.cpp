//  ---------------------- Doxygen info ----------------------
//! \file Console.cpp
//!
//! \brief
//! Implementation file for the class Console
//!
//! \details
//! The class Console provides the possibility of doing screen
//! outputs by a low-priority thread. Basically, the \c printf()
//! function is provided. For further details, please refer to
//! the file Console.h
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


#include <Console.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <pthread.h>
#include <OSAbstraction.h>


#ifdef WIN32// \ToDo Make this clean through the OSAbstraction
#include <Windows.h>	
#endif



// ****************************************************************
// Constructor 
//
Console::Console(		const unsigned int 	&Priority
                 	,	const FILE			*FileHandler)
{
	struct sched_param		ThreadSchedulingParams;

	pthread_attr_t			ThreadAttributes;

	TermintateThread		=	false;
	ThreadCreated			=	false;
	ConsoleThreadReadyToRun	=	false;

	pthread_mutex_init		(&(this->Mutex)		, NULL);
	pthread_cond_init		(&(this->CondVar)	, NULL);

	ThreadSchedulingParams.sched_priority	=	Priority;

	pthread_attr_init			(&ThreadAttributes								)	;
	pthread_attr_setschedpolicy	(&ThreadAttributes	,	SCHED_FIFO				)	;
	pthread_attr_setinheritsched(&ThreadAttributes	,	PTHREAD_EXPLICIT_SCHED	)	;
	pthread_attr_setschedparam	(&ThreadAttributes	,	&ThreadSchedulingParams	)	;

	pthread_create(		&ConsoleThread
	               	,	&ThreadAttributes
	               	,	&ConsoleThreadMain
	               	,	this);

	pthread_mutex_lock(&(this->Mutex));

	while (!ThreadCreated)
	{
		pthread_cond_wait (&(this->CondVar), &(this->Mutex));
	}

	pthread_mutex_unlock(&(this->Mutex));

	memset(this->Buffer, 0x0, 		2
	       	   	   	   	   	   *	CONSOLE_NUMBER_OF_BUFFER_ENTRIES
	       	   	   	   	   	   *	CONSOLE_BUFFER_ENTRY_SIZE
	       	   	   	   	   	   *	sizeof(char));



	this->BufferNumber		=	false;

	this->NumberOfMessages[0]	=	0;
	this->NumberOfMessages[1]	=	0;


	this->Handler				=	(FILE*)FileHandler;

	pthread_mutex_lock(&(this->Mutex));
	this->ConsoleThreadReadyToRun	=	true;
	pthread_mutex_unlock(&(this->Mutex));

	pthread_cond_signal(&(this->CondVar));
}


// ****************************************************************
// Destructor
//
Console::~Console(void)
{
	pthread_mutex_lock(&(this->Mutex));
	this->TermintateThread	=	true;
	pthread_mutex_unlock(&(this->Mutex));
	pthread_cond_signal(&(this->CondVar));
	pthread_join(this->ConsoleThread, NULL);
}


// ****************************************************************
// printf()
//
int Console::printf(const char* Format,...)
{
	int			Result;

    va_list		ListOfArguments;

	va_start(ListOfArguments, Format);


	pthread_mutex_lock(&(this->Mutex));
	if (this->NumberOfMessages[this->BufferNumber] >= CONSOLE_NUMBER_OF_BUFFER_ENTRIES)
	{
		sprintf(this->Buffer[this->BufferNumber][this->NumberOfMessages[this->BufferNumber] - 1], "Console::printf(): Buffer is full, skipping output!\n");
		Result = strlen(this->Buffer[this->BufferNumber][this->NumberOfMessages[this->BufferNumber] - 1]);
		pthread_mutex_unlock(&(this->Mutex));
		return(Result);
	}

	(this->NumberOfMessages[this->BufferNumber])++;
	Result = vsnprintf(this->Buffer[this->BufferNumber][NumberOfMessages[this->BufferNumber] - 1], CONSOLE_BUFFER_ENTRY_SIZE - 1, Format, ListOfArguments);
	pthread_mutex_unlock(&(this->Mutex));
	pthread_cond_signal(&(this->CondVar));

	va_end(ListOfArguments);
	return(Result);
}



	
	
// ****************************************************************
// ConsoleThreadMain()
//

void* Console::ConsoleThreadMain(void* ObjectPointer)
{
	unsigned int		i		=	0;

	Console* ThisObjectPtr = (Console*)ObjectPointer;
	
#ifdef WIN32
	// \ToDo Make this clean through the OSAbstraction
	SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_LOWEST);
#endif
	
	pthread_mutex_lock(&(ThisObjectPtr->Mutex));
	ThisObjectPtr->ThreadCreated	=	true;
	pthread_mutex_unlock(&(ThisObjectPtr->Mutex));

	pthread_cond_signal(&(ThisObjectPtr->CondVar));

	pthread_mutex_lock(&(ThisObjectPtr->Mutex));

	while (!ThisObjectPtr->ConsoleThreadReadyToRun)
	{
		pthread_cond_wait (&(ThisObjectPtr->CondVar), &(ThisObjectPtr->Mutex));
	}

	pthread_mutex_unlock(&(ThisObjectPtr->Mutex));

	for(;;)
	{
		pthread_mutex_lock(&(ThisObjectPtr->Mutex));
		if (ThisObjectPtr->NumberOfMessages[!(ThisObjectPtr->BufferNumber)] == 0)
		{
			if (ThisObjectPtr->TermintateThread)
			{
				pthread_mutex_unlock(&(ThisObjectPtr->Mutex));
				break;
			}
			pthread_cond_wait (&(ThisObjectPtr->CondVar), &(ThisObjectPtr->Mutex));

			// keep the maximum amount of CPU time for this thread at a minimum while the mutex is locked
			ThisObjectPtr->BufferNumber = !ThisObjectPtr->BufferNumber;

			pthread_mutex_unlock(&(ThisObjectPtr->Mutex));
		}
		else
		{
			pthread_mutex_unlock(&(ThisObjectPtr->Mutex));
		}

		for(i = 0; i < ThisObjectPtr->NumberOfMessages[!(ThisObjectPtr->BufferNumber)]; i++)
		{
			fprintf(ThisObjectPtr->Handler, "%s", ThisObjectPtr->Buffer[!(ThisObjectPtr->BufferNumber)][i]);
		}

		ThisObjectPtr->flush();
		ThisObjectPtr->NumberOfMessages[!(ThisObjectPtr->BufferNumber)] = 0;
	}

	for(i = 0; i < ThisObjectPtr->NumberOfMessages[ThisObjectPtr->BufferNumber]; i++)
	{
		fprintf(ThisObjectPtr->Handler, "%s", ThisObjectPtr->Buffer[ThisObjectPtr->BufferNumber][i]);
	}

	ThisObjectPtr->flush();

	pthread_exit(NULL);
	
	return (NULL);
}



// ****************************************************************
// flush()
//
void Console::flush(void) const
{
	fflush(this->Handler);
	return;
}


// ****************************************************************
// GetFileHandler()
//
FILE* Console::GetFileHandler(void) const
{
	return(this->Handler);
}
