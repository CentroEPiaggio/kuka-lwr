//  ---------------------- Doxygen info ----------------------
//! \file Console.h
//!
//! \brief
//! Header file for the class Console
//!
//! \details
//! The class Console provides the possibility of doing screen
//! outputs by a low-priority thread. Basically, a real-time
//! capable \c printf() function is provided.
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


#ifndef __Console__
#define __Console__

#include <stdio.h>
#include <pthread.h>
#include <sched.h>


//  ---------------------- Doxygen info ----------------------
//! \def CONSOLE_NUMBER_OF_BUFFER_ENTRIES
//!
//! \brief
//! The number of output messages that can be stored in one half of the
//! double buffer Console::Buffer[2].
//  ----------------------------------------------------------
#define CONSOLE_NUMBER_OF_BUFFER_ENTRIES	128


//  ---------------------- Doxygen info ----------------------
//! \def CONSOLE_BUFFER_ENTRY_SIZE
//!
//! \brief
//! The size of one single output message in bytes.
//  ----------------------------------------------------------
#define CONSOLE_BUFFER_ENTRY_SIZE			4096




//  ---------------------- Doxygen info ----------------------
//! \class Console
//!
//! \brief
//! Screen or file output for real-time threads
//!
//! \details
//! This class is used for the entire output to the console. To do this without
//! interfering with the real-time requirements of other processes and/or threads,
//! a dedicated thread with a low priority is used for the actual output. This class
//! implements the singleton pattern, since only \em one instance is needed in any process.
//!
//! Basically, this class offers a variant of the \c printf() function that can be used
//! by real-time threads of the calling process.
//  ----------------------------------------------------------
class Console
{
public:

//  ---------------------- Doxygen info ----------------------
//! \fn Console(const unsigned int &Priority, const FILE *FileHandler = stdout)
//!
//! \brief
//! Constructor
//!
//! \details
//! The constructor creates a process running at a priority of \c Priority, and
//! it prepares a double buffer for data exchange between real-time threads that
//! may call the method Console::printf() and the low-priority thread
//! Console::ConsoleThreadMain() that is responsible for the actual output to
//! \c FileHandler.
//!
//! \param Priority
//! The priority of the console thread
//!
//! \param FileHandler
//! A file handler, to which the output is written. The default is \c stdout.
//!
//! \attention
//! The call of the constructor does \b not fulfill any real-time requirements.
//  ----------------------------------------------------------
	Console(		const unsigned int 	&Priority
	        	,	const FILE			*FileHandler = stdout);


//  ---------------------- Doxygen info ----------------------
//! \fn ~Console(void)
//!
//! \brief
//! Destructor
//!
//! \details
//! By using the condition variable Console::CondVar and setting the flag
//! Console::TermintateThread, the destructor sends a signal
//! to the output thread Console::ConsoleThreadMain(), which will terminate
//! it. Afterwards, the thread is joined.
//!
//! \attention
//! The call of the destructor does \b not fulfill any real-time requirements.
//  ----------------------------------------------------------
	~Console(void);


//  ---------------------- Doxygen info ----------------------
//! \fn int printf(const char* Format, ...)
//!
//! \brief
//! A real-time wrapper for printf
//!
//! \details
//! This is the main method of the class Console; it may be called by real-time
//! threads of the process to which this object belongs. It copies the formated
//! character string to one part of the double buffer and uses the condition
//! variable Console::CondVar to wake up the low-priority thread
//! Console::ConsoleThreadMain(), which can work on the actual output as soon as
//! it becomes scheduled.
//!
//! \param Format
//! Formats the string to be sent to Console::Handler.
//! For details please refer for example to this page:
//! <a href="http://www.qnx.com/developers/docs/6.3.2/neutrino/lib_ref/p/printf.html">
//! http://www.qnx.com/developers/docs/6.3.2/neutrino/lib_ref/p/printf.html</a>.
//!
//! \param ...
//! A variable argument list is used here. For details, please refer to
//! <tt>stdarg.h</tt> and
//! <a href="http://www.qnx.com/developers/docs/6.3.2/neutrino/lib_ref/p/printf.html">
//! http://www.qnx.com/developers/docs/6.3.2/neutrino/lib_ref/p/printf.html</a>.
//!
//! \return
//!  - The number characters sent to the file handler Console::Handler.
//!  - If the capacity of the double buffer is exceeded, \c ENOBUFS is returned, and
//!    a corresponding string is written to the output in order to acknowledge the user.
//  ----------------------------------------------------------
	int printf(const char* Format, ...);


//  ---------------------- Doxygen info ----------------------
//! \fn void flush(void) const
//!
//! \brief
//! Prints all pending output data
//!
//! \details
//! This method flushes all pending data of the file handler Console::Handler.
//  ----------------------------------------------------------
	void flush(void) const;


//  ---------------------- Doxygen info ----------------------
//! \fn FILE* GetFileHandler(void) const
//!
//! \brief
//! Returns the file handler Console::Handler
//!
//! \return
//! File handler Console::Handler
//  ----------------------------------------------------------
	FILE* GetFileHandler(void) const;


protected:


//  ---------------------- Doxygen info ----------------------
//! \var FILE *Handler
//!
//! \brief
//! The file handler Console::Handler
//  ----------------------------------------------------------
	FILE				*Handler;


//  ---------------------- Doxygen info ----------------------
//! \fn static void* ConsoleThreadMain(void* ObjectPointer)
//!
//! \brief
//! Thread that performs the actual output
//!
//! \details
//! This thread runs at priority of \c Priority (specified by the constructor
//! Console::Console() of the class Console. After successful creation, this
//! thread waits for the condition variable Console::CondVar <b>(1)</b> to be waked up,
//! which means that one of the (real-time) threads wrote output data into the double
//! buffer Console::Buffer[2], <b>(2)</b> to lock the mutex Console::Mutex, <b>(3)</b> to
//! switch the boolean buffer selection variable, <b>(4)</b> to unlock the mutex
//! Console::Mutex again, <b>(5)</b> to empty the used half of the double buffer by writing
//! its content to Console::Handler, and <b>(6)</b> wait for a message in the other half
//! of the double buffer (or empty it if data is already waiting).
//!
//! If the thread gets waked up by the destructor Console::~Console() via
//! \c pthread_cond_signal(), and if Console::TermintateThread is set, the thread terminates.
//!
//! \param ObjectPointer
//! A pointer the current Console object of the calling process.
//  ----------------------------------------------------------
	static void* ConsoleThreadMain(void* ObjectPointer);


//  ---------------------- Doxygen info ----------------------
//! \var unsigned int NumberOfMessages[2]
//!
//! \brief
//! An array of two integer values; each representing the current number of message
//! in the double buffer Console::Buffer[2]
//  ----------------------------------------------------------
	unsigned int		NumberOfMessages[2];


//  ---------------------- Doxygen info ----------------------
//! \var bool ThreadCreated
//!
//! \brief
//! This flag is set during the thread creation in order to acknowledge the
//! creating thread.
//  ----------------------------------------------------------
	bool				ThreadCreated;


//  ---------------------- Doxygen info ----------------------
//! \var bool ConsoleThreadReadyToRun;
//!
//! \brief
//! This flag is set by the main thread in order to acknowledge the console thread that
//! it can run its loop.
//  ----------------------------------------------------------
	bool				ConsoleThreadReadyToRun;


//  ---------------------- Doxygen info ----------------------
//! \var bool TermintateThread
//!
//! \brief
//! If this flag is set by the destructor, and if the thread Console::ConsoleThreadMain()
//! gets waked up by \c pthread_signal(), Console::ConsoleThreadMain() will terminate.
//  ----------------------------------------------------------
	bool				TermintateThread;


//  ---------------------- Doxygen info ----------------------
//! \var bool BufferNumber
//!
//! \brief
//! Buffer selection variable
//!
//! \details
//!  - \c false \f$ \longrightarrow \f$ first double buffer half
//!  - \c true \f$ \longrightarrow \f$ second double buffer half
//!
//! This variable is protected by the mutex Console::Mutex.
//  ----------------------------------------------------------
	bool				BufferNumber;


//  ---------------------- Doxygen info ----------------------
//! \var pthread_t ConsoleThread
//!
//! \brief
//! Thread ID of Console::ConsoleThreadMain()
//!
//! \details
//! A \c pthread_t object containing the thread ID of Console::ConsoleThreadMain().
//  ----------------------------------------------------------
	pthread_t			ConsoleThread;


//  ---------------------- Doxygen info ----------------------
//! \var pthread_mutex_t Mutex
//!
//! \brief
//! Mutex to protect the variables Console::BufferNumber and Console::Buffer
//!
//! \details
//! This \c pthread_mutex_t object protects
//!  - the variables Console::BufferNumber between the output data sending
//!    threads (running under real-time conditions) and the output data writing thread
//!    Console::ConsoleThreadMain(), and
//!  - one half of the double buffer Console::Buffer among the real-time threads.
//  ----------------------------------------------------------
	pthread_mutex_t		Mutex;


//  ---------------------- Doxygen info ----------------------
//! \var pthread_cond_t CondVar
//!
//! \brief
//! Condition variable to acknowledge the thread Console::ConsoleThreadMain() that
//! new data is waiting for output
//!
//! \details
//! This \c pthread_cond_t object to
//! - let data-sending (real-time) threads acknowledge the data output thread
//!   Console::ConsoleThreadMain() that new data was written to one half of
//!   of the double buffer Console::Buffer, and to
//! - let the destructor Console::~Console() wake up the output thread
//!   Console::ConsoleThreadMain() to let him terminate if
//!   Console::TermintateThread is set.
//!
//! This condition variable is always used in combination with the mutex \c
//! Console:Mutex.
//  ----------------------------------------------------------
	pthread_cond_t		CondVar;


//  ---------------------- Doxygen info ----------------------
//! \var char Buffer[2][CONSOLE_NUMBER_OF_BUFFER_ENTRIES][CONSOLE_BUFFER_ENTRY_SIZE]
//!
//! \brief
//! Double buffer for data exchange between the data-sending (real-time) threads and Console::ConsoleThreadMain()
//!
//! \details
//! This double buffer variable is protected by the mutex Console::Mutex, such that
//! only one real-time thread can write new data into one half of it.
//  ----------------------------------------------------------
	char				Buffer[2][CONSOLE_NUMBER_OF_BUFFER_ENTRIES][CONSOLE_BUFFER_ENTRY_SIZE];


};	// class Console

#endif
