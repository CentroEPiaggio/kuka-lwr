//  ---------------------- Doxygen info ----------------------
//! \file LoggingMethods.cpp
//!
//! \brief
//! Implementation file for the four logging methods of the class FastResearchInterface
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
#include <DataLogging.h>
#include <errno.h>
#include <OSAbstraction.h>


// ****************************************************************
// PrepareLogging()
//
int FastResearchInterface::PrepareLogging(const char *FileIdentifier)
{
	if (this->LoggingState != FastResearchInterface::WriteLoggingDataFileCalled)
	{
		return(EINVAL);
	}
	else
	{
		this->LoggingState = FastResearchInterface::PrepareLoggingCalled;
	}

	return(this->DataLogger->PrepareLogging(this->CurrentControlScheme, FileIdentifier));
}


// ****************************************************************
// StartLogging()
//
int FastResearchInterface::StartLogging(void)
{
	if (this->LoggingState != FastResearchInterface::PrepareLoggingCalled)
	{
		return(EINVAL);
	}
	else
	{
		this->LoggingState = FastResearchInterface::StartLoggingCalled;
	}

	pthread_mutex_lock(&(this->MutexForLogging));
	this->LoggingIsActive	=	true;
	pthread_mutex_unlock(&(this->MutexForLogging));

	return(EOK);
}


// ****************************************************************
// StopLogging()
//
int FastResearchInterface::StopLogging(void)
{
	if (this->LoggingState != FastResearchInterface::StartLoggingCalled)
	{
		return(EINVAL);
	}
	else
	{
		this->LoggingState = FastResearchInterface::StopLoggingCalled;
	}

	pthread_mutex_lock(&(this->MutexForLogging));
	this->LoggingIsActive	=	false;
	pthread_mutex_unlock(&(this->MutexForLogging));

	return(EOK);
}


// ****************************************************************
// WriteLoggingDataFile()
//
int FastResearchInterface::WriteLoggingDataFile(void)
{
	if (this->LoggingState == FastResearchInterface::WriteLoggingDataFileCalled)
	{
		return(EINVAL);
	}

	this->LoggingState = FastResearchInterface::WriteLoggingDataFileCalled;

	if (this->LoggingState == FastResearchInterface::StartLoggingCalled)
	{
		pthread_mutex_lock(&(this->MutexForLogging));
		this->LoggingIsActive	=	false;
		pthread_mutex_unlock(&(this->MutexForLogging));
	}

	return(this->DataLogger->WriteToFile());
}

