//  ---------------------- Doxygen info ----------------------
//! \file ReadInitFile.cpp
//!
//! \brief
//! Implementation file for the method ReadInitFile() of the class FastResearchInterface
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
#include <errno.h>
#include <string.h>
#include <InitializationFileEntry.h>
#include <OSAbstraction.h>


// ****************************************************************
// ReadInitFile()
//
int FastResearchInterface::ReadInitFile(const char *InitFileName)
{
	unsigned int				ParameterCount				=	0;

	InitializationFileEntry		InitFileParser(InitFileName);

	if (InitFileName != NULL)
	{
		while ( InitFileParser.NextEntry() )
		{
 			if ( !stricmp (InitFileParser.GetSection(), "Priorities") )
			{
				if ( !stricmp (InitFileParser.GetName(), "KRCCommunicationThread") )
				{
					this->PriorityKRCCommunicationThread = atoi( InitFileParser.GetValue() );
					ParameterCount++;
				}
				if ( !stricmp (InitFileParser.GetName(), "TimerThread") )
				{
					this->PriorityTimerThread = atoi( InitFileParser.GetValue() );
					ParameterCount++;
				}
				if ( !stricmp (InitFileParser.GetName(), "MainThread") )
				{
					this->PriorityMainThread = atoi( InitFileParser.GetValue() );
					ParameterCount++;
				}
				if ( !stricmp (InitFileParser.GetName(), "OutputConsoleThread") )
				{
					this->PriorityOutputConsoleThread = atoi( InitFileParser.GetValue() );
					ParameterCount++;
				}
			}
 			if ( !stricmp (InitFileParser.GetSection(), "RobotName") )
			{
				if ( !stricmp (InitFileParser.GetName(), "Name") )
				{
               		strcpy(this->RobotName, InitFileParser.GetValue() );
					ParameterCount++;
				}
				if ( !stricmp (InitFileParser.GetName(), "IP") )
				{
               		strcpy(this->IP, InitFileParser.GetValue() );
					ParameterCount++;
				}
				if ( !stricmp (InitFileParser.GetName(), "PORT") )
				{
               		this->PORT = atoi( InitFileParser.GetValue() );
					ParameterCount++;
				}
			}

 			if ( !stricmp (InitFileParser.GetSection(), "ControlValues") )
			{
				if ( !stricmp (InitFileParser.GetName(), "CycleTime") )
				{
					this->CycleTime = atof( InitFileParser.GetValue() );
					ParameterCount++;
				}
			}

 			if ( !stricmp (InitFileParser.GetSection(), "Logging") )
			{
				if ( !stricmp (InitFileParser.GetName(), "NumberOfLoggingFileEntries") )
				{
					NumberOfLoggingFileEntries = atoi( InitFileParser.GetValue() );
					ParameterCount++;
				}
				if ( !stricmp (InitFileParser.GetName(), "LoggingPath") )
				{
               		strcpy(this->LoggingPath, InitFileParser.GetValue() );
					ParameterCount++;
				}
				if ( !stricmp (InitFileParser.GetName(), "LoggingFileName") )
				{
               		strcpy(this->LoggingFileName, InitFileParser.GetValue() );
					ParameterCount++;
				}
			}
		}
	}
	else
	{
		return(-1);
	}

	return(ParameterCount);

}

