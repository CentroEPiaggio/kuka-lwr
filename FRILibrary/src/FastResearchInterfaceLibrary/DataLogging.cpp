//  ---------------------- Doxygen info ----------------------
//! \file DataLogging.cpp
//!
//! \brief
//! Implementation file for the class DataLogging
//!
//! \details
//! The class DataLogging provides the possibility of writing all important
//! data that is exchanged between the remote host and the KRC unit to an
//! output file. For further details, please refer to the file
//! DataLogging.h
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


#include <DataLogging.h>
#include <FastResearchInterface.h>
#include <OSAbstraction.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <errno.h>
#include <string.h>



#define NUMBER_OF_ELEMENTS_PER_ENTRY	58
#define OUTPUT_FILE_STRING_LENGTH		1024
#define TIME_STRING_LENGTH				128

#ifndef PI
#define PI			3.1415926535897932384626433832795
#endif

#ifndef RAD
#define RAD(A)	((A) * PI / 180.0 )
#endif

#ifndef DEG
#define DEG(A)	((A) * 180.0 / PI )
#endif


// ****************************************************************
// Constructor 
//
DataLogging::DataLogging(		const char			*RobotName
							,	const char			*LoggingPath
                           	,	const char			*LoggingFileName
                           	,	const unsigned int	&MaxNumberOfEntries)
{
	unsigned int			i		=	0;

	this->MachineName				=	(char*)RobotName;
	this->OutputPath				=	(char*)LoggingPath;
	this->OutputFileName			=	(char*)LoggingFileName;
	this->MaximumNumberOfEntries	=	MaxNumberOfEntries;
	this->CurrentObjectState		=	DataLogging::WriteToFileCalled;
	this->OutputFileHandler			=	NULL;
	this->OutputCounter				=	0;
	this->CurrentControlScheme		=	0;

	this->LoggingMemory				=	new float*[NUMBER_OF_ELEMENTS_PER_ENTRY];

	for (i = 0; i < NUMBER_OF_ELEMENTS_PER_ENTRY; i++)
	{
		this->LoggingMemory[i]		=	new float[MaxNumberOfEntries];

		memset(		this->LoggingMemory[i]
		       ,	0x0
		       ,	this->MaximumNumberOfEntries * sizeof(float)	);
	}

	this->CompleteOutputFileString	=	new char[OUTPUT_FILE_STRING_LENGTH];

	memset(		this->CompleteOutputFileString
	       	,	0x0
	       	,	OUTPUT_FILE_STRING_LENGTH * sizeof(char));

}


// ****************************************************************
// Destructor
//
DataLogging::~DataLogging(void)
{
	unsigned int			i		=	0;

	if (this->CurrentObjectState == DataLogging::PrepareLoggingCalled)
	{
		this->WriteToFile();
	}

	for (i = 0; i < NUMBER_OF_ELEMENTS_PER_ENTRY; i++)
	{
		delete[]	LoggingMemory[i];
	}

	delete[]	LoggingMemory;
	delete[]	this->CompleteOutputFileString;
}


// ****************************************************************
// PrepareLogging()
//
int DataLogging::PrepareLogging(		const unsigned int	&ControlScheme
                                	,	const char			*FileIdentifier)
{
	char				TimeString[TIME_STRING_LENGTH];

	unsigned int		i		=	0;

	time_t				CurrentDayTime;

	memset(		TimeString
		    ,	0x0
		    ,	TIME_STRING_LENGTH * sizeof(char));

	this->CurrentControlScheme	=	ControlScheme;

	if (this->CurrentObjectState	==	DataLogging::PrepareLoggingCalled)
	{
		this->WriteToFile();
	}

	GetSystemTimeInSeconds(true);



	//REMOVE

	//------------------------
	
#ifdef _NTO_	
	
	struct _clockperiod 	ClockResolution;

	ClockResolution.nsec = 10000;	//ns
	ClockResolution.fract = 0;

	ClockPeriod(CLOCK_REALTIME, &ClockResolution, NULL, 0);
	//------------------------
	
#endif

	memset(		this->CompleteOutputFileString
	       	,	0x0
	       	,	OUTPUT_FILE_STRING_LENGTH * sizeof(char));

	for (i = 0; i < NUMBER_OF_ELEMENTS_PER_ENTRY; i++)
	{
		memset(		this->LoggingMemory[i]
		       ,	0x0
		       ,	this->MaximumNumberOfEntries * sizeof(float)	);
	}

	CurrentDayTime = time(NULL);
	strftime(TimeString, TIME_STRING_LENGTH, "%y%m%d-%H%M%S", localtime(&CurrentDayTime));
	if (FileIdentifier == NULL)
	{
		sprintf(this->CompleteOutputFileString, "%s%s-%s-%s", this->OutputPath, TimeString, this->MachineName, this->OutputFileName);
	}
	else
	{
		sprintf(this->CompleteOutputFileString, "%s%s-%s-%s-%s", this->OutputPath, TimeString, FileIdentifier, this->MachineName, this->OutputFileName);
	}

	if ( (this->OutputFileHandler = fopen(this->CompleteOutputFileString, "w") ) == NULL)
	{
		return(EBADF);
	}
	else
	{
		fprintf(this->OutputFileHandler, "Logging file of the KUKA Fast Research Interface: %s\n", this->CompleteOutputFileString);
		fprintf(this->OutputFileHandler, "This file contains all important control values and importable to Matlab and MS Excel.\n");
		fprintf(this->OutputFileHandler, "%s\n", ctime( &CurrentDayTime));
		fprintf(this->OutputFileHandler, "Robot name: %s\n", this->MachineName);

		switch (this->CurrentControlScheme)
		{
		case FastResearchInterface::JOINT_POSITION_CONTROL:
			fprintf(this->OutputFileHandler, "Active control scheme: joint position control\n\n");
			fprintf(this->OutputFileHandler, "Counter	KRCTime	LocalTime	ActFJ1	ActFJ2	ActFJ3	ActFJ4	ActFJ5	ActFJ6	ActFJ7	UDesJ1	UDesJ2	UDesJ3	UDesJ4	UDesJ5	UDesJ6	UDesJ7	ActJ1	ActJ2	ActJ3	ActJ4	ActJ5	ActJ6	ActJ7	KDesJ1	KDesJ2	KDesJ3	KDesJ4	KDesJ5	KDesJ6	KDesJ7\n");
			break;
		case FastResearchInterface::CART_IMPEDANCE_CONTROL:
			fprintf(this->OutputFileHandler, "Active control scheme: Cartesian impedance control\n\n");
			fprintf(this->OutputFileHandler, "Counter	KRCTime	LocalTime	DesKx	DesKy	DesKz	DesKa	DesKb	DesKc	DesDx	DesDy	DesDz	DesDa	DesDb	DesDc	UDesFx	UDesFy	UDesFz	UDesFa	UDesFb	UDesFc\n");
			break;
		case FastResearchInterface::JOINT_IMPEDANCE_CONTROL:
			fprintf(this->OutputFileHandler, "Active control scheme: joint impedance control\n\n");
			fprintf(this->OutputFileHandler, "Counter	KRCTime	LocalTime	ActFJ1	ActFJ2	ActFJ3	ActFJ4	ActFJ5	ActFJ6	ActFJ7	UDesPJ1	UDesPJ2	UDesPJ3	UDesPJ4	UDesPJ5	UDesPJ6	UDesPJ7	ActPJ1	ActPJ2	ActPJ3	ActPJ4	ActPJ5	ActPJ6	ActPJ7	KDesPJ1	KDesPJ2	KDesPJ3	KDesPJ4	KDesPJ5	KDesPJ6	KDesPJ7	DesKJ1	DesKJ2	DesKJ3	DesKJ4	DesKJ5	DesKJ6	DesKJ7	DesDJ1	DesDJ2	DesDJ3	DesDJ4	DesDJ5	DesDJ6	DesDJ7	UDesFJ1	UDesFJ2	UDesFJ3	UDesFJ4	UDesFJ5	UDesFJ6	UDesFJ7	KOffPJ1	KOffPJ2	KOffPJ3	KOffPJ4	KOffPJ5	KOffPJ6	KOffPJ7\n");
			break;
		default:
			return(EINVAL);
		}
	}

	fflush(this->OutputFileHandler);

	this->CurrentObjectState	=	DataLogging::PrepareLoggingCalled;
	this->OutputCounter			=	0;

	return(EOK);
}


// ****************************************************************
// AddEntry()
//
void DataLogging::AddEntry(		const tFriMsrData		&ReceivedFRIData
                           	,	const tFriCmdData		&SentFRIData		)
{
	unsigned int			i		=	0;

	this->LoggingMemory[ 0][this->OutputCounter % this->MaximumNumberOfEntries]	=	ReceivedFRIData.intf.timestamp;
	this->LoggingMemory[ 1][this->OutputCounter % this->MaximumNumberOfEntries]	=	GetSystemTimeInSeconds();

	if (this->CurrentControlScheme == FastResearchInterface::JOINT_POSITION_CONTROL)
	{
		for (i = 0; i < LBR_MNJ; i++)
		{
			this->LoggingMemory[ 2 +               i][this->OutputCounter % this->MaximumNumberOfEntries] = 		ReceivedFRIData.data.msrJntTrq	[i]	;
			this->LoggingMemory[ 2 + 1 * LBR_MNJ + i][this->OutputCounter % this->MaximumNumberOfEntries] = DEG(	SentFRIData.cmd.jntPos			[i]);
			this->LoggingMemory[ 2 + 2 * LBR_MNJ + i][this->OutputCounter % this->MaximumNumberOfEntries] = DEG(	ReceivedFRIData.data.msrJntPos	[i]);
			this->LoggingMemory[ 2 + 3 * LBR_MNJ + i][this->OutputCounter % this->MaximumNumberOfEntries] = DEG(	ReceivedFRIData.data.cmdJntPos	[i]);
		}
	}

	if (this->CurrentControlScheme == FastResearchInterface::CART_IMPEDANCE_CONTROL)
	{
		for (i = 0; i < FRI_CART_VEC; i++)
		{
			//! \todo Add further values for the case of FastResearchInterface::CART_IMPEDANCE_CONTROL (in particular pose values).
			this->LoggingMemory[ 2 +                    i][this->OutputCounter % this->MaximumNumberOfEntries] = SentFRIData.cmd.cartStiffness		[i];
			this->LoggingMemory[ 2 + 1 * FRI_CART_VEC + i][this->OutputCounter % this->MaximumNumberOfEntries] = SentFRIData.cmd.cartDamping		[i];
			this->LoggingMemory[ 2 + 2 * FRI_CART_VEC + i][this->OutputCounter % this->MaximumNumberOfEntries] = SentFRIData.cmd.addTcpFT			[i];
		}
	}

	if (this->CurrentControlScheme == FastResearchInterface::JOINT_IMPEDANCE_CONTROL)
	{
		for (i = 0; i < LBR_MNJ; i++)
		{
			this->LoggingMemory[ 2 +               i][this->OutputCounter % this->MaximumNumberOfEntries] = 		ReceivedFRIData.data.msrJntTrq			[i]	;
			this->LoggingMemory[ 2 +   	 LBR_MNJ + i][this->OutputCounter % this->MaximumNumberOfEntries] = DEG(	SentFRIData.cmd.jntPos					[i]);
			this->LoggingMemory[ 2 + 2 * LBR_MNJ + i][this->OutputCounter % this->MaximumNumberOfEntries] = DEG(	ReceivedFRIData.data.msrJntPos			[i]);
			this->LoggingMemory[ 2 + 3 * LBR_MNJ + i][this->OutputCounter % this->MaximumNumberOfEntries] = DEG(	ReceivedFRIData.data.cmdJntPos			[i]);
			this->LoggingMemory[ 2 + 4 * LBR_MNJ + i][this->OutputCounter % this->MaximumNumberOfEntries] = 		SentFRIData.cmd.jntStiffness			[i]	;
			this->LoggingMemory[ 2 + 5 * LBR_MNJ + i][this->OutputCounter % this->MaximumNumberOfEntries] = 		SentFRIData.cmd.jntDamping				[i]	;
			this->LoggingMemory[ 2 + 6 * LBR_MNJ + i][this->OutputCounter % this->MaximumNumberOfEntries] = 		SentFRIData.cmd.addJntTrq				[i]	;
			this->LoggingMemory[ 2 + 7 * LBR_MNJ + i][this->OutputCounter % this->MaximumNumberOfEntries] = DEG(	ReceivedFRIData.data.cmdJntPosFriOffset	[i]);
		}
	}

	this->OutputCounter++;

	return;
}


// ****************************************************************
// WriteToFile()
//
int DataLogging::WriteToFile(void)
{
	int					ReturnValue			=	0;

	unsigned int		StartIndex			=	0
					,	StopIndex			=	0
					,	Counter				=	0
					,	i					=	0
					,	ElementsPerLine		=	0;

	if (this->CurrentObjectState	!=	DataLogging::PrepareLoggingCalled)
	{
		return(EPERM);
	}
	else
	{
		this->CurrentObjectState	=	DataLogging::WriteToFileCalled;
	}

	//REMOVE
	//------------------------
#ifdef _NTO_		
	struct _clockperiod 	ClockResolution;

	ClockResolution.nsec = 1000000;	//ns
	ClockResolution.fract = 0;

	ClockPeriod(CLOCK_REALTIME, &ClockResolution, NULL, 0);
#endif
	//------------------------

	if (this->OutputCounter > this->MaximumNumberOfEntries)
	{
		StartIndex	=	(this->OutputCounter + 1) % this->MaximumNumberOfEntries;
		StopIndex	=	StartIndex + this->MaximumNumberOfEntries - 1;
	}
	else
	{
		StartIndex	=	0;
		StopIndex	=	this->OutputCounter - 1;
	}

	switch (this->CurrentControlScheme)
	{
	case FastResearchInterface::JOINT_POSITION_CONTROL:
		ElementsPerLine	=	2 + 4 * LBR_MNJ;
		break;
	case FastResearchInterface::CART_IMPEDANCE_CONTROL:
		ElementsPerLine	=	2 + 3 * FRI_CART_VEC;
		break;
	case FastResearchInterface::JOINT_IMPEDANCE_CONTROL:
		ElementsPerLine	=	2 + 8 * LBR_MNJ;
		break;
	default:
		return(EINVAL);
	}

	while (StopIndex >= StartIndex)
	{
		Counter++;
		fprintf(this->OutputFileHandler, "%d", Counter);
		for (i = 0; i < ElementsPerLine; i++)
		{
			fprintf(this->OutputFileHandler, "	%12.6f", this->LoggingMemory[i][StartIndex]);
		}
		fprintf(this->OutputFileHandler, "\n");
		StartIndex++;
	}

	fflush(this->OutputFileHandler);
	ReturnValue	=	fclose(this->OutputFileHandler);

	if (ReturnValue == 0)
	{
		return(EOK);
	}
	else
	{
		return(ReturnValue);
	}
}

