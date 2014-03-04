//  ---------------------- Doxygen info ----------------------
//! \file LWRLoggingExample.cpp
//!
//! \brief
//! Sample application for the class LWRJointPositionController using the
//! data logging functionality
//!
//! \details
//! This simple application feature a sample of how to use the
//! joint position controller \em and how to use the data logging
//! functionality of the KUKA Fast Research Interface
//! for the Light-Weight Robot IV. For details about the actual
//! interface class (i.e., class LWRJointPositionController), please
//! refer to the file LWRJointPositionController.h as well as to the
//! file containing the base controller interface class
//! (i.e., class LWRBaseControllerInterface), LWRBaseControllerInterface.h.
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


#include <LWRJointPositionController.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <math.h>


#define NUMBER_OF_JOINTS	7

#ifndef PI
#define PI	3.1415926535897932384626433832795
#endif



//*******************************************************************************************
// main()
//
int main(int argc, char *argv[])
{
	unsigned int				CycleCounter	=	0;

	int							ResultValue		=	0
							,	i				=	0;

	float						FunctionValue	=	0.0
							,	LoopVariable	=	0.0
							,	JointValuesInRad[NUMBER_OF_JOINTS]
	     					,	InitialJointValuesInRad[NUMBER_OF_JOINTS];

	LWRJointPositionController	*Robot;

	Robot	=	new LWRJointPositionController("/home/manuelb/FRI/FRILibrary/etc/980241-FRI-Driver.init");

	fprintf(stdout, "RobotJointPositionController object created. Starting the robot...\n");

	ResultValue	=	Robot->StartRobot();

	if (ResultValue == EOK)
	{
		fprintf(stdout, "Robot successfully started.\n");
	}
	else
	{
		fprintf(stderr, "ERROR, could not start robot: %s\n", strerror(ResultValue));
	}

	fprintf(stdout, "Current system state:\n%s\n", Robot->GetCompleteRobotStateAndInformation());

	Robot->GetMeasuredJointPositions(InitialJointValuesInRad);

	fprintf(stdout, "Preparing data logger...\n");
	// Data logging method 1
	Robot->PrepareLogging("OptionalStringForFileIdentification");
	fprintf(stdout, "Data logger prepared.\n");

	while (LoopVariable < 5.0 * PI)
	{
		Robot->WaitForKRCTick();

		if (!Robot->IsMachineOK())
		{
			fprintf(stderr, "ERROR, the machine is not ready anymore.\n");
			break;
		}

		CycleCounter++;

		if (CycleCounter == 2000)
		{
			// Data logging method 2
			ResultValue	=	Robot->StartLogging();

			if (ResultValue == EOK)
			{
				// this printf method is real-time capable
				Robot->printf("Data logging successfully started.\n");
			}
			else
			{
				Robot->printf("ERROR, cannot start data logging.\n");
			}
		}

		if (CycleCounter == 12000)
		{
			// Data logging method 3
			ResultValue	=	Robot->StopLogging();

			if (ResultValue == EOK)
			{
				Robot->printf("Data logging successfully stopped.\n");
			}
			else
			{
				Robot->printf("ERROR, cannot stop data logging.\n");
			}
		}

		FunctionValue	=	0.3 * sin(LoopVariable);
		FunctionValue	*=	FunctionValue;

		for (i = 0; i < NUMBER_OF_JOINTS; i++)
		{
			JointValuesInRad[i]	=	InitialJointValuesInRad[i] + FunctionValue;
		}

		Robot->SetCommandedJointPositions(JointValuesInRad);

		LoopVariable	+=	(float)0.001;
	}

	fprintf(stdout, "Writing data file...\n");
	// Data logging method 4
	Robot->WriteLoggingDataFile();
	fprintf(stdout, "Data file written.\n");

	fprintf(stdout, "Stopping the robot...\n");
	ResultValue	=	Robot->StopRobot();

	if (ResultValue != EOK)
	{
		fprintf(stderr, "An error occurred during stopping the robot...\n");
	}
	else
	{
		fprintf(stdout, "Robot successfully stopped.\n");
	}

	fprintf(stdout, "Deleting the object...\n");
	delete Robot;
	fprintf(stdout, "Object deleted...\n");

	return(EXIT_SUCCESS);
}
