//  ---------------------- Doxygen info ----------------------
//! \file LWRJointImpedanceControlExample.cpp
//!
//! \brief
//! Sample application for the class LWRJointImpedanceController
//!
//! \details
//! This simple application feature a sample of how to use the
//! joint impedance controller of the KUKA Fast Research Interface
//! for the Light-Weight Robot IV. For details about the actual
//! interface class (i.e., class LWRJointImpedanceController), please
//! refer to the file LWRJointImpedanceController.h.
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


#include <LWRJointImpedanceController.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <math.h>


#define NUMBER_OF_JOINTS		7

#define RUN_TIME_IN_SECONDS		10.0


//*******************************************************************************************
// main()
//
int main(int argc, char *argv[])
{
	unsigned int				CycleCounter	=	0
							,	i				=	0;

	int							ResultValue		=	0;

	float						CommandedTorquesInNm	[NUMBER_OF_JOINTS]
	     					,	CommandedStiffness		[NUMBER_OF_JOINTS]
	     					,	CommandedDamping		[NUMBER_OF_JOINTS]
	     					,	MeasuredTorquesInNm		[NUMBER_OF_JOINTS]
	     					,	JointValuesInRad		[NUMBER_OF_JOINTS];

	LWRJointImpedanceController	*Robot;

	Robot	=	new LWRJointImpedanceController("/home/manuelb/FRI/FRILibrary/etc/980241-FRI-Driver.init");

	fprintf(stdout, "RobotJointImpedanceController object created. Starting the robot...\n");

	for (i = 0; i < NUMBER_OF_JOINTS; i++)
	{
		CommandedStiffness	[i]	=	(float)200.0;
		CommandedDamping	[i]	=	(float)0.7;
		CommandedTorquesInNm[i]	=	(float)0.0;
	}

	Robot->SetCommandedJointStiffness	(CommandedStiffness		);
	Robot->SetCommandedJointDamping		(CommandedDamping		);
	Robot->SetCommandedJointTorques		(CommandedTorquesInNm	);

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

	Robot->GetMeasuredJointPositions(JointValuesInRad);

	fprintf(stdout, "Performing joint impedance control for %.1f seconds.\n", RUN_TIME_IN_SECONDS);

	while ((float)CycleCounter * Robot->GetCycleTime() < RUN_TIME_IN_SECONDS)
	{
		Robot->WaitForKRCTick();

		if (!Robot->IsMachineOK())
		{
			fprintf(stderr, "ERROR, the machine is not ready anymore.\n");
			break;
		}

		Robot->GetMeasuredJointTorques		(MeasuredTorquesInNm	);
		Robot->GetMeasuredJointPositions	(JointValuesInRad		);

		Robot->SetCommandedJointPositions	(JointValuesInRad);
		Robot->SetCommandedJointStiffness	(CommandedStiffness		);
		Robot->SetCommandedJointDamping		(CommandedDamping		);
		Robot->SetCommandedJointTorques		(CommandedTorquesInNm	);

		CycleCounter++;
	}

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
