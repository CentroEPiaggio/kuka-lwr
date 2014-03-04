#include <FastResearchInterface.h>
#include <Console.h>
#include <errno.h>
#include <string.h>
#include <OSAbstraction.h>
#include <FastResearchInterfaceTest.h>
#include <TypeIRML.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#ifndef PI
#define PI			3.1415926535897932384626433832795
#endif

#ifndef RAD
#define RAD(A)	((A) * PI / 180.0 )
#endif

#ifndef DEG
#define DEG(A)	((A) * 180.0 / PI )
#endif  

#define NUMBER_OF_JOINTS			7
   
// ****************************************************************
// PlayPointsSimple()
//
void PlayPointsSimple(FastResearchInterface *FRI)
{


	unsigned int				i							=	0		;

	int							ResultValue					=	0		;


	float						JointValuesInRad[NUMBER_OF_JOINTS]		;
	float						errorJoint[NUMBER_OF_JOINTS]		;

	double						CycleTime					=	0.002	;

	TypeIRML					*RML						=	NULL	;

	TypeIRMLInputParameters		*IP							=	NULL	;

	TypeIRMLOutputParameters	*OP							=	NULL	;

	RML					=	new TypeIRML(		NUMBER_OF_JOINTS
											,	CycleTime			);

	IP					=	new TypeIRMLInputParameters(NUMBER_OF_JOINTS);

	OP					=	new TypeIRMLOutputParameters(NUMBER_OF_JOINTS);

	memset(JointValuesInRad						, 0x0, NUMBER_OF_JOINTS * sizeof(float));
	
	FILE *record_points;
	record_points = fopen("points.txt", "r");
	
	if ((FRI->GetCurrentControlScheme() != FastResearchInterface::JOINT_POSITION_CONTROL) || (!FRI->IsMachineOK()))
	{	
		printf("Program is going to stop the robot.\n");
		FRI->StopRobot();

		FRI->GetMeasuredJointPositions(JointValuesInRad);
		FRI->SetCommandedJointPositions(JointValuesInRad);
				
		printf("Restarting the joint position control scheme.\n");
		ResultValue	=	FRI->StartRobot(FastResearchInterface::JOINT_POSITION_CONTROL);
		
		if ((ResultValue != EOK) && (ResultValue != EALREADY))
		{
			printf("An error occurred during starting up the robot...\n");
			delete	RML;
			delete	IP;
			delete	OP;
			
			return;	
		}

	}
	
	FRI->GetMeasuredJointPositions(JointValuesInRad);
	
	double qf[7];

	while (fscanf(record_points,"%lf %lf %lf %lf %lf %lf %lf %d %d\n", &qf[0],&qf[1],&qf[2],&qf[3],&qf[4],&qf[5],&qf[6]) != EOF)
	{
	
		for ( i = 0; i < NUMBER_OF_JOINTS; i++)
		{
			IP->CurrentPosition->VecData		[i] =	(double)DEG(JointValuesInRad[i]);
			IP->TargetPosition->VecData			[i]	=	qf[i];
			IP->MaxVelocity->VecData			[i] =	(double)50.0;		
			IP->MaxAcceleration->VecData		[i] =	(double)40.0;
			IP->SelectionVector->VecData		[i] =	true;		
		}
		
		ResultValue	=	TypeIRML::RML_WORKING;
	
		while ((FRI->IsMachineOK()) && (ResultValue != TypeIRML::RML_FINAL_STATE_REACHED))
		{
			FRI->WaitForKRCTick();

			ResultValue	=	RML->GetNextMotionState_Position(		*IP
																,	OP	);

			if ((ResultValue != TypeIRML::RML_WORKING) && (ResultValue != TypeIRML::RML_FINAL_STATE_REACHED))
			{
				printf("MoveToCandle(): ERROR during trajectory generation (%d).", ResultValue);
			}
												
			for ( i = 0; i < NUMBER_OF_JOINTS; i++)
			{
				JointValuesInRad[i]	=	RAD((double)(OP->NewPosition->VecData[i]));
			}

			FRI->SetCommandedJointPositions(JointValuesInRad);	
			
			*(IP->CurrentPosition)		=	*(OP->NewPosition);
			*(IP->CurrentVelocity)		=	*(OP->NewVelocity);
		}

		
	}


	if (!FRI->IsMachineOK())
	{
		printf("RunTrajectorySimple(): ERROR, machine is not ready.");
		
		delete	RML;
		delete	IP;
		delete	OP;
		
		return;
	}
	
	printf("Stopping the robot.\n");
	FRI->StopRobot();	

	delete	RML;
	delete	IP;
	delete	OP;
	
	return;
}


