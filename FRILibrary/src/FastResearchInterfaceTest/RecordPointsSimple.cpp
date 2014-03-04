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
// RecordPointsSimple()
//
void RecordPointsSimple(FastResearchInterface *FRI)
{

  FILE *record_points;
  record_points = fopen("points.txt","wt");  


	unsigned int				i							=	0		;

	int							ResultValue					=	0		;


	float						JointValuesInRad[NUMBER_OF_JOINTS]		;
	float						errorJoint[NUMBER_OF_JOINTS]		;
	float	  				JointStiffnessValues[LBR_MNJ]
	     				,	JointDampingValues[LBR_MNJ]
						,	CartStiffnessValues[FRI_CART_VEC]
	     				,	CartDampingValues[FRI_CART_VEC];

	double						CycleTime					=	0.002	;

	TypeIRML					*RML						=	NULL	;

	TypeIRMLInputParameters		*IP							=	NULL	;

	TypeIRMLOutputParameters	*OP							=	NULL	;

	RML					=	new TypeIRML(		NUMBER_OF_JOINTS
											,	CycleTime			);

	IP					=	new TypeIRMLInputParameters(NUMBER_OF_JOINTS);

	OP					=	new TypeIRMLOutputParameters(NUMBER_OF_JOINTS);

	memset(JointValuesInRad						, 0x0, NUMBER_OF_JOINTS * sizeof(float));
	
	if ((FRI->GetCurrentControlScheme() != FastResearchInterface::JOINT_IMPEDANCE_CONTROL) || (!FRI->IsMachineOK()))
	{	
		printf("Program is going to stop the robot.\n");
		FRI->StopRobot();

		FRI->GetMeasuredJointPositions(JointValuesInRad);
		FRI->SetCommandedJointPositions(JointValuesInRad);
				
		printf("Restarting the joint position control scheme.\n");
		ResultValue	=	FRI->StartRobot(FastResearchInterface::JOINT_IMPEDANCE_CONTROL);
		
		if ((ResultValue != EOK) && (ResultValue != EALREADY))
		{
			printf("An error occurred during starting up the robot...\n");
			delete	RML;
			delete	IP;
			delete	OP;
			
			return;	
		}
	}
	
		
	for (i = 0; i < LBR_MNJ; i++) //look if it is posible to implement in the robot with any errors
	{
		JointStiffnessValues	[i] =	(float)10.0;
		JointDampingValues		[i]	=	(float)0.7;
	}

	FRI->SetCommandedJointDamping(JointDampingValues);
	FRI->SetCommandedJointStiffness(JointStiffnessValues);
	
	int option = 1;
	while ((option == 1) || (option == 2))
	{	
		FRI->WaitForKRCTick();
		printf ("\n2 for record 1 do nothing  and other for exit");
		printf("\noption");
		scanf("%d",&option);
		FRI->GetMeasuredJointPositions(JointValuesInRad);
		if 	(option == 2)
		{
			for ( i = 0; i < NUMBER_OF_JOINTS; i++)
			{
				IP->CurrentPosition->VecData		[i] =	(double)DEG(JointValuesInRad[i]);
				fprintf(record_points,"%f ", IP->CurrentPosition->VecData		[i]);
			}
			fprintf(record_points,"\n");		
		}
	}
	printf("\nfinishing recording");
	
	if (!FRI->IsMachineOK())
	{
		printf("RunTrajectorySimple(): ERROR, machine is not ready.");
		
		delete	RML;
		delete	IP;
		delete	OP;
		
		return;
	}
	
	fclose(record_points); 
	
	//FRI->StopRobot();	
	printf("Stopping the robot.\n");

	delete	RML;
	delete	IP;
	delete	OP;
  
	return;

}


