#ifndef __LWRJointPositionController__
#define __LWRJointPositionController__
#include <FastResearchInterface.h>
#include <LWRBaseControllerInterface.h>
class LWRJointPositionController : public LWRBaseControllerInterface
{

public:

	LWRJointPositionController(const char *InitFileName):LWRBaseControllerInterface(InitFileName)
	{
	}

	~LWRJointPositionController(void)
	{
	}

	inline int StartRobot(const float &TimeOutValueInSeconds	=	120.0)
	{
		this->printf("Please start up the robot now by using KUKA Control Panel.\n");

		return(this->FRI->StartRobot(		FastResearchInterface::JOINT_POSITION_CONTROL
		                             	,	TimeOutValueInSeconds));
	}

	inline void SetCommandedJointPositions(const float *CommandedJointPositions)
	{
		return(this->FRI->SetCommandedJointPositions(CommandedJointPositions));
	}

};

#endif