#ifndef __LWRBaseControllerInterface__
#define __LWRBaseControllerInterface__

#include <FastResearchInterface.h>
#include <friComm.h>
#include <OSAbstraction.h>
#include <errno.h>
#include <stdarg.h>

class LWRBaseControllerInterface
{

public:

	LWRBaseControllerInterface(const char *InitFileName)
	{
		this->FRI			=	new FastResearchInterface(InitFileName);
	}

	~LWRBaseControllerInterface(void)
	{
		delete this->FRI;
	}

	virtual inline int StartRobot(const float &TimeOutValueInSeconds) = 0;

	inline int StopRobot(void)
	{
		return(this->FRI->StopRobot());
	}

	inline int GetMeasuredJointPositions(float *MeasuredJointPositions)
	{
		this->FRI->GetMeasuredJointPositions(MeasuredJointPositions);

		if (this->FRI->GetFRIMode() == FRI_STATE_OFF)
		{
			return(ENOTCONN);
		}
		else
		{
			return(EOK);
		}
	}

	inline int GetMeasuredJointTorques(float *MeasuredJointTorques)
	{
		this->FRI->GetMeasuredJointTorques(MeasuredJointTorques);

		if (this->FRI->GetFRIMode() == FRI_STATE_OFF)
		{
			return(ENOTCONN);
		}
		else
		{
			return(EOK);
		}
	}

	inline int WaitForKRCTick(const unsigned int &TimeoutValueInMicroSeconds = 0)
	{
		return(this->FRI->WaitForKRCTick(TimeoutValueInMicroSeconds));
	}

	inline int WaitForTimerTick(const unsigned int &TimeoutValueInMicroSeconds = 0)
	{
		return(this->FRI->WaitForTimerTick(TimeoutValueInMicroSeconds));
	}

	inline bool IsMachineOK(void)
	{
		return(this->FRI->IsMachineOK());
	}

	inline float GetCycleTime(void)
	{
		return(this->FRI->GetFRICycleTime());
	}

	inline const char* GetCompleteRobotStateAndInformation(void)
	{
		return(FRI->GetCompleteRobotStateAndInformation());
	}

	inline int printf(const char* Format, ...)
	{
		int			Result		=	0;
	    va_list		ListOfArguments;

		va_start(ListOfArguments, Format);
		Result = FRI->printf(Format, ListOfArguments);
		va_end(ListOfArguments);

		return(Result);
	}

	inline int PrepareLogging(const char *FileIdentifier = NULL)
	{
		return(FRI->PrepareLogging(FileIdentifier));
	}

	inline int StartLogging(void)
	{
		return(FRI->StartLogging());
	}

	inline int StopLogging(void)
	{
		return(FRI->StopLogging());
	}

	inline int WriteLoggingDataFile(void)
	{
		return(FRI->WriteLoggingDataFile());
	}

protected:
	FastResearchInterface		*FRI;

};	

#endif
