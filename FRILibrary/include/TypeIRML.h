//  ---------------------- Doxygen info ----------------------
//! \file TypeIRML.h
//!
//! \brief
//! Header file for the API of the Reflexxes Motion Library (Type I)
//!
//! \details
//! This file contains the user interface of the Type I On-Line
//! Trajectory Generation algorithm. Besides the constructor of the class
//! TypeIRML, the two most important methods are 
//! TypeIRML::GetNextMotionState_Position and
//! TypeIRML::GetNextMotionState_Velocity.
//! \n
//! \n
//! <b>GNU Lesser Public License</b>
//! \n
//! \n
//! This file is part of the Reflexxes Motion Library (Type I).
//! \n\n
//! The Reflexxes Motion Library (Type I) is free software: you can redistribute
//! it and/or modify it under the terms of the GNU General Public License
//! as published by the Free Software Foundation, either version 3 of the
//! License, or (at your option) any later version.
//! \n\n
//! The Reflexxes Motion Library (Type I) is distributed in the hope that it
//! will be useful, but WITHOUT ANY WARRANTY; without even the implied 
//! warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See
//! the GNU General Public License for more details.
//! \n\n
//! You should have received a copy of the GNU General Public License
//! along with the Reflexxes Motion Library (Type I). If not, see 
//! http://www.gnu.org/licenses.
//! \n
//! \n
//! \n
//! Reflexxes GmbH\n
//! Sandknoell 7\n
//! D-24805 Hamdorf\n
//! GERMANY\n
//! \n
//! http://www.reflexxes.com\n
//!
//! \date November 2011
//! 
//! \version 1.1
//!
//!	\author Reflexxes GmbH, <info@reflexxes.com> \n
//!	
//!
//!
//  ----------------------------------------------------------
//   For a convenient reading of this file's source code,
//   please use a tab width of four characters.
//  ----------------------------------------------------------


#ifndef __TypeIRML__
#define __TypeIRML__

#include <TypeIRMLMath.h>
#include <TypeIRMLVector.h>
#include <TypeIRMLInputParameters.h>
#include <TypeIRMLOutputParameters.h>
#include <TypeIRMLPolynomial.h>


using namespace TypeIRMLMath;

//  ---------------------- Doxygen info ----------------------
//! \class TypeIRML
//! 
//! \brief
//! This class contains the API of the Reflexxes Motion Library (Type I)
//!
//! \details
//! The algorithm works with an arbitrary number of degrees of freedom.
//! It can handle and proceed with any state of
//! motion, that is, a new motion trajectory can be generated in any
//! situation after any sensor event within the same control cycle.
//! Further and detailed information can be found in\n
//! \n
//! <b>T. Kroeger.\n
//! On-Line Trajectory Generation in Robotic Systems.\n
//! Springer Tracts in Advanced Robotics, Vol. 58, Springer, January 2010.\n
//! <a href="http://www.springer.com/978-3-642-05174-6" target="_blanc" title="You may order your personal copy at www.springer.com.">http://www.springer.com/978-3-642-05174-6</a></b>.\n\n
//  ----------------------------------------------------------
class TypeIRML
{
public:

//  ---------------------- Doxygen info ----------------------
//! \fn TypeIRML(const unsigned int &NoOfDOFs, const double &CycleTimeInSeconds)
//! 
//! \brief
//! Constructor of the class TypeIRML
//! 
//! \param NoOfDOFs
//! Indicates the number of degrees of freedom
//! 
//! \param CycleTimeInSeconds
//! Indicates the cycle time in seconds for the generator. A typical
//! value is 0.001 (seconds).
//  ----------------------------------------------------------
	TypeIRML				(		const unsigned int			&NoOfDOFs
								,	const double				&CycleTimeInSeconds  );
	
//  ---------------------- Doxygen info ----------------------
//! \fn ~TypeIRML(void)
//! 
//! \brief
//! Destructor of the class TypeIRML
//  ----------------------------------------------------------
	~TypeIRML				(		void	);


//  ---------------------- Doxygen info ----------------------
//! \enum TypeIRMLResult
//! 
//! \brief
//! Result values of the methods TypeIRML::GetNextMotionState_Position()
//! and TypeIRML::GetNextMotionState_Velocity()
//  ----------------------------------------------------------
	enum TypeIRMLResult
	{
		//! \brief
		//! A general error.
		RML_ERROR								=	-1,
		//! \brief
		//! The specified maximum velocity value of at least one
		//! degree of freedom is below the threshold of RML_MIN_VALUE_FOR_MAXVELOCITY.
		RML_MAX_VELOCITY_ERROR					=	-200,
		//! \brief
		//! The specified maximum acceleration value of at least one
		//! degree of freedom is below the threshold of RML_MIN_VALUE_FOR_MAXACCELERATION.
		RML_MAX_ACCELERATION_ERROR				=	-201,
		//! \brief
		//! The on-line trajectory generation algorithm is working; the final
		//! state of motion has not been reached yet.
		RML_WORKING								=	0,
		//! \brief
		//! The desired final state of motion has been reached.
		RML_FINAL_STATE_REACHED					=	1,
	};


//  ---------------------- Doxygen info ----------------------
//! \fn int GetNextMotionState_Position(const double* CurrentPosition, const double* CurrentVelocity, const double* MaxVelocity, const double* MaxAcceleration, const double* TargetPosition, const bool* SelectionVector, double* NewPosition, double* NewVelocity)
//! 
//! \brief
//! Calculates new position \f$ \vec{P}_{i+1} \f$ and velocity
//! \f$ \vec{V}_{i+1} \f$ values for target position-based trajectory generation
//! 
//! \param CurrentPosition
//! The current position \f$ \vec{P}_{i} \f$ (position of the last
//! cycle, given by the user)
//! 
//! \param CurrentVelocity
//! The current velocity \f$ \vec{V}_{i} \f$ (velocity of the last
//! cycle, given by the user)
//!
//! \param MaxVelocity
//! The maximum velocity \f$ \vec{V}^{\,max}_{i} \f$ (given by the user)
//!
//! \param MaxAcceleration
//! The maximum acceleration \f$ \vec{A}^{\,max}_{i} \f$ (given by the user)
//!
//! \param TargetPosition
//! The target Position \f$ \vec{P}^{\,trgt}_{i} \f$ (given by the user)
//!
//! \param
//! SelectionVector The selection vector \f$ \vec{S}_{i} \f$, which
//! determines, which DOFs are position controlled by the RML (of
//! the current control cycle, given by the user)
//!
//! \param NewPosition
//! Pointer to an array of double values for the new position
//! \f$ \vec{P}_{i+1} \f$ (position of the current control cycle,
//! to be calculated)
//!
//! \param NewVelocity
//! Pointer to an array of double values for the new velocity
//! \f$ \vec{V}_{i+1} \f$ (velocity of the current control cycle,
//! to be calculated)
//!
//! \return
//!  - TypeIRML::RML_MAX_VELOCITY_ERROR
//!  - TypeIRML::RML_MAX_ACCELERATION_ERROR
//!  - TypeIRML::RML_ERROR
//!  - TypeIRML::RML_WORKING 
//!  - TypeIRML::RML_FINAL_STATE_REACHED
//!
//! \sa TypeIRML::TypeIRMLResult
//  ----------------------------------------------------------
	int GetNextMotionState_Position	(		const double*	CurrentPosition
										,	const double*	CurrentVelocity
										,	const double*	MaxVelocity
										,	const double*	MaxAcceleration
										,	const double*	TargetPosition
										,	const bool*		SelectionVector
										,	double*			NewPosition
										,	double*			NewVelocity	);


//  ---------------------- Doxygen info ----------------------
//! \fn int GetNextMotionState_Position(const TypeIRMLInputParameters &IP, TypeIRMLOutputParameters *OP)
//! 
//! \brief
//! Calculates new position \f$ \vec{P}_{i+1} \f$ and velocity
//! \f$ \vec{V}_{i+1} \f$ values for target position-based trajectory generation
//! 
//! \param IP
//! Input parameters for the current control cycle
//! 
//! \param OP
//! Output parameters for the current control cycle
//!
//! \return
//!  - TypeIRML::RML_MAX_VELOCITY_ERROR
//!  - TypeIRML::RML_MAX_ACCELERATION_ERROR
//!  - TypeIRML::RML_ERROR
//!  - TypeIRML::RML_WORKING 
//!  - TypeIRML::RML_FINAL_STATE_REACHED
//!
//! \sa TypeIRML::TypeIRMLResult
//! \sa TypeIRMLMath::TypeIRMLInputParameters
//! \sa TypeIRMLMath::TypeIRMLOutputParameters
//  ----------------------------------------------------------										
	int GetNextMotionState_Position	(		const TypeIRMLInputParameters	&IP
										,	TypeIRMLOutputParameters		*OP	);


//  ---------------------- Doxygen info ----------------------
//! \fn int GetNextMotionState_Velocity(const double* CurrentPosition, const double* CurrentVelocity, const double* MaxAcceleration, const double* TargetVelocity, const bool* SelectionVector, double* NewPosition, double* NewVelocity)
//!
//! \brief
//! Calculates new position \f$ \vec{P}_{i+1} \f$ and velocity
//! \f$ \vec{V}_{i+1} \f$ values for target velocity-based trajectory generation
//!
//! \param CurrentPosition
//! The current position \f$ (\vec{P}_{i} \f$ (position of the last
//! cycle, given by the user)
//!
//! \param CurrentVelocity
//! The current velocity \f$ (\vec{V}_{i} \f$ (velocity of the last
//! cycle, given by the user)
//!
//! \param MaxAcceleration
//! The maximum acceleration \f$ \vec{A}^{\,max}_{i} \f$ (given by the user)
//!
//! \param TargetVelocity
//! The target velocity \f$ (\vec{V}_{i}^{\,trgt} \f$ (velocity to be
//! reached, given by the user)
//!
//! \param SelectionVector
//! The selection vector \f$ \vec{s}_{i} \f$, which determines, which DOFs
//! are position controlled by the RML (of the current control cycle,
//! given by the user)
//!
//! \param NewPosition
//! Pointer to an array of double values for the new position
//! \f$ (\vec{p}_{i} \f$ (position of the current control cycle,
//! to be calculated)
//!
//! \param NewVelocity
//! Pointer to an array of double values for the new velocity
//! \f$ (\vec{v}_{i} \f$ (velocity of the current control cycle,
//! to be calculated)
//!
//! \return
//!  - TypeIRML::RML_MAX_ACCELERATION_ERROR
//!  - TypeIRML::RML_ERROR
//!  - TypeIRML::RML_WORKING 
//!  - TypeIRML::RML_FINAL_STATE_REACHED
//!
//! \sa TypeIRML::TypeIRMLResult
//  ----------------------------------------------------------
	int GetNextMotionState_Velocity	(		const double*	CurrentPosition
										,	const double*	CurrentVelocity
										,	const double*	MaxAcceleration
										,	const double*	TargetVelocity
										,	const bool*		SelectionVector
										,	double*			NewPosition
										,	double*			NewVelocity	);


//  ---------------------- Doxygen info ----------------------
//! \fn int GetNextMotionState_Velocity(const TypeIRMLInputParameters &IP, TypeIRMLOutputParameters *OP)
//!
//! \brief
//! Calculates new position \f$ \vec{P}_{i+1} \f$ and velocity
//! \f$ \vec{V}_{i+1} \f$ values for target velocity-based trajectory generation
//!
//! \param IP
//! Input parameters for the current control cycle
//! 
//! \param OP
//! Output parameters for the current control cycle
//!
//! \return
//!  - TypeIRML::RML_MAX_ACCELERATION_ERROR
//!  - TypeIRML::RML_ERROR
//!  - TypeIRML::RML_WORKING 
//!  - TypeIRML::RML_FINAL_STATE_REACHED
//!
//! \sa TypeIRML::TypeIRMLResult
//! \sa TypeIRMLMath::TypeIRMLInputParameters
//! \sa TypeIRMLMath::TypeIRMLOutputParameters
//  ----------------------------------------------------------							
	int GetNextMotionState_Velocity	(		const TypeIRMLInputParameters	&IP
										,	TypeIRMLOutputParameters		*OP	);										


//  ---------------------- Doxygen info ----------------------
//! \fn double GetExecutionTime(void) const
//!
//! \brief
//! Returns the time in seconds that is required to time-optimally
//! reach the desired target state of motion
//!
//! \details
//! If this method is called after the execution of the method
//! TypeIRML::GetNextMotionState_Position(), the time value for the
//! position-based trajectory will be returned. If this method is
//! called after the execution of the method
//! TypeIRML::GetNextMotionState_Velocity(), the time value for the
//! velocity-based trajectory will be returned, i.e., the time until
//! which the last degree of freedom will reach its target velocity.
//! For the case, the final and desired state of motion has already
//! been reached (i.e., the result of
//! TypeIRML::GetNextMotionState_Position()
//! or TypeIRML::GetNextMotionState_Velocity() was
//! TypeIRML::RML_FINAL_STATE_REACHED), this method returns a zero
//! value.
//!
//! \return The value of the execution time in seconds. If neither the
//! method TypeIRML::GetNextMotionState_Position()
//! nor the method TypeIRML::GetNextMotionState_Velocity() was not
//! called before, the returned value is -1.0.
//!
//! \sa TypeIRML::GetNextMotionState_Position()
//! \sa TypeIRML::GetNextMotionState_Velocity()
//  ----------------------------------------------------------
	double	GetExecutionTime(void) const;


private:


//  ---------------------- Doxygen info ----------------------
//! \fn double CalculateMinimumSynchronizationTime(const TypeIRMLMath::TypeIRMLInputParameters &IP) const
//!
//! \brief
//! Calculates and returns the minimum synchronization time
//!
//! \details
//! This method computes the minimum execution time for each selected DOF,
//! determines the maximum one, and returns this value.
//!
//! \param IP
//! Pointer to the input parameters of the current call of
//! GetNextMotionState_Position()
//!
//! \remark
//! In the context of the on-line trajectory generation framework, this
//! method is considered as <em>Step 1</em>.
//!
//! \return
//! The value of the minimum synchronization time in seconds.
//  ----------------------------------------------------------
	double CalculateMinimumSynchronizationTime(const TypeIRMLMath::TypeIRMLInputParameters &IP) const;


//  ---------------------- Doxygen info ----------------------
//! \fn void SynchronizeTrajectory(const TypeIRMLMath::TypeIRMLInputParameters &IP, const double &SynchronizationTime, TypeIRMLMath::TypeIMotionPolynomials *PolynomialArray)
//!
//! \brief
//! Computes time-synchronized trajectories for all selected DOFs
//!
//! \details
//! This method computes all polynomial parameters \c PolynomialArray of
//! all selected DOFs. These calculations are based on the input parameters
//! that are given through the call of GetNextMotionState_Position() and
//! and the minimum synchronization time \c SynchronizationTime.
//! 
//! \param IP
//! Pointer to the input parameters of the current call of
//! GetNextMotionState_Position()
//!
//! \param SynchronizationTime
//! Minimum synchronization time in seconds
//! 
//! \param PolynomialArray
//! Pointer to an array of polynomials
//! (TypeIRMLMath::TypeIMotionPolynomials)
//!
//! \remark
//! In the context of the on-line trajectory generation framework, this
//! method is considered as <em>Step 2</em>.
//  ----------------------------------------------------------
	void SynchronizeTrajectory(		const TypeIRMLMath::TypeIRMLInputParameters		&IP
								,	const double									&SynchronizationTime
								,	TypeIRMLMath::TypeIMotionPolynomials			*PolynomialArray);


//  ---------------------- Doxygen info ----------------------
//! \fn int CalculateOutputValues( const TypeIRMLMath::TypeIMotionPolynomials *PolynomialArray, const TypeIRMLBoolVector &SelectionVector, TypeIRMLMath::TypeIRMLOutputParameters *OP)
//!
//! \brief
//! Computes the output values for of the method GetNextMotionState_Position()
//!
//! \details
//! Based on all polynomial parameters that have been calculated by the
//! method SynchronizeTrajectory(), this method computes the output values
//! for of the method GetNextMotionState_Position(), that is, the state of
//! motion for the next control cycle.
//! 
//! \param PolynomialArray
//! Pointer to the array of polynomials
//! (TypeIRMLMath::TypeIMotionPolynomials) that have been parameterized by
//! SynchronizeTrajectory().
//! 
//! \param SelectionVector
//! Selection vector of the current set of input parameters. Only if the
//! flag for a DOF is set to true, output values will be set by this
//! method.
//!
//! \param OP
//! Output values of GetNextMotionState_Position(), that is, the state of
//! motion for the next control cycle.
//!
//! \remark
//! In the context of the on-line trajectory generation framework, this
//! method is considered as <em>Step 3</em>.
//!
//! \return
//!  - TypeIRML::RML_ERROR
//!  - TypeIRML::RML_WORKING 
//!  - TypeIRML::RML_FINAL_STATE_REACHED
//  ----------------------------------------------------------
	int CalculateOutputValues(		const TypeIRMLMath::TypeIMotionPolynomials		*PolynomialArray
								,	const TypeIRMLBoolVector						&SelectionVector
								,	TypeIRMLMath::TypeIRMLOutputParameters			*OP);


//  ---------------------- Doxygen info ----------------------
//! \var int NumberOfDOFs
//! 
//! \brief
//! Number of degrees of freedom.
//!
//! \sa TypeIRML::TypeIRML()
//  ----------------------------------------------------------
	int								NumberOfDOFs						;


//  ---------------------- Doxygen info ----------------------
//! \var double CycleTime
//! 
//! \brief
//! Cycle time in seconds
//!
//! \sa TypeIRML::TypeIRML()
//  ----------------------------------------------------------
	double							CycleTime							;


//  ---------------------- Doxygen info ----------------------
//! \var double InternalClockInSeconds
//! 
//! \brief
//! Internal time. This time will set to zero if the input values have
//! changed. Otherwise, this value will be incremented by \c CycleTime
//! during every call of GetNextMotionState_Position() or
//! GetNextMotionState_Velocity()
//  ----------------------------------------------------------
	double							InternalClockInSeconds				;
	

//  ---------------------- Doxygen info ----------------------
//! \var double TrajectoryExecutionTimeForTheUser
//! 
//! \brief
//! Time in seconds until the final state of motion has been reached
//  ----------------------------------------------------------
	double							TrajectoryExecutionTimeForTheUser	;


//  ---------------------- Doxygen info ----------------------
//! \var TypeIRMLMath::TypeIRMLInputParameters *CurrentInputParameters
//! 
//! \brief
//! Pointer to an object of the class TypeIRMLInputParameters
//  ----------------------------------------------------------
	TypeIRMLMath::TypeIRMLInputParameters		*CurrentInputParameters;


//  ---------------------- Doxygen info ----------------------
//! \var TypeIRMLMath::TypeIRMLOutputParameters *OutputParameters
//! 
//! \brief
//! Pointer to an object of the class TypeIRMLOutputParameters
//  ----------------------------------------------------------
	TypeIRMLMath::TypeIRMLOutputParameters		*OutputParameters;


//  ---------------------- Doxygen info ----------------------
//! \var TypeIRMLMath::TypeIMotionPolynomials *Polynomials
//! 
//! \brief
//! Pointer to an array of TypeIMotionPolynomials objects
//  ----------------------------------------------------------
	TypeIRMLMath::TypeIMotionPolynomials		*Polynomials;

};



#endif