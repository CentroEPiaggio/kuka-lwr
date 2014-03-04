//  ---------------------- Doxygen info ----------------------
//! \file TypeIRMLProfiles.h
//!
//! \brief
//! Header file for the calculation of all motion
//! profiles for the Type I On-Line Trajectory Generation algorithm
//!
//! \details
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


#ifndef __TypeIRMLProfiles__
#define __TypeIRMLProfiles__


#include <TypeIRMLPolynomial.h>


namespace TypeIRMLMath
{	

//  ---------------------- Doxygen info ----------------------
//! \fn double ProfileStep1PosTri(const double &CurrentPosition, const double &CurrentVelocity, const double &MaxAcceleration, const double &TargetPosition)
//!
//! \brief
//! Calculates the execution time of the PosTri velocity profile
//!
//! \param CurrentPosition
//! Current position value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}P_{i} \f$
//!
//! \param CurrentVelocity
//! Current velocity value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}V_{i} \f$
//!
//! \param MaxAcceleration
//! Maximum allowed acceleration value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}A_{i}^{\,max} \f$
//!
//! \param TargetPosition
//! Target position value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}P_{i}^{\,trgt} \f$
//!
//! \return
//! Execution time for this profile in seconds
//  ----------------------------------------------------------
double ProfileStep1PosTri(		const double &CurrentPosition
							,	const double &CurrentVelocity
							,	const double &MaxAcceleration
							,	const double &TargetPosition);


//  ---------------------- Doxygen info ----------------------
//! \fn double ProfileStep1PosTrap(const double &CurrentPosition, const double &CurrentVelocity, const double &MaxVelocity, const double &MaxAcceleration, const double &TargetPosition)
//!
//! \brief Calculates the execution time of the PosTrap velocity profile
//!
//! \param CurrentPosition
//! Current position value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}P_{i} \f$
//!
//! \param CurrentVelocity
//! Current velocity value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}V_{i} \f$
//!
//! \param MaxVelocity
//! Maximum allowed velocity value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}V_{i}^{\,max} \f$
//!
//! \param MaxAcceleration
//! Maximum allowed acceleration value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}A_{i}^{\,max} \f$
//!
//! \param TargetPosition
//! Target position value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}P_{i}^{\,trgt} \f$
//!
//! \return
//! Execution time for this profile in seconds
//  ----------------------------------------------------------
double ProfileStep1PosTrap(		const double &CurrentPosition
							,	const double &CurrentVelocity
							,	const double &MaxVelocity
							,	const double &MaxAcceleration
							,	const double &TargetPosition);


//  ---------------------- Doxygen info ----------------------
//! \fn void ProfileStep2V_To_Vmax(double *CurrentPosition, double *CurrentVelocity, const double &MaxVelocity, const double &MaxAcceleration, double *ElapsedTime,	const bool &SignsAreInverted, TypeIRMLMath::TypeIMotionPolynomials *P)
//!
//! \brief Calculates the polynomial parameters to bring the velocity
//! value to the maximum velocity
//!
//! \param CurrentPosition
//! Current position value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}P_{i} \f$
//!
//! \param CurrentVelocity
//! Current velocity value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}V_{i} \f$
//!
//! \param MaxVelocity
//! Maximum allowed velocity value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}V_{i}^{\,max} \f$
//!
//! \param MaxAcceleration
//! Maximum allowed acceleration value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}A_{i}^{\,max} \f$
//!
//! \param ElapsedTime
//! Elapsed time in seconds, this value may be modified by the method.
//!
//! \param SignsAreInverted
//! Indicates whether the signs of the above values are inverted
//!
//! \param P
//! The set of polynomials that are parameterized by this method
//  ----------------------------------------------------------
void ProfileStep2V_To_Vmax(		double 									*CurrentPosition
							,	double 									*CurrentVelocity
							,	const double							&MaxVelocity
							,	const double							&MaxAcceleration
							,	double									*ElapsedTime					
							,	const bool								&SignsAreInverted
							,	TypeIRMLMath::TypeIMotionPolynomials	*P					);


//  ---------------------- Doxygen info ----------------------
//! \fn void ProfileStep2V_To_Zero(double *CurrentPosition, double *CurrentVelocity, const double &MaxAcceleration, double *ElapsedTime, const bool &SignsAreInverted, TypeIRMLMath::TypeIMotionPolynomials *P)
//!
//! \brief Calculates the polynomial parameters to bring the velocity
//! value to zero
//!
//! \param CurrentPosition
//! Current position value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}P_{i} \f$
//!
//! \param CurrentVelocity
//! Current velocity value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}V_{i} \f$
//!
//! \param MaxAcceleration
//! Maximum allowed acceleration value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}A_{i}^{\,max} \f$
//!
//! \param ElapsedTime
//! Elapsed time in seconds, this value may be modified by the method.
//!
//! \param SignsAreInverted
//! Indicates whether the signs of the above values are inverted
//!
//! \param P
//! The set of polynomials that are parameterized by this method
//  ----------------------------------------------------------
void ProfileStep2V_To_Zero(		double 									*CurrentPosition
							,	double 									*CurrentVelocity
							,	const double							&MaxAcceleration
							,	double									*ElapsedTime					
							,	const bool								&SignsAreInverted
							,	TypeIRMLMath::TypeIMotionPolynomials	*P					);


//  ---------------------- Doxygen info ----------------------
//! \fn void ProfileStep2PosTrap(double *CurrentPosition, double *CurrentVelocity, const double &MaxAcceleration, const double &TargetPosition, const double &SynchronizationTime, double *ElapsedTime, const bool &SignsAreInverted, TypeIRMLMath::TypeIMotionPolynomials *P)
//!
//! \brief Calculates all trajectory parameters the PosTrap
//! velocity profile
//!
//! \param CurrentPosition
//! Current position value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}P_{i} \f$
//!
//! \param CurrentVelocity
//! Current velocity value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}V_{i} \f$
//!
//! \param MaxAcceleration
//! Maximum allowed acceleration value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}A_{i}^{\,max} \f$
//!
//! \param TargetPosition
//! Target position value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}P_{i}^{\,trgt} \f$
//!
//! \param SynchronizationTime
//! Synchronization time \f$\ t_{i}^{\,sync} \f$ 
//!
//! \param ElapsedTime
//! Elapsed time in seconds, this value may be modified by the method.
//!
//! \param SignsAreInverted
//! Indicates whether the signs of the above values are inverted
//!
//! \param P
//! The set of polynomials that are parameterized by this method
//  ----------------------------------------------------------
void ProfileStep2PosTrap(		double 									*CurrentPosition
							,	double									*CurrentVelocity
							,	const double 							&MaxAcceleration
							,	const double 							&TargetPosition
							,	const double 							&SynchronizationTime
							,	double									*ElapsedTime					
							,	const bool								&SignsAreInverted
							,	TypeIRMLMath::TypeIMotionPolynomials	*P						);

//  ---------------------- Doxygen info ----------------------
//! \fn void ProfileStep2NegHldNegLin(double *CurrentPosition, double *CurrentVelocity, const double &MaxAcceleration, const double &TargetPosition, const double &SynchronizationTime, double *ElapsedTime, const bool &SignsAreInverted, TypeIRMLMath::TypeIMotionPolynomials *P)
//!
//! \brief Calculates all trajectory parameters the NegHldNegLin
//! velocity profile
//!
//! \param CurrentPosition
//! Current position value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}P_{i} \f$
//!
//! \param CurrentVelocity
//! Current velocity value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}V_{i} \f$
//!
//! \param MaxAcceleration
//! Maximum allowed acceleration value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}A_{i}^{\,max} \f$
//!
//! \param TargetPosition
//! Target position value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}P_{i}^{\,trgt} \f$
//!
//! \param SynchronizationTime
//! Synchronization time \f$\ t_{i}^{\,sync} \f$ 
//!
//! \param ElapsedTime
//! Elapsed time in seconds, this value may be modified by the method.
//!
//! \param SignsAreInverted
//! Indicates whether the signs of the above values are inverted
//!
//! \param P
//! The set of polynomials that are parameterized by this method
//  ----------------------------------------------------------
void ProfileStep2NegHldNegLin(		double		 							*CurrentPosition
								,	double	 								*CurrentVelocity
								,	const double 							&MaxAcceleration
								,	const double 							&TargetPosition
								,	const double 							&SynchronizationTime
								,	double									*ElapsedTime					
								,	const bool								&SignsAreInverted
								,	TypeIRMLMath::TypeIMotionPolynomials	*P						);


}	// namespace TypeIRMLMath

#endif
