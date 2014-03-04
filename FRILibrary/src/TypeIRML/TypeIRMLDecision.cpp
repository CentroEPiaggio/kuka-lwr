//  ---------------------- Doxygen info ----------------------
//! \file TypeIRMLDecision.cpp
//!
//! \brief
//! Implementation file for decisions of the two decision trees of the 
//! Type I On-Line Trajectory Generation algorithm
//!
//! \sa TypeIRMLDecision.h
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



#include <TypeIRMLDecision.h>
#include <TypeIRMLMath.h>


//************************************************************************************
// Decision_1001()

bool TypeIRMLMath::Decision_1001(const double &CurrentVelocity)
{
	return(CurrentVelocity >= 0.0);
}


//************************************************************************************
// Decision_1002()

bool TypeIRMLMath::Decision_1002(		const double &CurrentVelocity
									,	const double &MaxVelocity)
{
	return(CurrentVelocity <= MaxVelocity);
}


//************************************************************************************
// Decision_1003()

bool TypeIRMLMath::Decision_1003(		const double &CurrentPosition
									,	const double &CurrentVelocity
									,	const double &MaxAcceleration
									,	const double &TargetPosition)
{
	return((CurrentPosition + 0.5 * pow2(CurrentVelocity) / MaxAcceleration) <= TargetPosition);
}


//************************************************************************************
// Decision_1004()

bool TypeIRMLMath::Decision_1004(		const double &CurrentPosition
									,	const double &CurrentVelocity
									,	const double &MaxVelocity
									,	const double &MaxAcceleration
									,	const double &TargetPosition)
{
	return( (CurrentPosition + (2.0 * pow2(MaxVelocity) - pow2(CurrentVelocity))
				/ (2.0 * MaxAcceleration) ) <= TargetPosition );
}


//************************************************************************************
// Decision_2001()

bool TypeIRMLMath::Decision_2001(const double &CurrentVelocity)
{
	return(TypeIRMLMath::Decision_1001(CurrentVelocity));
}


//************************************************************************************
// Decision_2002()

bool TypeIRMLMath::Decision_2002(		const double &CurrentVelocity
									,	const double &MaxVelocity)
{
	return(TypeIRMLMath::Decision_1002(		CurrentVelocity
										,	MaxVelocity));
}


//************************************************************************************
// Decision_2003()

bool TypeIRMLMath::Decision_2003(		const double &CurrentPosition
									,	const double &CurrentVelocity
									,	const double &MaxAcceleration
									,	const double &TargetPosition)
{
	return(TypeIRMLMath::Decision_1003(		CurrentPosition
										,	CurrentVelocity
										,	MaxAcceleration
										,	TargetPosition	));
}


//************************************************************************************
// Decision_2004()

bool TypeIRMLMath::Decision_2004(		const double &CurrentPosition
									,	const double &CurrentVelocity
									,	const double &MaxAcceleration
									,	const double &TargetPosition
									,	const double &SynchronizationTime)
{
	return((CurrentPosition + SynchronizationTime * CurrentVelocity
			- 0.5 * pow2(CurrentVelocity) / MaxAcceleration) <= TargetPosition);
}