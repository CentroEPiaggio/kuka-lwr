//  ---------------------- Doxygen info ----------------------
//! \file TypeIRMLPolynomial.cpp
//!
//! \brief
//! Implementation file for polynomials designed for the Type I On-Line Trajectory
//! Generation algorithm
//!
//! \sa TypeIRMLPolynomial.h
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



#include <TypeIRMLPolynomial.h>
#include <TypeIRMLMath.h>


//************************************************************************************
// Constructor


TypeIRMLMath::TypeIRMLPolynomial::TypeIRMLPolynomial()
{
	a0		= 0.0;
	a1		= 0.0;
	a2		= 0.0;
	DeltaT	= 0.0;
	Degree	= 0;
}


//************************************************************************************
// Destructor

TypeIRMLMath::TypeIRMLPolynomial::~TypeIRMLPolynomial()
{}


//************************************************************************************
// SetCoefficients()
// f(x) = a_2 * (t - DeltaT)^2 + a_1 * (t - DeltaT) + a_0

void TypeIRMLMath::TypeIRMLPolynomial::SetCoefficients(		const double	&Coeff2
														,	const double	&Coeff1
														,	const double	&Coeff0
														,	const double	&Diff)
{
	a0		= Coeff0;
	a1		= Coeff1;
	a2		= Coeff2;
	DeltaT	= Diff;

	if (a2 != 0.0)
	{
		Degree = 2;
		return;
	}

	if (a1 != 0.0)
	{
		Degree = 1;
		return;
	}

	Degree = 0;
	return;
}


//*******************************************************************************************
// CalculateValue()
// calculates f(t)

double TypeIRMLMath::TypeIRMLPolynomial::CalculateValue(const double &t) const
{
	return(	((Degree == 2)?
			(a2 * (t - DeltaT) * (t - DeltaT) + a1 * (t - DeltaT) + a0):
			((Degree == 1)?
			(a1 * (t - DeltaT) + a0):
			(a0))));
}
