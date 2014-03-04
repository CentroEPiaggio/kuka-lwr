//  ---------------------- Doxygen info ----------------------
//! \file TypeIRMLMath.h
//!
//! \brief
//! Header file for functions and definitions of constant values and macros
//! 
//! \details
//! Header file for definitions of constant values and macros to be used
//! for within in the library of the Type I On-Line Trajectory Algorithm.
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


#ifndef __TypeIRMLMath__
#define __TypeIRMLMath__


//*******************************************************************************************
// Include files



namespace TypeIRMLMath
{

//*******************************************************************************************
// Definitions and macros




//  ---------------------- Doxygen info ----------------------
//! \def RML_INFINITY
//!
//! \brief
//! A value for infinity \f$ \infty = 10^{100} \f$
//  ----------------------------------------------------------
#define		RML_INFINITY				((double)1.0e100)


//  ---------------------- Doxygen info ----------------------
//! \def RML_DENOMINATOR_EPSILON
//!
//! \brief
//! A threshold value for zero to be used for denominators
//  ----------------------------------------------------------
#define		RML_DENOMINATOR_EPSILON		((double)1.0e-12)


//  ---------------------- Doxygen info ----------------------
//! \def RML_MIN_VALUE_FOR_MAXVELOCITY
//!
//! \brief
//! Positive threshold value to determine the minimum allowed value for
//! the maximum velocity value
//  ----------------------------------------------------------
#define		RML_MIN_VALUE_FOR_MAXVELOCITY		((double)1.0e-4)


//  ---------------------- Doxygen info ----------------------
//! \def RML_MIN_VALUE_FOR_MAXACCELERATION
//!
//! \brief
//! Positive threshold value to determine the minimum allowed value
//! for the maximum acceleration value
//  ----------------------------------------------------------
#define		RML_MIN_VALUE_FOR_MAXACCELERATION	((double)1.0e-4)


//  ---------------------- Doxygen info ----------------------
//! \def pow2(A)
//!
//! \brief
//! A to the power of 2 
//!
//! \param A
//! Basis
//!
//! \return
//! Result value
//  ----------------------------------------------------------
#define pow2(A)							((A)*(A))


//  ---------------------- Doxygen info ----------------------
//! \fn double RMLSqrt(const double &Value)
//!
//! \brief Calculates the real square root of a given value
//!
//! \details
//! If the value is negative a value of zero will be returned.
//!
//! \param Value
//! Square root radicand
//!
//! \return
//! Square root value (real)
//  ----------------------------------------------------------
double RMLSqrt(const double &Value);


}	// namespace TypeIRMLMath

#endif
