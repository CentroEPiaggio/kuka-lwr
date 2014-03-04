//  ---------------------- Doxygen info ----------------------
//! \file TypeIRMLOutputParameters.h
//!
//! \brief
//! Header file for the output parameters of the Reflexxes Motion Library
//! (Type I)
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


#ifndef __TypeIRMLOutputParameters__
#define __TypeIRMLOutputParameters__


#include <TypeIRMLVector.h>


namespace TypeIRMLMath
{


//  ---------------------- Doxygen info ----------------------
//! \class TypeIRMLOutputParameters
//!
//! \brief
//! This class describes the output parameters of the Reflexxes Motion Library
//! (Type I)
//! 
//! \details
//! It is part of the namespace TypeIRMLMath.
//! 
//! \sa TypeIRML
//! \sa TypeIRMLInputParameters
//  ----------------------------------------------------------
class TypeIRMLOutputParameters
{

public:


//  ---------------------- Doxygen info ----------------------
//! \fn TypeIRMLOutputParameters(const unsigned int &NumberOfDOFs)
//!
//! \brief
//! Constructor of the class TypeIRMLOutputParameters
//!
//! \details
//! Allocates memory for the specified number of degrees of freedom
//!
//! \warning
//! A call of this method is \em not real-time capable!!!
//!
//! \param NumberOfDOFs
//! Number of degrees of freedom
//  ----------------------------------------------------------

	TypeIRMLOutputParameters(const unsigned int &NumberOfDOFs)
	{
		this->NewPosition			=	new TypeIRMLDoubleVector(NumberOfDOFs);
		this->NewVelocity			=	new TypeIRMLDoubleVector(NumberOfDOFs);

		this->NewPosition->Set(0.0);
		this->NewVelocity->Set(0.0);
	}


//  ---------------------- Doxygen info ----------------------
//! \fn TypeIRMLOutputParameters(const TypeIRMLOutputParameters & OP)
//!
//! \brief
//! Copy constructor of the class TypeIRMLOutputParameters
//!
//! \details
//! Allocates memory for the specified number of degrees of freedom.
//!
//! \warning
//! \brief A call of this method is \em not real-time capable!!!
//!
//! \param OP
//! Original object reference
//  ----------------------------------------------------------
	TypeIRMLOutputParameters(const TypeIRMLOutputParameters & OP)
	{
		this->NewPosition	=	new TypeIRMLDoubleVector(OP.NewPosition->VectorDimension);
		this->NewVelocity	=	new TypeIRMLDoubleVector(OP.NewVelocity->VectorDimension);

		this->NewPosition	=	OP.NewPosition;
		this->NewVelocity	=	OP.NewVelocity;
	}


//  ---------------------- Doxygen info ----------------------
//! \fn TypeIRMLOutputParameters &operator = (const TypeIRMLOutputParameters &OP)
//!
//! \brief
//! Copy operator
//!
//! \param OP
//! Set of output parameters
//  ----------------------------------------------------------
	TypeIRMLOutputParameters &operator = (const TypeIRMLOutputParameters &OP)
	{
		this->NewPosition	=	OP.NewPosition;
		this->NewVelocity	=	OP.NewVelocity;

		return (*this);
	}


//  ---------------------- Doxygen info ----------------------
//! \fn ~TypeIRMLOutputParameters(void)
//!
//! \brief
//! Destructor of class TypeIRMLOutputParameters, frees heap memory
//  ----------------------------------------------------------
	~TypeIRMLOutputParameters(void)
	{
		delete this->NewPosition	;
		delete this->NewVelocity	;

		this->NewPosition	=	NULL;
		this->NewVelocity	=	NULL;
	}

	TypeIRMLDoubleVector*	NewPosition;
	TypeIRMLDoubleVector*	NewVelocity;
};



}	// namespace

#endif
