/* @doc Computation of the dynamic aspect for a robot.
   This class will load the description of a robot from a VRML file
   following the OpenHRP syntax. Using ForwardVelocity it is then
   possible specifying the angular velocity and the angular value 
   to get the absolute position, and absolute velocity of each 
   body separetly. Heavy rewriting from the original source
   of Adrien and Jean-Remy. 
 
   This implantation is an updated based on a mixture between 
   the code provided by Jean-Remy and Adrien.
 
   Copyright (c) 2005-2006, 
   @author Olivier Stasse, Ramzi Sellouati, Jean-Remy Chardonnet, Adrien Escande, Abderrahmane Kheddar
   Copyright (c) 2007-2009
   @author Olivier Stasse, Oussama Kannoun, Fumio Kanehiro.
   JRL-Japan, CNRS/AIST
 
   All rights reserved.

   Please refers to file License.txt for details on the license.

*/

/*! System includes */
#include <iostream>
#include <sstream>
#include <fstream>
#include <string.h>

#include "Debug.h"

/*! Local library includes. */
#include "MatrixAbstractLayer/MatrixAbstractLayer.h"
#include "dynamicsJRLJapan/DynamicBody.h"
#include "DynMultiBodyPrivate.h"
#include "robotDynamics/jrlBody.h"

#include "fileReader.h"

using namespace dynamicsJRLJapan;

/*! Implements methods related to propertis for DynamicsMultiBody */
bool DynMultiBodyPrivate::getProperty(const std::string &inProperty,std::string &outValue)
{
  if (inProperty=="ComputeVelocity")
    {
      if (m_ComputeVelocity)
	outValue="true";
      else
	outValue="false";
      return true;
    }
  else if (inProperty=="ComputeAcceleration")
    {
      if (m_ComputeAcceleration)
	outValue="true";
      else
	outValue="false";
      return true;

    }
    else if (inProperty=="ComputeSkewCom")
    {
        if (m_ComputeSkewCoM)
            outValue="true";
        else
            outValue="false";
        return true;

    }
  else if (inProperty=="ComputeCoM")
    {
      if (m_ComputeCoM)
	outValue="true";
      else
	outValue="false";
      return true;

    }
  else if (inProperty=="ComputeMomentum")
    {
      if (m_ComputeMomentum)
	outValue="true";
      else
	outValue="false";
      return true;
    }
  else if (inProperty=="ComputeAccelerationCoM")
    {
      if (m_ComputeAccCoM)
	outValue="true";
      else
	outValue="false";
      return true;

    }
  else if (inProperty=="ComputeBackwardDynamics")
    {
      if (m_ComputeBackwardDynamics)
	outValue="true";
      else
	outValue="false";
      return true;

    }
  else if (inProperty=="ComputeZMP")
    {
      if (m_ComputeZMP)
	outValue="true";
      else
	outValue="false";
      return true;

    }
  else if (inProperty=="TimeStep")
    {
      ostringstream aos;
      aos << m_TimeStep;
      outValue=aos.str();
    }
  else 
  {
	  std::cout << " Unknown property '" << inProperty << "'" << std::endl;
  }
  outValue="false";
  return false;
}

bool DynMultiBodyPrivate::setProperty(std::string &inProperty,const std::string &inValue)
{
  if (inProperty=="ComputeVelocity")
    {
      if (inValue=="true")
        {
	  setComputeVelocity(true);
	  return true;
        }
      else if (inValue=="false")
        {
	  setComputeVelocity(false);
	  return true;
        }
    }
  else if (inProperty=="ComputeAcceleration")
    {
      if (inValue=="true")
        {
	  setComputeAcceleration(true);
	  return true;
        }
      else if (inValue=="false")
        {
	  setComputeAcceleration(false);
	  return true;
        }
    }
    else if (inProperty=="ComputeSkewCom")
    {
        if (inValue=="true")
        {
            setComputeSkewCoM(true);
            return true;
        }
        else if (inValue=="false")
        {
            setComputeSkewCoM(false);
            return true;
        }
    }
  else if (inProperty=="ComputeMomentum")
    {
      if (inValue=="true")
        {
	  setComputeMomentum(true);
	  return true;
        }
      else if (inValue=="false")
        {
	  setComputeMomentum(false);
	  return true;
        }
    }
  else if (inProperty=="ComputeCoM")
    {
      if (inValue=="true")
        {
	  setComputeCoM(true);
	  return true;
        }
      else if (inValue=="false")
        {
	  setComputeCoM(false);
	  return true;
        }
    }
  else if (inProperty=="ComputeAccelerationCoM")
    {
      if (inValue=="true")
        {
	  setComputeAccelerationCoM(true);
	  return true;
        }
      else if (inValue=="false")
        {
	  setComputeAccelerationCoM(false);
	  return true;
        }
    }
  else if (inProperty=="ComputeBackwardDynamics")
    {
      if (inValue=="true")
        {
	  setComputeBackwardDynamics(true);
	  return true;
        }
      else if (inValue=="false")
        {
	  setComputeBackwardDynamics(false);
	  return true;
        }
    }
  else if (inProperty=="ComputeZMP")
    {
      if (inValue=="true")
        {
	  setComputeZMP(true);
	  return true;
        }
      else if (inValue=="false")
        {
	  setComputeZMP(false);
	  return true;
        }

    }
  else if (inProperty=="FileJointRank")
    {
      m_FileLinkJointRank = inValue;
    }
  else if (inProperty=="TimeStep")
    {
      istringstream iss(inValue);
      iss >> m_TimeStep;
    }
  else if (inProperty=="ResetIteration")
  {
	  ResetIterationNumber();
  }
  else 
  {
	  std::cout << " Unknown property '" << inProperty << "'" << std::endl;
  }
  return false;
}

bool DynMultiBodyPrivate::isSupported(const std::string &aName)
{
  if (aName=="ComputeVelocity")
    return true;
  else if (aName=="ComputeAcceleration")
    return true;
  else if (aName=="ComputeCoM")
    return true;
  else if (aName=="ComputeAccCoM")
    return true;
  else if (aName=="ComputeBackwardDynamics")
    return true;
  else if (aName=="ComputeZMP")
    return true;
  else if (aName=="ComputeSkewCom")
      return true;
  else if (aName=="TimeStep")
    return true;
  return false;
}
