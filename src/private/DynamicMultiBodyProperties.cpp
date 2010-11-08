/*
 * Copyright 2009, 2010,
 *
 * Oussama Kanoun
 * Francois Keith
 * Florent Lamiraux
 * Olivier Stasse
 *
 * JRL/LAAS, CNRS/AIST
 *
 * This file is part of dynamicsJRLJapan.
 * dynamicsJRLJapan is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * dynamicsJRLJapan is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Lesser Public License for more details.
 * You should have received a copy of the GNU Lesser General Public License
 * along with dynamicsJRLJapan.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  Research carried out within the scope of the Associated
 *  International Laboratory: Joint Japanese-French Robotics
 *  Laboratory (JRL)
 *
 */

/*! System includes */
#include <iostream>
#include <sstream>
#include <fstream>
#include <string.h>

#include "Debug.h"

/*! Local library includes. */
#include "jrl/mal/matrixabstractlayer.hh"
#include "jrl/dynamics/dynamicbody.hh"
#include "DynMultiBodyPrivate.h"
#include "abstract-robot-dynamics/body.hh"

#include "fileReader.h"

using namespace dynamicsJRLJapan;

/*! Implements methods related to propertis for DynamicsMultiBody */
bool DynMultiBodyPrivate::getProperty(const std::string &inProperty,std::string &outValue) const
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
	  std::cout << " Unknown getProperty '" << inProperty << "'" << std::endl;
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
	  std::cout << " Unknown setProperty '" << inProperty << "'" << std::endl;
  }
  return false;
}

bool DynMultiBodyPrivate::isSupported(const std::string &aName) const
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
