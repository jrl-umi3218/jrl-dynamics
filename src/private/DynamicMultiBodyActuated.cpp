/*
 * Copyright 2009, 2010, 
 *
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
/* Implements Actuated methods of DynamicsMultiBody */
void DynMultiBodyPrivate::setActuatedJoints(std::vector<CjrlJoint *>& lActuatedJoints)
{
  bool Same=true;

  if (m_ActuatedJoints.size()==
      lActuatedJoints.size())
    {
      for(unsigned int i=0;i<m_ActuatedJoints.size();i++)
	if (m_ActuatedJoints[i] != lActuatedJoints[i])
	  Same=false;
      
    }

  if (Same)
    return;

  m_ActuatedJoints = lActuatedJoints;

  m_SynchronizationBetweenActuatedVectorAndJoints = false;
}

const std::vector<CjrlJoint *>& DynMultiBodyPrivate::getActuatedJoints() const
{
  return m_ActuatedJoints;
}


void DynMultiBodyPrivate::GetJointIDInConfigurationFromActuatedID(std::vector<int> &VectorFromActuatedIDToConfigurationID)
{
  VectorFromActuatedIDToConfigurationID.clear();
  VectorFromActuatedIDToConfigurationID = m_ActuatedIDToConfiguration;
}

int DynMultiBodyPrivate::BuildLinkBetweenActuatedVectorAndJoints()
{
  ODEBUG(" Wenth through here.");
  if (m_ActuatedIDToConfiguration.size()==0)
    return BuildLinkFromActuatedJoints();
 
  return BuildLinkFromActuatedIDs();
}

int DynMultiBodyPrivate::BuildLinkFromActuatedJoints()
{
  ODEBUG(" Went through here.");
  m_ActuatedIDToConfiguration.resize(m_ActuatedJoints.size());
  
  for(unsigned int IndexInActuatedVector=0;
      IndexInActuatedVector<m_ActuatedJoints.size();
      IndexInActuatedVector++)
    {
      bool FoundActuatedJoint = false;
      unsigned int IndexInJointVector=0;
      for(IndexInJointVector=0;IndexInJointVector<m_JointVector.size();IndexInJointVector++)
	if (m_JointVector[IndexInJointVector]==m_ActuatedJoints[IndexInActuatedVector])
	  {
	    FoundActuatedJoint = true;
	    break;
	  }
      if (FoundActuatedJoint)
	{
	  ((JointPrivate *)(m_JointVector[IndexInJointVector]))->setIDinActuated(IndexInActuatedVector);
	  m_ActuatedIDToConfiguration[IndexInActuatedVector] = IndexInJointVector;
	}
      else
	{
	  cerr << "Unable to find actuated joint: "<< m_ActuatedJoints[IndexInActuatedVector] << 
	    " in the vector of joints." << endl;
	  return -1;
	}
    }
  m_SynchronizationBetweenActuatedVectorAndJoints = true;
  return 0;
}

int DynMultiBodyPrivate::BuildLinkFromActuatedIDs()
{
  ODEBUG("Went through here.");
  m_ActuatedJoints.resize(m_ActuatedIDToConfiguration.size());
  
  for(unsigned int IndexInActuatedIDs=0;
      IndexInActuatedIDs<m_ActuatedIDToConfiguration.size();
      IndexInActuatedIDs++)
    {
      bool FoundActuatedJoint = false;
      unsigned int IndexInJointVector=0;
      for(IndexInJointVector=0;IndexInJointVector<m_JointVector.size();IndexInJointVector++)
	if (m_JointVector[IndexInJointVector]->rankInConfiguration()== (unsigned int)
	    m_ActuatedIDToConfiguration[IndexInActuatedIDs])
	  {
	    FoundActuatedJoint = true;
	    break;
	  }
      if (FoundActuatedJoint)
	{
	  m_ActuatedJoints[IndexInActuatedIDs] = m_JointVector[IndexInJointVector];
	  ODEBUG(" " << ((JointPrivate *)m_JointVector[IndexInJointVector])->getName() << " actuated:" 
		  << IndexInActuatedIDs);
	}
      else
	{
	  cerr << "Unable to find actuated joint: "<< m_ActuatedJoints[IndexInJointVector] << 
	    " in the vector of joints." << endl;
	  return -1;
	}
    }
  m_SynchronizationBetweenActuatedVectorAndJoints = true;
  return 0;
}


CjrlJoint* DynMultiBodyPrivate::GetJointFromActuatedID(int JointID)
{

  for(unsigned int i=0;i<m_JointVector.size();i++)
    {
      JointPrivate * r;
      if (((r=(JointPrivate *)m_JointVector[i])->getIDinActuated())==JointID)
        {
	  ODEBUG("JointPrivate : "<< r->getName() << " " << JointID );

	  return r;
        }
    }
  return 0;
}
