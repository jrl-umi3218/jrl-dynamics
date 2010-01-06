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
