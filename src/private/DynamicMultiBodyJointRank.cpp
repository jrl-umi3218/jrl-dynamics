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

/*! Implements the map between the joints' names and their rank */

int DynMultiBodyPrivate::setLinksBetweenJointNamesAndRank(std::vector<NameAndRank_t> &aLinks)
{
  if (m_LinksBetweenJointNamesAndRank.size()!=
      aLinks.size())
    m_LinksBetweenJointNamesAndRank.resize(aLinks.size());

  for(unsigned int i=0;i<m_LinksBetweenJointNamesAndRank.size();i++)
    {
      m_LinksBetweenJointNamesAndRank[i] = aLinks[i];
    }
  return 0;
}

int DynMultiBodyPrivate::getLinksBetweenJointNamesAndRank(std::vector<NameAndRank_t> &aLinks)
{
  if (m_LinksBetweenJointNamesAndRank.size()!=
      aLinks.size())
    aLinks.resize(m_LinksBetweenJointNamesAndRank.size());

  for(unsigned int i=0;i<m_LinksBetweenJointNamesAndRank.size();i++)
    {
      aLinks[i] = m_LinksBetweenJointNamesAndRank[i];
    }
  return 0;
}

void DynMultiBodyPrivate::setJointOrderInConfig(std::vector<CjrlJoint *>inJointVector)
{
  if (m_LinksBetweenJointNamesAndRank.size()!=inJointVector.size())
    m_LinksBetweenJointNamesAndRank.resize(inJointVector.size());

  unsigned int LocalRank = 0;
  for(unsigned int i=0;i<inJointVector.size();i++)
    {

      char Buffer[128];
      memset(Buffer,0,128);
      sprintf(Buffer,"JOINT_%2d",i);
      strcpy(m_LinksBetweenJointNamesAndRank[i].LinkName,Buffer);
      m_LinksBetweenJointNamesAndRank[i].RankInConfiguration=LocalRank;
      LocalRank+= inJointVector[i]->numberDof();
    }

}


int DynMultiBodyPrivate::JointRankFromName(JointPrivate *aJoint)
{

  ODEBUG("m_LinksBetweenJointNamesAndRank.size():" << m_LinksBetweenJointNamesAndRank.size());
  for(unsigned int i=0;i<m_LinksBetweenJointNamesAndRank.size();i++)
    {
      if (!strcmp(m_LinksBetweenJointNamesAndRank[i].LinkName,(char *)aJoint->getName().c_str()))
	return m_LinksBetweenJointNamesAndRank[i].RankInConfiguration;
    }
  return -1;
}

JointPrivate * DynMultiBodyPrivate::JointFromRank(int aRank)
{
  if (m_LinksBetweenJointNamesAndRank.size()!=0)
    {
      string JointName;
      for(unsigned int i=0;i<m_LinksBetweenJointNamesAndRank.size();i++)
        {
	  if (m_LinksBetweenJointNamesAndRank[i].RankInConfiguration==(unsigned int)aRank)
	    JointName = m_LinksBetweenJointNamesAndRank[i].LinkName;
        }
      for(unsigned int i=0;i<m_JointVector.size();i++)
        {
	  if (((JointPrivate *)m_JointVector[i])->getName()==JointName)
	    return (JointPrivate *)m_JointVector[i];
        }
    }

  int CurrentTestRank=0;

  for(unsigned int i=0;i<m_JointVector.size();i++)
    {
      int RankRangeBegin=CurrentTestRank;
      int RankRangeEnd=RankRangeBegin+((JointPrivate *)m_JointVector[i])->numberDof();
      if((aRank>=RankRangeBegin) &&
	 (aRank<RankRangeEnd))
	return (JointPrivate *)m_JointVector[i];
      CurrentTestRank=RankRangeEnd;
    }
  ODEBUG("Looking for rank " << aRank << " failed " << m_LinksBetweenJointNamesAndRank.size());
  return (JointPrivate *)0;
}
