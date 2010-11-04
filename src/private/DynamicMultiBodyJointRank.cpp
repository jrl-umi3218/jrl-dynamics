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
