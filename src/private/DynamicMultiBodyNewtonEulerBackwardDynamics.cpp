/*
 * Copyright 2009, 2010,
 *
 * Florent Lamiraux
 * Layale Saab
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

void DynMultiBodyPrivate::BackwardDynamics(DynamicBodyPrivate & CurrentBody )
{
  JointPrivate * currentJoint = CurrentBody.getJointPrivate();

  //currentJoint->updateTorqueAndForce();
  currentJoint->SupdateTorqueAndForce();
  // Update the vector related to the computed quantities.
  for(unsigned int i=0;i<m_StateVectorToJoint.size();i++)
    {
      unsigned int StateRankComputed=false;

      JointPrivate * aJoint = (JointPrivate *)m_JointVector[m_StateVectorToJoint[i]];
      if (aJoint!=0)
        {
	  DynamicBodyPrivate *aDB = (DynamicBodyPrivate *) aJoint->linkedBody();
	  if (aDB!=0)
            {
	      StateRankComputed = true;
	      for(unsigned int k=0;k<3;k++)
                {
		  m_Forces(i,k)=aDB->m_Force[k];
		  m_Torques(i,k) = aDB->m_Torque[k];
                }
            }
        }

      if (!StateRankComputed)
        {
	  for(unsigned int k=0;k<3;k++)
            {
	      m_Forces(i,k)=0.0;
	      m_Torques(i,k)=0.0;
            }
        }
    }

  MAL_VECTOR_RESIZE(m_JointTorques,m_StateVectorToJoint.size());
  MAL_VECTOR_FILL(m_JointTorques,0);
  
  for (unsigned int j=0;j<m_JointVector.size();j++)
  {
	   unsigned int StateRankComputed=false;
	   unsigned int index = m_JointVector[j]->rankInConfiguration();
	   JointPrivate * aJoint = (JointPrivate *)m_JointVector[j]; 
	  if (aJoint!=0)
	  {
		DynamicBodyPrivate *aDB = (DynamicBodyPrivate *) aJoint->linkedBody();
		if (aDB!=0)
		{
		StateRankComputed = true;
		for (unsigned int n=0;n<aJoint->numberDof();n++)
		    m_JointTorques[index+n] = aDB->stau[n];
			
		}
	  }
	  if (!StateRankComputed)
	  {
		  for (unsigned int n=0;n<aJoint->numberDof();n++)
		  m_JointTorques[index+n]=0;
	  }
  }

}