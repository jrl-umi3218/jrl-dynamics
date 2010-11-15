/* @doc Computation of the torques and forces.
   Copyright (c) 2009-2010
   @author Olivier Stasse,
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