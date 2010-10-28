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

 // MAL_VECTOR_RESIZE(m_JointTorques,m_StateVectorToJoint.size());
 // MAL_VECTOR_FILL(m_JointTorques,0);
  for (unsigned int j=0;j<m_JointVector.size();j++)
  {
	  JointPrivate * aJoint = (JointPrivate *)m_JointVector[j];
	  //std::cout << "Joint " <<((JointPrivate *)m_JointVector[j])->getName() << std::endl;
	  DynamicBodyPrivate *aDB = (DynamicBodyPrivate *) aJoint->linkedBody();
	  unsigned int index = m_JointVector[j]->rankInConfiguration();

	  //std::cout << "index = " << index << std::endl;
	  //std::cout << "nbDofs = " << aJoint->numberDof() << std::endl;
	  //std::cout << "torque size = " << aDB->stau.size() << std::endl;
	  for (unsigned int n=0;n<aJoint->numberDof();n++)
	  {
	       //std::cout << "n = " << n << std::endl;
		  // std::cout << "index+n = " << index+n << std::endl;
		   m_JointTorques[index+n] = aDB->stau[n];
		//  std::cout << "JointTorques = " << m_JointTorques << std::endl;
	   }
	//  std::cout << "taun = " << aDB->stau << std::endl;
	//  std::cout << "tau0 = " << aDB->stau[0] << std::endl;
	}
 ////std::cout << "JointTorques = " << m_JointTorques << std::endl;
}