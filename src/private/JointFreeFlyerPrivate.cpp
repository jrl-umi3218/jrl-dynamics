/*
  Copyright (c) 2010, 
  @author Olivier Stasse, Oussama Kanoun
   
  JRL-Japan, CNRS/AIST
 
  All rights reserved.

  Please refers to file License.txt for details on the license.
   
*/
#include "Debug.h"

#include "JointPrivate.h"
#include "JointFreeFlyerPrivate.h"
#include "DynamicBodyPrivate.h"


using namespace dynamicsJRLJapan;

JointFreeflyerPrivate::JointFreeflyerPrivate()
 :JointPrivate()
{
  m_dof6D.resize(6,false);
  m_nbDofs = 6;
  CreateLimitsArray();
}

JointFreeflyerPrivate::JointFreeflyerPrivate(const JointFreeflyerPrivate &a)
 :JointPrivate(a)
{
  m_dof6D.resize(6,false);
}

JointFreeflyerPrivate::JointFreeflyerPrivate(JointFreeflyerPrivate &a)
 :JointPrivate(a)
{
  m_dof6D.resize(6,false);
}

JointFreeflyerPrivate::JointFreeflyerPrivate(const MAL_S4x4_MATRIX(,double) &inInitialPosition)
{
  type(JointPrivate::FREE_JOINT);
  m_inGlobalFrame = true;
  m_globalPoseAtConstruction = inInitialPosition;

  ODEBUG2("freeflyer: inInitialPosition" << inInitialPosition);
  MAL_S3_VECTOR(laxis, double);

  MAL_S3_VECTOR_ACCESS(laxis,0) = 1.0;
  MAL_S3_VECTOR_ACCESS(laxis,1) = 0.0;
  MAL_S3_VECTOR_ACCESS(laxis,2) = 0.0;

  axis(laxis);
  m_dof6D.resize(6,false);
   
}

JointFreeflyerPrivate::~JointFreeflyerPrivate()
{
}

bool JointFreeflyerPrivate::updateTransformation(const vectorN & inDofVector)
{
  
  for (unsigned int i=0; i<6; i++)
    m_dof6D(i) = inDofVector(rankInConfiguration() + i);
  
  UpdatePoseFrom6DOFsVector(m_dof6D);
  return true;
}

bool JointFreeflyerPrivate::updateVelocity(const vectorN & inRobotConfigVector,
					   const vectorN & inRobotSpeedVector)
{
  DynamicBodyPrivate* currentBody = (DynamicBodyPrivate*)(linkedBody());
  
  for(unsigned int i=0;i<3;i++)
    {
      currentBody->v0[i] = inRobotSpeedVector(rankInConfiguration()+i);

      currentBody->w[i] = 
	currentBody->lw[i] = inRobotSpeedVector(rankInConfiguration()+i+3);
            
    }
  return true;
}

bool JointFreeflyerPrivate::updateAcceleration(const vectorN & inRobotConfigVector,
					       const vectorN & inRobotSpeedVector,
					       const vectorN & inRobotAccelerationVector)
{
  DynamicBodyPrivate* currentBody = (DynamicBodyPrivate*)(linkedBody());
  
  for(unsigned int i=0;i<3;i++)
    {
      currentBody->dv[i] = 
	currentBody->ldv[i] = inRobotAccelerationVector(rankInConfiguration()+i);
      currentBody->dw[i] = 
	currentBody->ldw[i] = inRobotAccelerationVector(rankInConfiguration()+i+3);
    }
  return true;
}

