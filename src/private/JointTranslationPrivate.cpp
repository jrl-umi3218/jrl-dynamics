/*
  Copyright (c) 2010, 
  @author Olivier Stasse, Oussama Kanoun
   
  JRL-Japan, CNRS/AIST
 
  All rights reserved.

  Please refers to file License.txt for details on the license.
   
*/
#include "Debug.h"

#include "JointTranslationPrivate.h"
#include "DynamicBodyPrivate.h"


using namespace dynamicsJRLJapan;

JointTranslationPrivate::JointTranslationPrivate()
  :JointPrivate()
{
  m_nbDofs = 1;
  CreateLimitsArray();
}
JointTranslationPrivate::JointTranslationPrivate(const MAL_S4x4_MATRIX(,double) &inInitialPosition)
{
  type(JointPrivate::PRISMATIC_JOINT);
  m_inGlobalFrame = true;
  m_globalPoseAtConstruction = inInitialPosition;

  ODEBUG2("translation: inInitialPosition" << inInitialPosition);
  MAL_S3_VECTOR(laxis, double);

  MAL_S3_VECTOR_ACCESS(laxis,0) = 1.0;
  MAL_S3_VECTOR_ACCESS(laxis,1) = 0.0;
  MAL_S3_VECTOR_ACCESS(laxis,2) = 0.0;

  axis(laxis);
}

JointTranslationPrivate::~JointTranslationPrivate()
{}

bool JointTranslationPrivate::updateTransformation(const vectorN& inDofVector)
{
  vector3d vek,wn3d;
  DynamicBodyPrivate* body = (DynamicBodyPrivate*)(linkedBody());
  DynamicBodyPrivate* parentbody = (DynamicBodyPrivate*)(parentJoint()->linkedBody());
  
  body->q = inDofVector(rankInConfiguration());
  quantity( body->q);

  for (unsigned int i = 0; i<3; i++)
    vek[i] *= body->q;
  
  body->R = parentbody->R;
  
  MAL_S3x3_C_eq_A_by_B(wn3d, body->R, vek);
  
  body->p = parentbody->p+ MAL_S3x3_RET_A_by_B(parentbody->R,body->b) + wn3d;

  return true;
}

bool JointTranslationPrivate::updateVelocity(const vectorN & inRobotConfigVector,
					   const vectorN & inRobotSpeedVector)
{
  return true;
}

bool JointTranslationPrivate::updateAcceleration(const vectorN & inRobotConfigVector,
						 const vectorN & inRobotSpeedVector,
						 const vectorN & inRobotAccelerationVector)
{
  return true;
}
