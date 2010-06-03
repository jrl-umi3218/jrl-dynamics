/*
  Copyright (c) 2010, 
  @author Olivier Stasse, Oussama Kanoun
   
  JRL-Japan, CNRS/AIST
 
  All rights reserved.

  Please refers to file License.txt for details on the license.
   
*/
#include "Debug.h"

#include "JointAnchorPrivate.h"
#include "DynamicBodyPrivate.h"


using namespace dynamicsJRLJapan;

JointAnchorPrivate::JointAnchorPrivate(const MAL_S4x4_MATRIX(,double) &inInitialPosition)
{
  type(JointPrivate::FIX_JOINT);
  m_inGlobalFrame = true;
  m_globalPoseAtConstruction = inInitialPosition;

  ODEBUG2("anchor: inInitialPosition" << inInitialPosition);
  MAL_S3_VECTOR(laxis, double);

  MAL_S3_VECTOR_ACCESS(laxis,0) = 1.0;
  MAL_S3_VECTOR_ACCESS(laxis,1) = 0.0;
  MAL_S3_VECTOR_ACCESS(laxis,2) = 0.0;

  axis(laxis);
}

JointAnchorPrivate::~JointAnchorPrivate()
{
}

bool JointAnchorPrivate::updateTransformation(const vectorN & inDofVector)
{
  return true;
}

bool JointAnchorPrivate::updateVelocity(const vectorN &inRobotConfigVector,
					const vectorN &inRobotConfigSpeed)
{
  return true;
}

bool JointAnchorPrivate::updateAcceleration(const vectorN &inRobotConfigVector,
					    const vectorN &inRobotSpeedVector,
					    const vectorN &inRobotAccelerationVector)
{
  return true;
}
