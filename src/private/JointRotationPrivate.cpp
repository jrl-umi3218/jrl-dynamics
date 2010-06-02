/*
  Copyright (c) 2010, 
  @author Olivier Stasse, Oussama Kanoun
   
  JRL-Japan, CNRS/AIST
 
  All rights reserved.

  Please refers to file License.txt for details on the license.
   
*/
#include "Debug.h"

#include "JointRotationPrivate.h"
#include "DynamicBodyPrivate.h"


using namespace dynamicsJRLJapan;

JointRotationPrivate::JointRotationPrivate()
  :JointPrivate()
{
}

JointRotationPrivate::JointRotationPrivate(const MAL_S4x4_MATRIX(,double) &inInitialPosition)
{
  type(JointPrivate::REVOLUTE_JOINT);
  m_inGlobalFrame = true;
  m_globalPoseAtConstruction = inInitialPosition;

  ODEBUG2("rotation: inInitialPosition" << inInitialPosition);

  MAL_S3_VECTOR(laxis, double);

  MAL_S3_VECTOR_ACCESS(laxis,0) = 1.0;
  MAL_S3_VECTOR_ACCESS(laxis,1) = 0.0;
  MAL_S3_VECTOR_ACCESS(laxis,2) = 0.0;

  axis(laxis);
}

JointRotationPrivate::~JointRotationPrivate()
{
}

bool JointRotationPrivate::updateTransformation(const vectorN & inDofVector)
{
  DynamicBodyPrivate* body = (DynamicBodyPrivate*)(linkedBody());
  
  DynamicBodyPrivate* parentbody = 0;
  if ((DynamicBodyPrivate*)(parentJoint())!=0)
      parentbody = (DynamicBodyPrivate*)(parentJoint()->linkedBody());
  
  body->q = inDofVector(rankInConfiguration());
  quantity( body->q);
  matrix3d localR;
  RodriguesRotation(body->a, body->q, localR);
  
  MAL_S3x3_MATRIX(,double) Rtmp;
  if (parentbody!=0)
    {
      MAL_S3x3_C_eq_A_by_B(Rtmp ,parentbody->R , body->R_static);
      body->p = parentbody->p + MAL_S3x3_RET_A_by_B(parentbody->R,body->b);
    }
  else
    {
      Rtmp = body->R_static;
      body->p = body->b;
    }

  MAL_S3x3_C_eq_A_by_B(body->R , Rtmp, localR);
  
  for( unsigned int i=0;i<3;i++)
    for(unsigned int j=0;j<3;j++)
      MAL_S4x4_MATRIX_ACCESS_I_J(body->m_transformation,i,j) = body->R(i,j);
  
  for( unsigned int i=0;i<3;i++)
    MAL_S4x4_MATRIX_ACCESS_I_J(body->m_transformation,i,3) = body->p(i);

  return 0;
}
