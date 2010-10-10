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
  m_nbDofs=1;
  CreateLimitsArray();
}

JointRotationPrivate::JointRotationPrivate(const MAL_S4x4_MATRIX(,double) &inInitialPosition)
  :JointPrivate()
{
  m_nbDofs=1;
  CreateLimitsArray();

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
  if (parentJoint()!=0)
    parentbody = (DynamicBodyPrivate*)(parentJoint()->linkedBody());
  
  // Read body variables in the state vector
  body->q = inDofVector(rankInConfiguration());
  quantity( body->q);

  // Compute Rodrigues representation of the motion.
  matrix3d localR;
  RodriguesRotation(body->a, body->q, localR);
  
  // Update position and orientation
  matrix3d Rtmp;
  if (parentbody!=0)
    {
      // For orientation of the body.
      MAL_S3x3_C_eq_A_by_B(Rtmp ,parentbody->R , body->R_static);
      // For its position.
      body->p = parentbody->p + MAL_S3x3_RET_A_by_B(parentbody->R,body->b);

      //update the translation/rotation axis of joint
      MAL_S3x3_C_eq_A_by_B(body->w_a,Rtmp, body->a);
  
    }
  else
    {
      // For orientation of the body.
      Rtmp = body->R_static;
      // For its position.
      body->p = body->b;
    }

  // Put it in a homogeneous form.
  MAL_S3x3_C_eq_A_by_B(body->R , Rtmp, localR);
  
  // Store intermediate information for further computation
  body->Riip1 = localR;
  body->Riip1t = MAL_S3x3_RET_TRANSPOSE(localR);

  for( unsigned int i=0;i<3;i++)
    for(unsigned int j=0;j<3;j++)
      MAL_S4x4_MATRIX_ACCESS_I_J(body->m_transformation,i,j) = body->R(i,j);
  
  for( unsigned int i=0;i<3;i++)
    MAL_S4x4_MATRIX_ACCESS_I_J(body->m_transformation,i,3) = body->p(i);

  ODEBUG("a:" << endl << currentBody->a );

  return 0;
}

bool JointRotationPrivate::updateVelocity(const vectorN &inRobotConfigVector,
					  const vectorN &inRobotSpeedVector)
{
//	std::cout << "in Rotation update vel" << std::endl;
  DynamicBodyPrivate* currentBody = (DynamicBodyPrivate*)(linkedBody());
  DynamicBodyPrivate* currentMotherBody = 0;
  vector3d NE_tmp, NE_tmp2; 
  if (parentJoint()!=0)
    currentMotherBody = (DynamicBodyPrivate*)(parentJoint()->linkedBody());

  currentBody->dq = inRobotSpeedVector(rankInConfiguration());
  matrix3d RstaticT = MAL_S3x3_RET_TRANSPOSE(currentBody->R_static);    
  // Computes the angular velocity
  
  // In the global frame.
  ODEBUG("dq: "<< currentBody->dq );
  NE_tmp = currentBody->w_a * currentBody->dq;
  //	  NE_tmp = MAL_S3x3_RET_A_by_B(currentBody->R,NE_tmp);
  if (currentMotherBody!=0)
    currentBody->w  = currentMotherBody->w  + NE_tmp;
  else 
    currentBody->w  = NE_tmp;

  ODEBUG("w: " << currentBody->w );
  // In the local frame.
  NE_tmp = currentBody->a * currentBody->dq;
  
  if (currentMotherBody!=0)
    {
      MAL_S3x3_C_eq_A_by_B(NE_tmp2,RstaticT,currentMotherBody->lw);
    }
  else 
    MAL_S3_VECTOR_FILL(NE_tmp2,0.0);

  NE_tmp2 = MAL_S3x3_RET_A_by_B(currentBody->Riip1t,NE_tmp2);
  
  currentBody->lw  = NE_tmp2  + NE_tmp;
  
  // Computes the linear velocity.
  if (currentMotherBody!=0)
    {
      MAL_S3x3_C_eq_A_by_B(NE_tmp, currentMotherBody->R,
			   currentBody->b);
      MAL_S3_VECTOR_CROSS_PRODUCT(NE_tmp2,
				  currentMotherBody->w , 
				  NE_tmp);
      currentBody->v0 = currentMotherBody->v0 + NE_tmp2;
    }
  else 
    MAL_S3_VECTOR_FILL(currentBody->v0,0.0);
  

  return true;
}

bool JointRotationPrivate::updateAcceleration(const vectorN &inRobotConfigVector,
					      const vectorN &inRobotSpeedVector,
					      const vectorN &inRobotAccelerationVector)
{
 
  DynamicBodyPrivate* currentBody = (DynamicBodyPrivate*)(linkedBody());
  matrix3d RstaticT = MAL_S3x3_RET_TRANSPOSE(currentBody->R_static);    
  DynamicBodyPrivate* currentMotherBody = 0;
  vector3d NE_tmp, NE_tmp2, NE_tmp3, NE_RotByMotherdv;
  if (parentJoint()!=0)
    currentMotherBody = (DynamicBodyPrivate*)(parentJoint()->linkedBody());

  currentBody->ddq = inRobotAccelerationVector(rankInConfiguration());

  // ******************* Computes the angular acceleration for joint i. ********************
  // In global reference frame.
  // NE_tmp2 = z_{i-1} * dqi
  NE_tmp2 = currentBody->w_a * currentBody->dq;
  // NE_tmp3 = w^{(0)}_i x z_{i-1} * dqi
  MAL_S3_VECTOR_CROSS_PRODUCT(NE_tmp3,currentBody->w,NE_tmp2);
  // NE_tmp2 = z_{i-1} * ddqi
  NE_tmp2 = currentBody->w_a * currentBody->ddq;
  currentBody->dw = NE_tmp2 + NE_tmp3;
  if (currentMotherBody!=0)
    currentBody->dw += currentMotherBody->dw;
  
  // In local reference frame.
  if (currentMotherBody!=0)
    {
      MAL_S3x3_C_eq_A_by_B(NE_tmp,RstaticT,currentMotherBody->ldw);
      MAL_S3x3_C_eq_A_by_B(currentBody->ldw,currentBody->Riip1t, NE_tmp);
    }
  else
    MAL_S3_VECTOR_FILL(currentBody->ldw,0.0);
  
  NE_tmp = currentBody->a * currentBody->ddq;
  NE_tmp2 = currentBody->a * currentBody->dq;
  MAL_S3_VECTOR_CROSS_PRODUCT(NE_tmp3,currentBody->lw,NE_tmp2);
  currentBody->ldw= currentBody->ldw + NE_tmp + NE_tmp3;
  ODEBUG(" " <<currentBody->getName() << " currentBody->ldw:" << currentBody->ldw);
  
  // ******************* Computes the linear acceleration for joint i. ********************
  // In global reference frame
  if (currentMotherBody!=0)
    {
      MAL_S3x3_C_eq_A_by_B(NE_tmp, currentMotherBody->R , currentBody->b);
      MAL_S3_VECTOR_CROSS_PRODUCT(NE_tmp2,currentMotherBody->w,NE_tmp);
      MAL_S3_VECTOR_CROSS_PRODUCT(NE_tmp3,currentMotherBody->w,NE_tmp2);
      
      // NE_tmp2 = dw_I x r_{i,i+1}
      MAL_S3_VECTOR_CROSS_PRODUCT(NE_tmp2,currentMotherBody->dw,NE_tmp);
      
      currentBody->dv = NE_tmp2 + NE_tmp3 + currentMotherBody->dv;
    }
  else 
    MAL_S3_VECTOR_FILL(currentBody->dv,0.0);

  // In local reference frame.
  // NE_tmp3 = w_i x (w_i x r_{i,i+1})
  if (currentMotherBody!=0)
    {
      
      MAL_S3_VECTOR_CROSS_PRODUCT(NE_tmp2,currentMotherBody->lw,currentBody->b);
      MAL_S3_VECTOR_CROSS_PRODUCT(NE_tmp3,currentMotherBody->lw,NE_tmp2);
      
      // NE_tmp2 = dw_I x r_{i,i+1}
      MAL_S3_VECTOR_CROSS_PRODUCT(NE_tmp2,currentMotherBody->ldw,currentBody->b);
      
      NE_tmp = NE_tmp2 + NE_tmp3 + currentMotherBody->ldv;
      
      MAL_S3x3_C_eq_A_by_B(NE_RotByMotherdv,RstaticT,NE_tmp);
      currentBody->ldv = MAL_S3x3_RET_A_by_B(currentBody->Riip1t,NE_RotByMotherdv);
      ODEBUG(" " << currentBody->getName() << " Mother->ldv:" <<currentMotherBody->ldv);
    }
  else 
    MAL_S3_VECTOR_FILL(currentBody->ldv,0);

  return true;
}

const matrixNxP & JointRotationPrivate::pcalc(const vectorN & qi)
{
	MAL_MATRIX_RESIZE(m_phi,6,1);
	m_phi(2,0)=1;
    return m_phi;

}

const matrixNxP & JointRotationPrivate::pdcalc(const vectorN & qi)
{
	MAL_MATRIX_RESIZE(m_dotphi,6,1);
	MAL_MATRIX_FILL(m_dotphi,0);

  return m_dotphi;
}