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
      matrix3d ltmp1;
      MAL_S3x3_C_eq_A_by_B(ltmp1,
			   parentbody->R, 
			   body->R_static);
      MAL_S3x3_C_eq_A_by_B(body->w_a,ltmp1, body->a);
  
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

void JointRotationPrivate::updateTorqueAndForce()
{
  DynamicBodyPrivate * CurrentBody = (DynamicBodyPrivate*)(linkedBody());

  MAL_S3x3_MATRIX(,double) aRt;

  MAL_S3x3_MATRIX(,double) currentBodyRt;
  currentBodyRt = MAL_S3x3_RET_TRANSPOSE(CurrentBody->R);
  
  /*//MAL_S3x3_MATRIX(,double) currentBodyinitialRt;
  matrix4d initialTransform  = CurrentBody->joint()->initialPosition();
  for(unsigned int li=0;li<3;li++)
    for(unsigned int lj=0;lj<3;lj++)
      currentBodyRt = MAL_S4x4_MATRIX_ACCESS_I_J(initialTransform,lj,li);

  // currentBodyRt = MAL_S3x3_RET_A_by_B(currentBodyRt,currentBodyinitialRt);
  */
  MAL_S3_VECTOR(,double) lg;
  lg(0) = 0.0;
  lg(1) = 0.0;
  lg(2) = -9.81;

  /* lg should be expressed in frame i */
  lg = MAL_S3x3_RET_A_by_B(currentBodyRt,lg);
  ODEBUG(" lg :" << lg);
  /* Compute the torque
   * with eq. (7.147) Spong RMC p. 277
   *
   *
   */
  MAL_S3_VECTOR(,double) firstterm,
    sndterm, thirdterm, fifthterm,tmp;
  // Do not fourth term because it is the angular acceleration.

  /* Force - Constant part: 2nd and 3rd term of eq.(7.146) 
     m_i a_{c,i} - m_i g_i
   */
  ODEBUG3(" Body name: " << CurrentBody->getName() << " : " << lg << " mass: " << CurrentBody->mass());
  tmp = CurrentBody->ldv_c - lg;
  ODEBUG3(" Acceleration: " << CurrentBody->ldv_c);
  CurrentBody->m_Force =  tmp * CurrentBody->mass();
  /* Get the local center of mass */
  vector3d lc = CurrentBody->localCenterOfMass();

  /* Torque - 5th term : w_i x (I_i w_i)*/  
  MAL_S3x3_MATRIX(,double) lI = CurrentBody->getInertie();
  tmp = MAL_S3x3_RET_A_by_B(lI,CurrentBody->lw);
  //  tmp = MAL_S3x3_RET_A_by_B(lI,CurrentBody->w);


  MAL_S3_VECTOR_CROSS_PRODUCT(fifthterm,CurrentBody->lw,tmp);

  /* Torque - 4th term and 5th term 
  Torque_i = I_i * alpha_i +  w_i x I_i w_i) */
  MAL_S3x3_C_eq_A_by_B(tmp,lI,CurrentBody->ldw);
  CurrentBody->m_Torque =  tmp + fifthterm ;

  //CurrentBody->m_Torque = CurrentBody->ldw + fifthterm;
  //std::cout << "alpha_i: " << CurrentBody->ldw << std::endl;
  /* Compute with the force
   * eq. (7.146) Spong RMC p. 277
   * fi = R^i_{i+1} * f_{i+1} + m_i * a_{c,i} - m_i * g_i
   * g_i is the gravity express in the i reference frame.
   */

  matrix4d curtri;
  curtri = currentTransformation();

  matrix4d invcurtri;
  MAL_S4x4_INVERSE(curtri,invcurtri,double);      

  for(unsigned int IndexChild = 0;
      IndexChild< m_Children.size();IndexChild++)
    {
      JointPrivate * ChildJoint = m_Children[IndexChild];
      if (ChildJoint!=0)
	{
	  DynamicBodyPrivate *ChildBody = ChildJoint->linkedDBody();
	  
	  //cout << "Child Bodies : " << Child->getName() << endl;
	  aRt = MAL_S3x3_RET_A_by_B(ChildBody->R_static,ChildBody->Riip1);
	  
	  // /* Force computation. */
	  // R_i_{i+1} f_{i+1}
	  tmp= MAL_S3x3_RET_A_by_B(aRt, ChildBody->m_Force);
	  CurrentBody->m_Force += tmp;
	  
	  /* Torque computation. */
	  /* 1st term : R^i_{i+1} t_{i+1} */
	  firstterm = MAL_S3x3_RET_A_by_B(aRt, ChildBody->m_Torque);
	  
	  /* 3rd term : (R_i_{i+1} f_{i+1}) x rip1,ci */
	  matrix4d curtrip1= ChildBody->joint()->currentTransformation();
	  matrix4d ip1Mi;
	  MAL_S4x4_C_eq_A_by_B(ip1Mi, invcurtri, curtrip1);
	  
	  /* rip1,ci = (riip1)^(-1)rici 
	     Note: rip1,c1 is expressed in the frame of link i.
	  */
	  vector3d res3d; 
	  
	  MAL_S3_VECTOR_ACCESS(res3d,0) = 
	    MAL_S3_VECTOR_ACCESS(lc,0)-
	    MAL_S4x4_MATRIX_ACCESS_I_J(ip1Mi,0,3); 
	  MAL_S3_VECTOR_ACCESS(res3d,1) = 
	    MAL_S3_VECTOR_ACCESS(lc,1) - 
	    MAL_S4x4_MATRIX_ACCESS_I_J(ip1Mi,1,3) ;
	  MAL_S3_VECTOR_ACCESS(res3d,2) = 
	    MAL_S3_VECTOR_ACCESS(lc,2)-
	    MAL_S4x4_MATRIX_ACCESS_I_J(ip1Mi,2,3);
	  MAL_S3_VECTOR_CROSS_PRODUCT(thirdterm,tmp, res3d);
	  
	  CurrentBody->m_Torque += firstterm + thirdterm;
	}
      else 
	{
	  cout << "Strange " << getName() << " has a NIL child at " << IndexChild << endl;
	}
    }
  ODEBUG3(" " << getName() << ":" << CurrentBody->m_Force);
  /* 2nd term : -f_i x r_{i,ci} */
  MAL_S3_VECTOR_CROSS_PRODUCT(sndterm,CurrentBody->m_Force, lc);
  CurrentBody->m_Torque = CurrentBody->m_Torque - sndterm;
}
