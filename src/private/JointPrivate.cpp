/*
  Copyright (c) 2005-2006, 
  @author Olivier Stasse, Oussama Kanoun, Fumio Kanehiro, Florent Lamiraux
   
  JRL-Japan, CNRS/AIST
 
  All rights reserved.

  Please refers to file License.txt for details on the license.
   
*/
#include "Debug.h"

#include "JointPrivate.h"
#include "DynamicBodyPrivate.h"

using namespace dynamicsJRLJapan;


JointPrivate::JointPrivate(int ltype, MAL_S3_VECTOR(,double) & laxis,
             float lquantite, MAL_S4x4_MATRIX(,double) & lpose):
  m_inGlobalFrame(false),
  m_nbDofs(0),
  m_type(ltype),
  m_axis(laxis),
  m_quantity(lquantite),
  m_poseInParentFrame(lpose),
  m_FatherJoint(0),
  m_Body(0),
  m_IDinActuated(-1)
{
  MAL_S4x4_MATRIX_SET_IDENTITY(m_globalPoseAtConstruction);
  MAL_S4x4_MATRIX_SET_IDENTITY(m_globalPoseAtConstructionNormalized);
  m_FromRootToThis.push_back(this);
  CreateLimitsArray();

  /*! Initialize spatial quantities */
  MAL_MATRIX_RESIZE(m_phi,6,0);
  MAL_MATRIX_RESIZE(m_dotphi,6,0);
}

JointPrivate::JointPrivate(int ltype, MAL_S3_VECTOR(,double) & laxis,
             float lquantite, MAL_S3_VECTOR(,double) & translationStatic):
  m_inGlobalFrame(false),
  m_nbDofs(0),
  m_type(ltype),
  m_axis(laxis),
  m_quantity(lquantite),
  m_FatherJoint(0),
  m_Body(0),
  m_IDinActuated(-1)
{
  MAL_S4x4_MATRIX_SET_IDENTITY(m_globalPoseAtConstruction);
  MAL_S4x4_MATRIX_SET_IDENTITY(m_globalPoseAtConstructionNormalized);
  MAL_S4x4_MATRIX_SET_IDENTITY(m_poseInParentFrame);
  MAL_S4x4_MATRIX_ACCESS_I_J(m_poseInParentFrame,0,3) = translationStatic[0];
  MAL_S4x4_MATRIX_ACCESS_I_J(m_poseInParentFrame,1,3) = translationStatic[1];
  MAL_S4x4_MATRIX_ACCESS_I_J(m_poseInParentFrame,2,3) = translationStatic[2];
  m_FromRootToThis.push_back(this);

  CreateLimitsArray();

  /*! Initialize spatial quantities */
  MAL_MATRIX_RESIZE(m_phi,6,0);
  MAL_MATRIX_RESIZE(m_dotphi,6,0);

}

JointPrivate::JointPrivate(int ltype, MAL_S3_VECTOR(,double) & laxis,
             float lquantite):
  m_inGlobalFrame(false),
  m_nbDofs(0),
  m_type(ltype),
  m_axis(laxis),
  m_quantity(lquantite),
  m_FatherJoint(0),
  m_Body(0),
  m_IDinActuated(-1)
{
  MAL_S4x4_MATRIX_SET_IDENTITY(m_globalPoseAtConstruction);
  MAL_S4x4_MATRIX_SET_IDENTITY(m_globalPoseAtConstructionNormalized);
  MAL_S4x4_MATRIX_SET_IDENTITY(m_poseInParentFrame);
  m_FromRootToThis.push_back(this);
  CreateLimitsArray();

  /*! Initialize spatial quantities */
  MAL_MATRIX_RESIZE(m_phi,6,0);
  MAL_MATRIX_RESIZE(m_dotphi,6,0);

}

JointPrivate::JointPrivate(const JointPrivate &r)
{
  m_type = r.type();
  m_axis = r.axis();
  m_quantity=r.quantity();
  m_poseInParentFrame=r.pose();
  m_FatherJoint = 0;
  m_Name=r.getName();
  m_IDinActuated=r.getIDinActuated();
  m_FromRootToThis.push_back(this);
  m_inGlobalFrame=r.m_inGlobalFrame;
  m_Body = 0;
  m_nbDofs = r.m_nbDofs;
  CreateLimitsArray();

  for(unsigned int i=0;i<numberDof();i++)
    {
      m_LowerLimits[i] = r.lowerBound(i);
      m_UpperLimits[i] = r.upperBound(i);
      m_LowerVelocityLimits[i] = r.lowerVelocityBound(i);
      m_UpperVelocityLimits[i] = r.upperVelocityBound(i);
    }


  /*! Initialize spatial quantities */
  MAL_MATRIX_RESIZE(m_phi,6,0);
  MAL_MATRIX_RESIZE(m_dotphi,6,0);

}

JointPrivate::JointPrivate():
  m_inGlobalFrame(false),
  m_nbDofs(0),
  m_quantity(0.0),
  m_FatherJoint(0),
  m_Body(0),
  m_IDinActuated(-1)
{
  MAL_S3_VECTOR_ACCESS(m_axis,0) = 0.0;
  MAL_S3_VECTOR_ACCESS(m_axis,1) = 0.0;
  MAL_S3_VECTOR_ACCESS(m_axis,2) = 0.0;
  MAL_S4x4_MATRIX_SET_IDENTITY(m_poseInParentFrame);
  MAL_S4x4_MATRIX_SET_IDENTITY(m_globalPoseAtConstruction);
  MAL_S4x4_MATRIX_SET_IDENTITY(m_globalPoseAtConstructionNormalized);

  m_type = FREE_JOINT;
  m_FromRootToThis.push_back(this);
  CreateLimitsArray();

  /*! Initialize spatial quantities */
  MAL_MATRIX_RESIZE(m_phi,6,0);
  MAL_MATRIX_RESIZE(m_dotphi,6,0);

}

JointPrivate::~JointPrivate()
{}

void JointPrivate::CreateLimitsArray()
{
  ODEBUG("CreateLimitsArray: "<< numberDof() << " " << getName());
  if (numberDof()!=0)
    {
      m_LowerLimits.resize(numberDof());
      m_UpperLimits.resize(numberDof());
      m_LowerVelocityLimits.resize(numberDof());
      m_UpperVelocityLimits.resize(numberDof());
      for (unsigned int i=0; i<numberDof(); i++)
        {
	  m_LowerLimits[i] = 0;
	  m_UpperLimits[i] = 0;
	  m_LowerVelocityLimits[i] = 0;
	  m_UpperVelocityLimits[i] = 0;
        }
    }
  else
    {
      m_LowerLimits.clear();
      m_UpperLimits.clear();
      m_LowerVelocityLimits.clear();
      m_UpperVelocityLimits.clear();
    }
}


JointPrivate & JointPrivate::operator=(const JointPrivate & r)
{
  m_type = r.type();
  m_axis = r.axis();
  m_quantity=r.quantity();
  m_poseInParentFrame=r.pose();
  m_Name = r.getName();
  m_IDinActuated = r.getIDinActuated();
  m_inGlobalFrame = r.m_inGlobalFrame;

  CreateLimitsArray();
  for(unsigned int i=0;i<numberDof();i++)
    {
      m_LowerLimits[i] = r.lowerBound(i);
      m_UpperLimits[i] = r.upperBound(i);
      m_LowerVelocityLimits[i] = r.lowerVelocityBound(i);
      m_UpperVelocityLimits[i] = r.upperVelocityBound(i);
    }
  return *this;
};



void JointPrivate::UpdatePoseFrom6DOFsVector(MAL_VECTOR(,double) a6DVector)
{
  // Update the orientation of the joint.
  // Takes the three euler joints

  MAL_S4x4_MATRIX_ACCESS_I_J(m_poseInParentFrame,0,3) = a6DVector(0);
  MAL_S4x4_MATRIX_ACCESS_I_J(m_poseInParentFrame,1,3) = a6DVector(1);
  MAL_S4x4_MATRIX_ACCESS_I_J(m_poseInParentFrame,2,3) = a6DVector(2);

  DynamicBodyPrivate* body = dynamic_cast<DynamicBodyPrivate*>(m_Body);
  if (!body)
    {
      std::cerr << "m_Body is not an instance of DynamicBodyPrivate" << std::endl;
    }
  body->p[0] = a6DVector(0);
  body->p[1] = a6DVector(1);
  body->p[2] = a6DVector(2);

  MAL_S3x3_MATRIX(,double) D,B,C,A;
  double CosTheta, SinTheta,
    CosPhi, SinPhi,
    CosPsi, SinPsi;


  CosPsi = cos(a6DVector(3));
  SinPsi = sin(a6DVector(3));
  CosTheta = cos(a6DVector(4));
  SinTheta = sin(a6DVector(4));
  CosPhi = cos(a6DVector(5));
  SinPhi = sin(a6DVector(5));

  //Formulae for the above commented rotation composition
  A(0,0) = CosTheta * CosPhi ;
  A(1,0) = CosTheta * SinPhi;
  A(2,0) = -SinTheta;

  A(0,1) = CosPhi * SinPsi * SinTheta - CosPsi * SinPhi;
  A(1,1) = CosPsi * CosPhi + SinPsi * SinTheta * SinPhi;
  A(2,1) = CosTheta * SinPsi;

  A(0,2) = CosPsi * CosPhi * SinTheta + SinPhi * SinPsi;
  A(1,2) = - CosPhi * SinPsi + CosPsi * SinTheta * SinPhi;
  A(2,2) = CosPsi * CosTheta;

  body->R = A;

  for(int i=0;i<3;i++)
    for(int j=0;j<3;j++)
      MAL_S4x4_MATRIX_ACCESS_I_J(m_poseInParentFrame,i,j) = A(i,j);

  ODEBUG("m_poseInParentFrame : " << m_poseInParentFrame <<
	 " A: "<<endl << A <<
	 " tmp " << endl << tmp <<
	 "C " << endl << C <<
	 "D " << endl << D <<
	 "B " << endl << B );
  body->m_transformation = m_poseInParentFrame;
}

void JointPrivate::UpdateVelocityFrom2x3DOFsVector(MAL_S3_VECTOR(,double) & aLinearVelocity,
					    MAL_S3_VECTOR(,double) & anAngularVelocity)
{
  m_RigidVelocity.linearVelocity(aLinearVelocity);
  m_RigidVelocity.rotationVelocity(anAngularVelocity);
}


void JointPrivate::RodriguesRotation(vector3d& inAxis, double inAngle, matrix3d& outRotation)
{
  vector3d wn3d;
  double norm_w = MAL_S3_VECTOR_NORM(inAxis);

  if (norm_w < 10e-7)
    {
      MAL_S3x3_MATRIX_SET_IDENTITY(outRotation);
    }
  else
    {
      double th = norm_w * inAngle;
      wn3d = inAxis / norm_w;
      double ct = cos(th);
      double lct= (1-ct);
      double st = sin(th);
      outRotation(0,0) = ct + wn3d[0]*wn3d[0]* lct;
      outRotation(0,1) = wn3d[0]*wn3d[1]*lct-wn3d[2]*st;
      outRotation(0,2) = wn3d[1] * st+wn3d[0]*wn3d[2]*lct;
      outRotation(1,0) = wn3d[2]*st +wn3d[0]*wn3d[1]*lct;
      outRotation(1,1) = ct + wn3d[1]*wn3d[1]*lct;
      outRotation(1,2) = -wn3d[0]*st+wn3d[1]*wn3d[2]*lct;
      outRotation(2,0) = -wn3d[1]*st+wn3d[0]*wn3d[2]*lct;
      outRotation(2,1) = wn3d[0]*st + wn3d[1]*wn3d[2]*lct;
      outRotation(2,2) = ct + wn3d[2]*wn3d[2]*lct;
    }
}

void JointPrivate::updateWorldCoMPosition()
{
  DynamicBodyPrivate* currentBody = (DynamicBodyPrivate*)(linkedBody());
  vector3d NE_cl,lc = currentBody->localCenterOfMass();
  MAL_S3x3_C_eq_A_by_B(m_wlc,
		       currentBody->R,lc);
  currentBody->w_c  = m_wlc + currentBody->p;
}

void JointPrivate::updateMomentum()
{
  DynamicBodyPrivate* currentBody = (DynamicBodyPrivate*)(linkedBody());
  vector3d NE_tmp,NE_tmp2, NE_tmp3;
  matrix3d NE_Rt;
  // Computes momentum matrix P.
  ODEBUG("w: " << currentBody->w );
  MAL_S3_VECTOR_CROSS_PRODUCT(NE_tmp,currentBody->w, m_wlc);
  ODEBUG("cl^w: " << NE_tmp);
  ODEBUG("mass: " << currentBody->getMass());
  ODEBUG("v0: " << currentBody->v0 );
  currentBody->P=  (currentBody->v0 +
	   NE_tmp )* currentBody->getMass();
  ODEBUG("P: " << currentBody->P);
  // Computes angular momentum matrix L
  // Lk = xc x Pk + R * I * Rt * w
  MAL_S3x3_TRANSPOSE_A_in_At(currentBody->R,NE_Rt);
  
  MAL_S3_VECTOR_CROSS_PRODUCT(NE_tmp3,currentBody->w_c,currentBody->P);
  
  MAL_S3x3_C_eq_A_by_B(NE_tmp2,NE_Rt , currentBody->w);
  MAL_S3x3_C_eq_A_by_B(NE_tmp, currentBody->getInertie(),NE_tmp2);
  MAL_S3x3_C_eq_A_by_B(NE_tmp2, currentBody->R,NE_tmp);
  currentBody->L = NE_tmp3 + NE_tmp2;
  ODEBUG("L: " << lL);

}

void JointPrivate::updateAccelerationCoM()
{
  DynamicBodyPrivate* currentBody = (DynamicBodyPrivate*)(linkedBody());
  vector3d lc = currentBody->localCenterOfMass();
  vector3d NE_tmp2,NE_tmp3;
  
  // *******************  Acceleration for the center of mass of body  i ************************
  MAL_S3_VECTOR_CROSS_PRODUCT(NE_tmp2,currentBody->lw,lc);
  MAL_S3_VECTOR_CROSS_PRODUCT(NE_tmp3,currentBody->lw,NE_tmp2);
  
  // NE_tmp2 = dw_I x r_{i,i+1}
  MAL_S3_VECTOR_CROSS_PRODUCT(NE_tmp2,currentBody->ldw,lc);
  
  currentBody->ldv_c = currentBody->ldv + NE_tmp2 + NE_tmp3;
  
  ODEBUG(currentBody->getName() << " CoM linear acceleration / local frame");
  ODEBUG(" lc = " << lc);
  ODEBUG(" w_i x (w_i x lc) = " << NE_tmp3 << " | (lwd x lc) = " << NE_tmp2);
  ODEBUG(" lw: " << currentBody->lw << " ldw: " << currentBody->ldw);
  ODEBUG(" ldv: " << currentBody->ldv);
  ODEBUG(" R_static: " << currentBody->R_static);
  ODEBUG(" b: " << currentBody->b);
  ODEBUG(" currentBody->Riip1t: " << currentBody->Riip1t);
  ODEBUG(" ldv_c: " << currentBody->ldv_c);
}	  



void JointPrivate::updateTorqueAndForce()
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
  ODEBUG(" Body name: " << CurrentBody->getName() << " : " << lg << " mass: " << CurrentBody->mass());
  tmp = CurrentBody->ldv_c + lg;
  ODEBUG(" Acceleration: " << CurrentBody->ldv_c);
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
  ODEBUG(" " << getName() << ":" << CurrentBody->m_Force);
  /* 2nd term : -f_i x r_{i,ci} */
  MAL_S3_VECTOR_CROSS_PRODUCT(sndterm,CurrentBody->m_Force, lc);
  CurrentBody->m_Torque = CurrentBody->m_Torque - sndterm;

}
