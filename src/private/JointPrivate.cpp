/*
 * Copyright 2010,
 *
 * Oussama Kanoun
 * Francois Keith
 * Florent Lamiraux
 * Olivier Stasse
 *
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
#include "Debug.h"

#include "JointPrivate.h"
#include "DynamicBodyPrivate.h"

namespace dynamicsJRLJapan {


JointPrivate::JointPrivate(int ltype, MAL_S3_VECTOR_TYPE(double) & laxis,
			   double lquantite, MAL_S4x4_MATRIX_TYPE(double) & lpose):
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

JointPrivate::JointPrivate(int ltype, MAL_S3_VECTOR_TYPE(double) & laxis,
			   double lquantite, MAL_S3_VECTOR_TYPE(double) & translationStatic):
  m_inGlobalFrame(false),
  m_globalPoseAtConstruction(),
  m_globalPoseAtConstructionNormalized(),
  m_Children(),
  m_nbDofs(0),
  m_STcoef(),
  m_STmcom(),
  m_wlc(),
  m_type(ltype),
  m_axis(laxis),
  m_quantity(lquantite),
  m_poseInParentFrame(),
  m_FatherJoint(0),
  m_FromRootToThis(),
  m_Body(0),
  m_dynBody(),
  m_Name(),
  m_IDinActuated(-1),
  m_LowerLimits(),
  m_UpperLimits(),
  m_LowerVelocityLimits(),
  m_UpperVelocityLimits(),
  m_LowerTorqueLimits(),
  m_UpperTorqueLimits(),
  m_EquivalentInertia(),
  m_RigidVelocity(),
  m_J(),
  m_StateVectorPosition(),
  m_XL(),
  m_iXpi(),
  m_X0(),
  m_sv(),
  m_sa(),
  m_sf(),
  m_sI(),
  m_phi(),
  m_dotphi(),
  m_Zeta(),
  sk(),
  Xl_i(),
  Xj_i(),
  X0i_j(),
  f_ext(),
  XpiiT()
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

JointPrivate::JointPrivate(int ltype, MAL_S3_VECTOR_TYPE(double) & laxis,
			   double lquantite):
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
  m_poseInParentFrame = r.pose();
  m_inGlobalFrame = r.getinGlobalFrame();
  m_Body = 0;
  m_nbDofs = r.m_nbDofs;
  m_globalPoseAtConstruction = r.m_globalPoseAtConstruction;
  m_globalPoseAtConstructionNormalized = r.initialPosition();
  CreateLimitsArray();

  for(unsigned int i=0;i<numberDof();i++)
    {
      m_LowerLimits[i] = r.lowerBound(i);
      m_UpperLimits[i] = r.upperBound(i);
      m_LowerVelocityLimits[i] = r.lowerVelocityBound(i);
      m_UpperVelocityLimits[i] = r.upperVelocityBound(i);
      m_LowerTorqueLimits[i] = r.lowerTorqueBound(i);
      m_UpperTorqueLimits[i] = r.upperTorqueBound(i);
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

void JointPrivate::setnumberDof(unsigned int lNbDofs)
{
  m_nbDofs = lNbDofs;
  CreateLimitsArray();
  resizeSpatialFields();
}

void JointPrivate::CreateLimitsArray()
{
  ODEBUG("CreateLimitsArray: "<< numberDof() << " " << getName());
  if (numberDof()!=0)
    {
      m_LowerLimits.resize(numberDof());
      m_UpperLimits.resize(numberDof());
      m_LowerVelocityLimits.resize(numberDof());
      m_UpperVelocityLimits.resize(numberDof());
      m_LowerTorqueLimits.resize(numberDof());
      m_UpperTorqueLimits.resize(numberDof());
      for (unsigned int i=0; i<numberDof(); i++)
        {
	  m_LowerLimits[i] = 0;
	  m_UpperLimits[i] = 0;
	  m_LowerVelocityLimits[i] = 0;
	  m_UpperVelocityLimits[i] = 0;
	  m_LowerTorqueLimits[i] = 0;
	  m_UpperTorqueLimits[i] = 0;
        }
    }
  else
    {
      m_LowerLimits.clear();
      m_UpperLimits.clear();
      m_LowerVelocityLimits.clear();
      m_UpperVelocityLimits.clear();
      m_LowerTorqueLimits.clear();
      m_UpperTorqueLimits.clear();
    }
}


JointPrivate & JointPrivate::operator=(const JointPrivate & r)
{
  m_type = r.type();
  m_axis = r.axis();
  m_quantity=r.quantity();
  m_poseInParentFrame=r.pose();
  m_FatherJoint = 0;
  m_Name=r.getName();
  m_IDinActuated=r.getIDinActuated();
  m_FromRootToThis.push_back(this);
  m_inGlobalFrame = r.getinGlobalFrame();

  const  MAL_S4x4_MATRIX_TYPE(double) & aGlobalPoseAtConstructionNormalized
    = r.initialPosition();
  m_globalPoseAtConstruction = aGlobalPoseAtConstructionNormalized;
  m_globalPoseAtConstructionNormalized  = aGlobalPoseAtConstructionNormalized ;
  m_Body = 0;
  m_nbDofs = r.numberDof();
  CreateLimitsArray();

  for(unsigned int i=0;i<numberDof();i++)
    {
      m_LowerLimits[i] = r.lowerBound(i);
      m_UpperLimits[i] = r.upperBound(i);
      m_LowerVelocityLimits[i] = r.lowerVelocityBound(i);
      m_UpperVelocityLimits[i] = r.upperVelocityBound(i);
      m_LowerTorqueLimits[i] = r.lowerTorqueBound(i);
      m_UpperTorqueLimits[i] = r.upperTorqueBound(i);
    }


  /*! Initialize spatial quantities */
  MAL_MATRIX_RESIZE(m_phi,6,0);
  MAL_MATRIX_RESIZE(m_dotphi,6,0);
  return *this;
}

ostream & operator<<(ostream & os, const JointPrivate &r)
{
  os << "Type: ";
  if (r.type()==JointPrivate::FREE_JOINT)
    os << "FREE_JOINT" ;
  else if (r.type()==JointPrivate::PRISMATIC_JOINT)
    os << "PRISMATIC_JOINT";
  else if (r.type()==JointPrivate::REVOLUTE_JOINT)
    os << "REVOLUTE_JOINT";
  else if (r.type()==JointPrivate::FIX_JOINT)
    os << "FIX_JOINT";
  os << endl;

  os << "Number of DOFs:" << r.numberDof() << endl;

  os << "Axis:" << r.axis() << endl;

  os << "Quantity: " << r.quantity() << endl;

  os << "Name : " << r.getName() << endl;

  os << "ID in Actuated: " << r.getIDinActuated() << endl;

  os << "Position in parent frame:" << r.pose() << endl;

  os << "Initial position in global frame:" << r.initialPosition() << endl;

  os << "In global frame :" << r.getinGlobalFrame() << endl;

  os << "Position, Velocity, and Torque Limits: " << endl;
  for(unsigned int i=0;i<r.numberDof();i++)
    {
      os << i << " [ " << r.lowerBound(i) << " , "  << r.upperBound(i) << "]"
	 << " [ " << r.lowerVelocityBound(i) << " , "  << r.upperVelocityBound(i) << "]"
	 << " [ " << r.lowerTorqueBound(i) << " , "  << r.upperTorqueBound(i) << "]"
	 << endl;
    }
  return os;
}

void JointPrivate::UpdatePoseFrom6DOFsVector(MAL_VECTOR_TYPE(double) a6DVector)
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

  MAL_S3x3_MATRIX_TYPE(double) D,B,C,A;
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
	 "C " << endl << C <<
	 "D " << endl << D <<
	 "B " << endl << B );
  body->m_transformation = m_poseInParentFrame;
}

void JointPrivate::
UpdateVelocityFrom2x3DOFsVector(MAL_S3_VECTOR_TYPE(double) & aLinearVelocity,
				MAL_S3_VECTOR_TYPE(double) & anAngularVelocity)
{
  m_RigidVelocity.linearVelocity(aLinearVelocity);
  m_RigidVelocity.rotationVelocity(anAngularVelocity);
}


void JointPrivate::
RodriguesRotation(vector3d& inAxis, double inAngle, matrix3d& outRotation)
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
  MAL_S3x3_TRANSPOSE_A_in_At(currentBody->R,NE_Rt);

  // Global velocities (angular and linear)
  vector3d gv= currentBody->R*currentBody->sv.v0();
  vector3d gw=currentBody->R*currentBody->sv.w();
  // Computes momentum matrix P.
  ODEBUG("w: " << gw );
  MAL_S3_VECTOR_CROSS_PRODUCT(NE_tmp,gw,
			      currentBody->w_c);
  ODEBUG("cl^w: " << NE_tmp);
  ODEBUG("mass: " << currentBody->getMass());
  ODEBUG("v0: " << gv );
  currentBody->P=  (gv +
	   NE_tmp )* currentBody->getMass();
  ODEBUG("P: " << currentBody->P);
  // Computes angular momentum matrix L
  // Lk = xc x Pk + R * I * Rt * w

  MAL_S3_VECTOR_CROSS_PRODUCT(NE_tmp3,currentBody->w_c,currentBody->P);

  MAL_S3x3_C_eq_A_by_B(NE_tmp2,NE_Rt , currentBody->w);
  MAL_S3x3_C_eq_A_by_B(NE_tmp, currentBody->getInertie(),NE_tmp2);
  MAL_S3x3_C_eq_A_by_B(NE_tmp2, currentBody->R,NE_tmp);
  currentBody->L = NE_tmp3 + NE_tmp2;
  ODEBUG("L: " << currentBody->L);

}

void JointPrivate::updateAccelerationCoM()
{
  DynamicBodyPrivate* currentBody = (DynamicBodyPrivate*)(linkedBody());
  vector3d lc = currentBody->localCenterOfMass();
  vector3d NE_tmp2,NE_tmp3;

  /* *******************  Acceleration for the center of mass of body  i ******************* */
  MAL_S3_VECTOR_CROSS_PRODUCT(NE_tmp2,currentBody->sv.w(),lc);
  MAL_S3_VECTOR_CROSS_PRODUCT(NE_tmp3,currentBody->sv.w(),NE_tmp2);

  // NE_tmp2 = dw_I x r_{i,i+1}
  MAL_S3_VECTOR_CROSS_PRODUCT(NE_tmp2,currentBody->sa.dw(),lc);

  currentBody->ldv_c = currentBody->sa.dv0() + NE_tmp2 + NE_tmp3;

  ODEBUG(currentBody->getName() << " CoM linear acceleration / local frame");
  ODEBUG(" lc = " << lc);
  ODEBUG(" w_i x (w_i x lc) = " << NE_tmp3 << " | (lwd x lc) = " << NE_tmp2);
  ODEBUG(" lw: " << currentBody->sv.w() << " ldw: " << currentBody->sa.dw());
  ODEBUG(" ldv: " << currentBody->sa.dv0());
  ODEBUG(" R_static: " << currentBody->R_static);
  ODEBUG(" b: " << currentBody->b);
  ODEBUG(" currentBody->Riip1t: " << currentBody->Riip1t);
  ODEBUG(" ldv_c: " << currentBody->ldv_c);
}



void JointPrivate::updateTorqueAndForce()
{
  DynamicBodyPrivate * CurrentBody = (DynamicBodyPrivate*)(linkedBody());

  MAL_S3x3_MATRIX_TYPE(double) aRt;

  MAL_S3x3_MATRIX_TYPE(double) currentBodyRt;
  currentBodyRt = MAL_S3x3_RET_TRANSPOSE(CurrentBody->R);

  /*//MAL_S3x3_MATRIX_TYPE(double) currentBodyinitialRt;
  matrix4d initialTransform  = CurrentBody->joint()->initialPosition();
  for(unsigned int li=0;li<3;li++)
    for(unsigned int lj=0;lj<3;lj++)
      currentBodyRt = MAL_S4x4_MATRIX_ACCESS_I_J(initialTransform,lj,li);

  // currentBodyRt = MAL_S3x3_RET_A_by_B(currentBodyRt,currentBodyinitialRt);
  */
  MAL_S3_VECTOR_TYPE(double) lg;
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
  MAL_S3_VECTOR_TYPE(double) firstterm,
    sndterm, thirdterm, fifthterm,tmp;
  // Do not fourth term because it is the angular acceleration.

  /* Force - Constant part: 2nd and 3rd term of eq.(7.146)
     m_i a_{c,i} - m_i g_i
   */
  ODEBUG(" Body name: " << CurrentBody->getName() << " : "
	 << lg << " mass: " << CurrentBody->mass());
  tmp = CurrentBody->ldv_c - lg;
  ODEBUG(" Acceleration: " << CurrentBody->ldv_c);
  CurrentBody->m_Force =  tmp * CurrentBody->mass();
  /* Get the local center of mass */
  vector3d lc = CurrentBody->localCenterOfMass();

  /* Torque - 5th term : w_i x (I_i w_i)*/
  MAL_S3x3_MATRIX_TYPE(double) lI = CurrentBody->getInertie();
  tmp = MAL_S3x3_RET_A_by_B(lI,CurrentBody->lw);
  //  tmp = MAL_S3x3_RET_A_by_B(lI,CurrentBody->w);


  MAL_S3_VECTOR_CROSS_PRODUCT(fifthterm,CurrentBody->lw,tmp);

  /* Torque - 4th term and 5th term
     Torque_i = I_i * alpha_i +  w_i x I_i w_i) */
  MAL_S3x3_C_eq_A_by_B(tmp,lI,CurrentBody->ldw);
  CurrentBody->m_Torque =  tmp + fifthterm ;

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

bool JointPrivate::getinGlobalFrame() const
{
  return m_inGlobalFrame;
}

}
