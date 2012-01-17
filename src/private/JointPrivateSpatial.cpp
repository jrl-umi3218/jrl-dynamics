/*
 * Copyright 2010,
 *
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
/* \doc implements the spatial body class.
 */
#include "Debug.h"

#include "JointPrivate.h"
#include "DynamicBodyPrivate.h"
#include <jrl/dynamics/dynamicbody.hh>

using namespace dynamicsJRLJapan;

const Spatial::Velocity & JointPrivate::sv()
{
  return m_sv;
}

const Spatial::Acceleration & JointPrivate::sa()
{
  return m_sa;
}

const Spatial::PluckerTransform & JointPrivate::XL()
{
  return m_XL;
}

const Spatial::PluckerTransform & JointPrivate::X0()
{
  return m_X0;
}

DynamicBodyPrivate * JointPrivate::getLinkedDynamicBodyPrivate() 
{
  DynamicBodyPrivate* currentBody = dynamic_cast<DynamicBodyPrivate*>(linkedBody());

  if (currentBody==0)
    {
      DynamicBody * inBody = dynamic_cast<DynamicBody *>(linkedBody());
      if (inBody!=0)
	{
	  currentBody = dynamic_cast<DynamicBodyPrivate *>(inBody->m_privateObj.get());
	}
    }
  return currentBody;
}

DynamicBodyPrivate * JointPrivate::getMotherDynamicBodyPrivate() 
{
  DynamicBodyPrivate* currentMotherBody = 0;
  
  if (parentJoint()!=0)
    {
      JointPrivate * aJP = dynamic_cast<JointPrivate *>(parentJoint());
      if (aJP!=0)
	currentMotherBody = aJP->getLinkedDynamicBodyPrivate();
    }
  return currentMotherBody;
}

void JointPrivate::resizeSpatialFields()
{
  DynamicBodyPrivate* currentBody = getLinkedDynamicBodyPrivate();
  if (currentBody!=0)
    {
      // We must ensure that there is at least one dof per joint,
      // to make the anchor joint work properly.
      unsigned nbDofs = std::max (m_nbDofs, 1u);
      MAL_VECTOR_RESIZE(currentBody->sq, nbDofs);
      MAL_VECTOR_RESIZE(currentBody->sdq, nbDofs);
      MAL_VECTOR_RESIZE(currentBody->sddq, nbDofs);
      MAL_VECTOR_RESIZE(currentBody->stau, nbDofs);

      MAL_VECTOR_FILL(currentBody->sq, 0.);
      MAL_VECTOR_FILL(currentBody->sdq, 0.);
      MAL_VECTOR_FILL(currentBody->sddq, 0.);
      MAL_VECTOR_FILL(currentBody->stau, 0.);
    }
}
/*! Spatial notations specifications */
/* modified by L.S*/
Spatial::PluckerTransform JointPrivate::xjcalc(const vectorN & qi)
{
  /* Considering the type of the joint: case of ff or rotate there are 2
   * expressions for the local rotation matrix R of the body frame in the joint
   * frame. */

  DynamicBodyPrivate* currentBody = getLinkedDynamicBodyPrivate();
  

  /* Read body variables in the state vector. */
  for(unsigned int i=0;i<m_nbDofs;i++)
    currentBody->sq[i] = qi(rankInConfiguration()+i);

  MAL_VECTOR_TYPE(double) qlin,qang;
  MAL_VECTOR_RESIZE(qlin,3);
  MAL_VECTOR_RESIZE(qang,3);

  if (m_nbDofs == 1)
    rotx(currentBody->sq, currentBody->localR);
  else if (m_nbDofs == 6)
    {
      for (int i = 0; i <3; i++)
	{
	  qlin(i)=currentBody->sq[i];
	  qang(i)=currentBody->sq[i+3];
	  currentBody->b[i]=qlin(i);
	}
      eulerXYZ(qang, currentBody->localR);
    }
  else
    MAL_S3x3_MATRIX_SET_IDENTITY(currentBody->localR);
  MAL_S3x3_MATRIX_TYPE(double) Rt = MAL_S3x3_RET_TRANSPOSE(currentBody->localR);
  MAL_S3_VECTOR_TYPE(double) t;
  MAL_S3_VECTOR_FILL(t,0);

  return Spatial::PluckerTransform(Rt,t);
}

/* Computation of the rotation matrix based on euler angles and notation by
 * L.S.
 */
void JointPrivate::eulerXYZ(MAL_VECTOR_TYPE(double) & qi_ang, matrix3d & localRot)
{
  double CosTheta, SinTheta,
    CosPhi, SinPhi,
    CosPsi, SinPsi;

  CosPsi = cos(qi_ang(0));
  SinPsi = sin(qi_ang(0));
  CosTheta = cos(qi_ang(1));
  SinTheta = sin(qi_ang(1));
  CosPhi = cos(qi_ang(2));
  SinPhi = sin(qi_ang(2));

  /* Formulae for the above commented rotation composition. */
  localRot(0,0) = CosTheta * CosPhi;
  localRot(1,0) = CosTheta * SinPhi;
  localRot(2,0) = -SinTheta;

  localRot(0,1) = - CosPsi * SinPhi + CosPhi * SinPsi * SinTheta;
  localRot(1,1) = CosPsi * CosPhi + SinPsi * SinTheta * SinPhi;
  localRot(2,1) = CosTheta * SinPsi;

  localRot(0,2) = CosPsi * CosPhi * SinTheta + SinPhi * SinPsi;
  localRot(1,2) = - CosPhi * SinPsi + CosPsi * SinTheta * SinPhi;
  localRot(2,2) = CosPsi * CosTheta;

}

void JointPrivate::rotx(const vectorN & qi_ang, matrix3d & localRot)
{
  localRot(0,0)=1;
  localRot(1,0)=0;
  localRot(2,0)=0;

  localRot(0,1)=0;
  localRot(1,1)=cos(qi_ang(0));
  localRot(2,1)=sin(qi_ang(0));

  localRot(0,2)=0;
  localRot(1,2)=-sin(qi_ang(0));
  localRot(2,2)=cos(qi_ang(0));

}

/* Computation of the skew symmetric matrix of a 3d vector by L.S. */
const MAL_S3x3_MATRIX_TYPE(double) & JointPrivate::skew(MAL_S3_VECTOR_TYPE(double) & qi_pos)
{
  sk(0,0)=0;
  sk(1,0)=qi_pos(2);
  sk(2,0)=-qi_pos(1);

  sk(0,1)=-qi_pos(2);
  sk(1,1)=0;
  sk(2,1)=qi_pos(0);

  sk(0,2)=qi_pos(1);
  sk(1,2)=-qi_pos(0);
  sk(2,2)=0;

  return sk;
}

void JointPrivate::initXL()
{
  matrix3d lR;
  vector3d lp;
  for(unsigned int i=0;i<3;i++)
    for(unsigned int j=0;j<3;j++)
      MAL_S3x3_MATRIX_ACCESS_I_J(lR,i,j) =
	MAL_S4x4_MATRIX_ACCESS_I_J(m_poseInParentFrame,i,j);

  for(unsigned int i=0;i<3;i++)
    MAL_S3_VECTOR_ACCESS(lp,i) =
      MAL_S4x4_MATRIX_ACCESS_I_J(m_poseInParentFrame,i,3);

  m_XL = Spatial::PluckerTransform(lR,lp);
  /* Assuming at first an identity matrix for Xj(i). */
  m_iXpi = m_XL;
  
  matrix3d lR_c;
  MAL_S3x3_MATRIX_SET_IDENTITY(lR_c);
  linkedDBody()->sXilc = Spatial::PluckerTransform(lR_c,
						 linkedBody()->localCenterOfMass());

}

/* Update functions by O.S */
/* Rigid transformation */
bool JointPrivate::updateTransformation(const vectorN& inRobotConfigVector)
{
  Spatial::PluckerTransform Xj = xjcalc(inRobotConfigVector);
  m_iXpi = Xj * m_XL;

  if (m_FatherJoint!=0)
    {
      Spatial::PluckerTransform piX0 = m_FatherJoint->X0();
      m_X0 = m_iXpi * piX0;
    }
  return true;
}

/* Velocity update */
bool JointPrivate::updateVelocity(const vectorN& /*inRobotConfigVector*/,
				  const vectorN& inRobotSpeedVector)
{
  vectorN localSpeed;
  MAL_VECTOR_RESIZE(localSpeed,numberDof());
  for(unsigned int i=0;i<numberDof();i++)
    localSpeed(i) = inRobotSpeedVector(rankInConfiguration()+i);

  Spatial::Velocity psv = m_FatherJoint->sv();

  m_sv = (m_iXpi * psv);
  vectorN a = MAL_RET_A_by_B(m_phi,localSpeed);
  m_sv = m_sv + a;

  m_Zeta = MAL_RET_A_by_B(m_dotphi,localSpeed);
  m_Zeta = m_Zeta + (m_sv^a);
  return true;
}

bool JointPrivate::updateAcceleration(const vectorN& /*inRobotConfigVector*/,
				      const vectorN& inRobotSpeedVector,
				      const vectorN& /*inRobotAccelerationVector*/)
{
  vectorN localAcc;
  MAL_VECTOR_RESIZE(localAcc,numberDof());
  for(unsigned int i=0;i<numberDof();i++)
    localAcc(i) = inRobotSpeedVector(rankInConfiguration()+i);

  vectorN a2 = MAL_RET_A_by_B(m_phi,localAcc);

  // Udpate acceleration.
  Spatial::Acceleration psa= m_FatherJoint->sa();
  m_sa = m_iXpi * psa;
  m_sa = m_sa + a2 + m_Zeta;

  // Update force.
  Spatial::Force af;
  af = m_sI * m_sa;
  Spatial::Momentum ah;
  ah = m_sI * m_sv;
  Spatial::Force af2;
  af2 = m_sv^ah;
  af = af + af2;
  return true;
}

/* Updates methods using Spatial notations by L.S*/

/* Rigid transformation */
bool JointPrivate::SupdateTransformation(const vectorN& inRobotConfigVector)
{
  DynamicBodyPrivate* currentBody = getLinkedDynamicBodyPrivate();
  DynamicBodyPrivate* currentMotherBody = getMotherDynamicBodyPrivate();

  Xj_i = xjcalc(inRobotConfigVector);
  Xl_i = Spatial::PluckerTransform(MAL_S3x3_RET_TRANSPOSE(currentBody->R_static),
				   currentBody->b);

  currentBody->sXpii=Xj_i*Xl_i;
  matrix3d Rpii;
  Rpii=currentBody->sXpii.R();
  vector3d Tpii;
  Tpii=currentBody->sXpii.p();

  matrix3d Rtmp2;
  Rtmp2 = MAL_S3x3_RET_A_by_B(currentBody->R_static,currentBody->localR);
  X0i_j=Spatial::PluckerTransform(MAL_S3x3_RET_TRANSPOSE(Rtmp2),currentBody->b);

  if (currentMotherBody!=0)
    {
      Spatial::PluckerTransform piX0 = currentMotherBody->sX0i;
      currentBody->sX0i = currentBody->sXpii*piX0;
    }
  else
    currentBody->sX0i=X0i_j;

  // Update position and orientation and used for computation of worldComPosition
  matrix3d Rtmp;
  if (currentMotherBody!=0)
    {
      // For orientation of the body.
      MAL_S3x3_C_eq_A_by_B(Rtmp ,currentMotherBody->R , currentBody->R_static);
      // For its position.
      currentBody->p = currentMotherBody->p
	+ MAL_S3x3_RET_A_by_B(currentMotherBody->R,currentBody->b);
      // Update the translation/rotation axis of joint.
      MAL_S3x3_C_eq_A_by_B(currentBody->w_a,Rtmp, currentBody->a);
    }
  else
    {
      // For orientation of the body.
      Rtmp = currentBody->R_static;
      // For its position.
      currentBody->p = currentBody->b;
    }

  // Put it in a homogeneous form.
  MAL_S3x3_C_eq_A_by_B(currentBody->R , Rtmp, currentBody->localR);

  for( unsigned int i=0;i<3;i++)
    for(unsigned int j=0;j<3;j++)
      MAL_S4x4_MATRIX_ACCESS_I_J(currentBody->m_transformation,i,j) = currentBody->R(i,j);

  for( unsigned int i=0;i<3;i++)
    MAL_S4x4_MATRIX_ACCESS_I_J(currentBody->m_transformation,i,3) = currentBody->p(i);

  MAL_S3x3_C_eq_A_by_B(m_wlc,currentBody->R,currentBody->localCenterOfMass());
  currentBody->w_c  = m_wlc + currentBody->p;

  ODEBUG4INC(currentBody->w_c , "CoMs.dat", " ");
  return true;
}

/* Velocity update */
bool JointPrivate::SupdateVelocity(const vectorN& inRobotConfigVector,
				   const vectorN& inRobotSpeedVector)
{
  DynamicBodyPrivate* currentBody = getLinkedDynamicBodyPrivate();
  DynamicBodyPrivate* currentMotherBody = getMotherDynamicBodyPrivate();
  vector3d NE_tmp, NE_tmp2;

  for(unsigned int i=0;i<m_nbDofs;i++)
    {
      currentBody->sdq[i] = inRobotSpeedVector(rankInConfiguration()+i);
    }
  // In the global frame.
  ODEBUG("dq: "<< currentBody->sdq );

  // Computes the angular velocity

  pcalc(inRobotConfigVector);
  vectorN a = MAL_RET_A_by_B(m_phi,currentBody->sdq);

  if (currentMotherBody!=0)
    {
      Spatial::Velocity psv = currentMotherBody->sv;
      Spatial::Velocity sv_tmp = currentBody->sXpii*psv;
      currentBody->sv  = sv_tmp  + a;
    }
  else
    currentBody->sv  = a;

  /*new code by L.S*/

  vectorN b = currentBody->sv^a;
  pdcalc(inRobotConfigVector);
  m_Zeta = MAL_RET_A_by_B(m_dotphi,currentBody->sdq);
  m_Zeta = m_Zeta + b;

  return true;
}

bool JointPrivate::SupdateAcceleration(const vectorN& /*inRobotConfigVector*/,
				       const vectorN& /*inRobotSpeedVector*/,
				       const vectorN& inRobotAccelerationVector)
{
  DynamicBodyPrivate* currentBody = getLinkedDynamicBodyPrivate();
  DynamicBodyPrivate* currentMotherBody = getMotherDynamicBodyPrivate();

  for(unsigned int i=0;i<m_nbDofs;i++)

    {
      currentBody->sddq[i] = inRobotAccelerationVector(rankInConfiguration()+i);
    }

  // Udpate acceleration.
  vectorN a2 = MAL_RET_A_by_B(m_phi,currentBody->sddq);
  a2 = a2 + m_Zeta;

  if (currentMotherBody!=0)
    {
      Spatial::Acceleration psa= currentMotherBody->sa;
      Spatial::Acceleration sa_tmp = currentBody->sXpii*psa;
      currentBody->sa  = sa_tmp + a2;
    }
  else
    {
      currentBody->sa  = a2;
    }
  //compute the force on body i
  /* Update force with new conform code by L.S. */
  MAL_S3x3_MATRIX_TYPE(double) lI = currentBody->getInertie();
  double lmass = currentBody->getMass();
  MAL_S3_VECTOR_TYPE(double) lc = currentBody->localCenterOfMass();
  MAL_S3x3_MATRIX_TYPE(double) Sc = skew(lc);
  MAL_S3x3_MATRIX_TYPE(double) ScSct = MAL_S3x3_RET_A_by_B(Sc,MAL_S3x3_RET_TRANSPOSE(Sc));
  lI = lI + ScSct*lmass;
  MAL_S3_VECTOR_TYPE(double) lh=lc*lmass;
  currentBody->sIa = Spatial::Inertia(lI,lh,lmass);

  // Update world com position
  const double gravity_cst = -9.81;//0;
  vector3d g;
  MAL_S3_VECTOR_FILL(g,0);
  g(2) = gravity_cst;
  vector3d lfext,wfext;
  MAL_S3_VECTOR_FILL(lfext,0);
  MAL_S3_VECTOR_FILL(wfext,0);
  wfext = currentBody->w_c^g;
  lfext = g;

  f_ext = Spatial::Force(lfext,wfext);

  Spatial::Force sf,sf1,sf2,sftmp;
  sf = currentBody->sIa*currentBody->sa;
  Spatial::Momentum sh;
  sh = currentBody->sIa*currentBody->sv;
  sf1 = currentBody->sv^sh;

  /* ----------> force = X.f developed such that XF.f = (X)^{-T}.f or in this
                 case X=currentBody->sX0i cf. Springer Handbook of Robotics
                 Table 2.1. */
  sftmp=f_ext*lmass;
  sf2 = currentBody->sX0i*sftmp;
  sf = sf + sf1;
  currentBody->sf = sf - sf2;

  return true;
}

void JointPrivate::SupdateTorqueAndForce()
{
  DynamicBodyPrivate * currentBody = getLinkedDynamicBodyPrivate();
  DynamicBodyPrivate* currentMotherBody = getMotherDynamicBodyPrivate();

  vector3d fi,ni;
  fi=currentBody->sf.f();
  ni=currentBody->sf.n0();
  MAL_VECTOR_DIM(t, double, 6);
  for (unsigned int j=0;j<3;j++)
    {
      t(j)=fi(j);
      t(j+3)=ni(j);
    }

  currentBody->stau = MAL_RET_A_by_B(MAL_RET_TRANSPOSE(m_phi),t);

  currentBody->m_Torque = currentBody->sf.n0();
  currentBody->m_Force = currentBody->sf.f();

  if (currentMotherBody!=0)
    {
      Spatial::Force sft;
      /* ----------> force = XF^{-1}.
       * f developed such that XF^{-1}.f = (X)^{T}.f or in this case
       * X=currentBody->sXpii cf. Springer Handbook of Robotics Table 2.1. */
      XpiiT = Spatial::PluckerTransformTranspose(currentBody->sXpii);
      sft = XpiiT*currentBody->sf;
      currentMotherBody->sf = currentMotherBody->sf + sft;
    }
}
