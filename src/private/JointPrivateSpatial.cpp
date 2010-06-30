/* \doc implements the spatial body class.
  Copyright (c) 2010
  @author Olivier Stasse
   
  JRL-Japan, CNRS/AIST
 
  All rights reserved.

  Please refers to file License.txt for details on the license.
   
*/
#include "Debug.h"

#include "JointPrivate.h"

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

/*! Spatial notations specifications */

Spatial::PluckerTransform JointPrivate::xjcalc(const vectorN & qi)
{
  // Default value.
  // This method should be re-implemented according
  // to the nature of the joint.
  matrix3d lR;
  vector3d lp;
  
  MAL_S3x3_MATRIX_SET_IDENTITY(lR);
  
  for(unsigned int i=0;i<3;i++)
    MAL_S3_VECTOR_ACCESS(lp,i) = 0.0;
  
  return  Spatial::PluckerTransform(lR,lp);
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
  // Assuming at first an identity matrix for Xj(i).
  m_iXpi = m_XL;
}

/* Updates methods using Spatial notations */

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
bool JointPrivate::updateVelocity(const vectorN& inRobotConfigVector,
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

bool JointPrivate::updateAcceleration(const vectorN& inRobotConfigVector,
				      const vectorN& inRobotSpeedVector,
				      const vectorN& inRobotAccelerationVector)
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

const matrixNxP & JointPrivate::pcalc(vectorN &qi)
{
  return m_phi;
}

const matrixNxP & JointPrivate::pdcalc(vectorN &qi)
{
  return m_dotphi;
}
