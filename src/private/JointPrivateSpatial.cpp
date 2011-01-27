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

using namespace dynamicsJRLJapan;


/*JointPrivateSpatial::JointPrivateSpatial(int ltype)
:JointPrivate()
{
	m_type = ltype;
	switch (m_type)
	{
	case FREE_JOINT:
		{	
			m_nbDofs = 6;
			// Initialize spatial quantities //
			MAL_MATRIX_RESIZE(m_phi,6,6);
			MAL_MATRIX_FILL(m_phi,0);
			MAL_MATRIX_RESIZE(m_dotphi,6,6);
			MAL_MATRIX_FILL(m_dotphi,0);
			for(unsigned int i=0;i<6;i++)
				m_phi(i,i)=1;
		}
	break;

	case REVOLUTE_JOINT:
		{
			m_nbDofs = 1;
			// Initialize spatial quantities 
			MAL_MATRIX_RESIZE(m_phi,6,1);
			MAL_MATRIX_FILL(m_phi,0);
			MAL_MATRIX_RESIZE(m_dotphi,6,1);
			MAL_MATRIX_FILL(m_dotphi,0);
			m_phi(0,0)=1;
		}
	break;

	case PRISMATIC_JOINT:
		{
			m_nbDofs = 1;
			// Initialize spatial quantities //
			MAL_MATRIX_RESIZE(m_phi,6,1);
			MAL_MATRIX_FILL(m_phi,0);
			MAL_MATRIX_RESIZE(m_dotphi,6,1);
			MAL_MATRIX_FILL(m_dotphi,0);
			m_phi(0,0)=1;
		}
	break;

	case default:
		{
			m_nbDofs = 0;
			// Initialize spatial quantities //
			MAL_MATRIX_RESIZE(m_phi,6,1);
			MAL_MATRIX_FILL(m_phi,0);
			MAL_MATRIX_RESIZE(m_dotphi,6,1);
			MAL_MATRIX_FILL(m_dotphi,0);
		}
  }
  CreateLimitsArray();
}
*/

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
/*modified by L.S*/
Spatial::PluckerTransform JointPrivate::xjcalc(const vectorN & qi)

{	   
  /*matrix3d lR;
  vector3d lp;

  MAL_S3x3_MATRIX_SET_IDENTITY(lR);

  for(unsigned int i=0;i<3;i++)
    MAL_S3_VECTOR_ACCESS(lp,i) = 0.0;
  
  return  Spatial::PluckerTransform(lR,lp);*/
  
	/*Considering the type of the joint: case of ff or rotate there are 2 expressions 
	for the local rotation matrix R of the body frame in the joint frame*/
	
	DynamicBodyPrivate* currentBody = (DynamicBodyPrivate*)(linkedBody());
	MAL_VECTOR_RESIZE(currentBody->sq, m_nbDofs);
	// Read body variables in the state vector
	for(unsigned int i=0;i<m_nbDofs;i++)
		currentBody->sq[i] = qi(rankInConfiguration()+i);
	//quantity( currentBody->sq);

	MAL_VECTOR(,double) qlin,qang;
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
					//currentBody->b = currentBody->b + qlin;
					currentBody->b[i]=qlin(i);
				}
			eulerXYZ(qang, currentBody->localR);
		}
		MAL_S3x3_MATRIX(,double) Rt = MAL_S3x3_RET_TRANSPOSE(currentBody->localR);
		MAL_S3_VECTOR(,double) t;
		MAL_S3_VECTOR_FILL(t,0);
		
	return Spatial::PluckerTransform(Rt,t);

}
		

/*const matrixNxP & JointPrivate::pcalc(const vectorN & qi)
{
   if (m_nbDofs == 1)
   {
		MAL_MATRIX_RESIZE(m_phi,6,1);
		m_phi(2,0)=1;
   }
   else if (m_nbDofs == 6)
   {
	   MAL_MATRIX_RESIZE(m_phi,6,6)
	   for(unsigned int i=0;i<6;i++)
		   m_phi(i,i)=1;
   }
   return m_phi;

}

const matrixNxP & JointPrivate::pdcalc(const vectorN & qi)
{
	if (m_nbDofs == 1)
	{
		MAL_MATRIX_RESIZE(m_dotphi,6,1);
		MAL_MATRIX_FIll(m_dotphi,0);
	}
   else if (m_nbDofs == 6)
   {
	   for(unsigned int i=0;i<6;i++)
	   {	
		   MAL_MATRIX_RESIZE(m_dotphi,6,6);
			MAL_MATRIX_FIll(m_dotphi,0);
	   }
   }
  return m_dotphi;
}*/
 
/*Computation of the rotation matrix based on euler angles and notation by L.S*/
void JointPrivate::eulerXYZ(MAL_VECTOR(,double) & qi_ang, matrix3d & localRot)
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

  //Formulae for the above commented rotation composition
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

/*Computation of the external force due to gravity by L.S*/
/*MAL_VECTOR(,double) & JointPrivate::ComputeExtForce()
{
	MAL_VECTOR(fext,double);
	MAL_VECTOR_RESIZE(fext,6);
	MAL_VECTOR_FILL(fext,0);
	const double gravity_cst = -9.81;
	fext(5)=gravity_cst;
	MAL_VECTOR(g,double);
	MAL_VECTOR_RESIZE(g,3);
	MAL_VECTOR_FILL(g,0);
	g(2)=gravity_cst;

	//lc = currentBody->localCenterOfMass();
	//wc0 = MAL_S3x3_RET_A_by_B(currentBody->R,lc);
	//wc = currentBody->p+ wc0;
	vector3d wc = currentBody->wc;
	vector3d wcg = skew(wc)*g;
	for (unsigned int i=0;i<3;i++)
		fext(i)=wcg(i);

	return fext;
}*/


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

/* Updates methods using Spatial notations */

/* Rigid transformation */
bool JointPrivate::SupdateTransformation(const vectorN& inRobotConfigVector)
{
	DynamicBodyPrivate* currentBody = (DynamicBodyPrivate*)(linkedBody());
  
	DynamicBodyPrivate* currentMotherBody = 0;
	if (parentJoint()!=0)
		currentMotherBody = (DynamicBodyPrivate*)(parentJoint()->linkedBody());
  
	Xj_i = xjcalc(inRobotConfigVector);
	Xl_i = Spatial::PluckerTransform(MAL_S3x3_RET_TRANSPOSE(currentBody->R_static),currentBody->b);

    currentBody->sXpii=Xj_i*Xl_i;

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
	return true;
}

/* Velocity update */
bool JointPrivate::SupdateVelocity(const vectorN& inRobotConfigVector,
				  const vectorN& inRobotSpeedVector)
{
	DynamicBodyPrivate* currentBody = (DynamicBodyPrivate*)(linkedBody());
	DynamicBodyPrivate* currentMotherBody = 0;
	vector3d NE_tmp, NE_tmp2; 
	if (parentJoint()!=0)
		currentMotherBody = (DynamicBodyPrivate*)(parentJoint()->linkedBody());

	MAL_VECTOR_RESIZE(currentBody->sdq, m_nbDofs);
	for(unsigned int i=0;i<m_nbDofs;i++)
	{
		currentBody->sdq[i] = inRobotSpeedVector(rankInConfiguration()+i);
	}
    // In the global frame.
	ODEBUG("dq: "<< currentBody->sdq );

	matrix3d RstaticT = MAL_S3x3_RET_TRANSPOSE(currentBody->R_static);    
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

	ODEBUG("sv: " << currentBody->sv );

  /*new code by L.S*/

	vectorN b = currentBody->sv^a;
	pdcalc(inRobotConfigVector);
	m_Zeta = MAL_RET_A_by_B(m_dotphi,currentBody->sdq); 
	m_Zeta = m_Zeta + b;

	return true;
}

bool JointPrivate::SupdateAcceleration(const vectorN& inRobotConfigVector,
				      const vectorN& inRobotSpeedVector,
				      const vectorN& inRobotAccelerationVector)
{

	DynamicBodyPrivate* currentBody = (DynamicBodyPrivate*)(linkedBody());
	DynamicBodyPrivate* currentMotherBody = 0;
  
	if (parentJoint()!=0)
    currentMotherBody = (DynamicBodyPrivate*)(parentJoint()->linkedBody());

	MAL_VECTOR_RESIZE(currentBody->sddq,m_nbDofs);
	for(unsigned int i=0;i<m_nbDofs;i++)
	  /*wrong old expression*/
//    localAcc(i) = inRobotSpeedVector(rankInConfiguration()+i);
	 /*new expression to be verified by L.S */
	{
		currentBody->sddq[i] = inRobotAccelerationVector(rankInConfiguration()+i);
	}

	 // Udpate acceleration.

	//vectorN a2 = MAL_RET_A_by_B(m_phi,localAcc);
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
	/*update force with new conform code by L.S*/
	MAL_S3x3_MATRIX(,double) lI = currentBody->getInertie();
	double lmass = currentBody->getMass();
	MAL_S3_VECTOR(,double) lc = currentBody->localCenterOfMass();
	MAL_S3_VECTOR(,double) lh=lc*lmass;
	currentBody->sIa = Spatial::Inertia(lI,lh,lmass);

	// MAL_VECTOR(,double) f_ext = ComputeExtForce();

	// Update position and orientation and used for computation of worldComPosition
	matrix3d Rtmp;
	if (currentMotherBody!=0)
    {
		// For orientation of the body.
		MAL_S3x3_C_eq_A_by_B(Rtmp ,currentMotherBody->R , currentBody->R_static);
		// For its position.
		currentBody->p = currentMotherBody->p + MAL_S3x3_RET_A_by_B(currentMotherBody->R,currentBody->b);
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

	//update world com position
	MAL_S3x3_C_eq_A_by_B(m_wlc,currentBody->R,lc);
	currentBody->w_c  = m_wlc + currentBody->p;
	const double gravity_cst = -9.81;
	vector3d g;
	MAL_S3_VECTOR_FILL(g,0);
	g(2) = gravity_cst;
	vector3d lfext,wfext;
	MAL_S3_VECTOR_FILL(lfext,0);
	MAL_S3_VECTOR_FILL(wfext,0);
	lfext = currentBody->w_c^g;
	wfext = g;

	f_ext = Spatial::Force(lfext,wfext);

	Spatial::Force sf,sf1,sf2,sftmp;
	sf = currentBody->sIa*currentBody->sa;
	Spatial::Momentum sh;
	sh = currentBody->sIa*currentBody->sv;
	sf1 = currentBody->sv^sh;
	Spatial::PluckerTransform invX0i; 
	invX0i.inverse(currentBody->sX0i);

	//----------> To develop the transpose of a spatial matrix
	//sf2 = f_ext*lmass;
	//sf2 = Spatial::transpose(invX0i)*sf2;

	//----------> force = X.f developed such that XF.f = (X)^{-T}.f or in this case X=currentBody->sX0i 
	sftmp=f_ext*lmass;
	sf2 = currentBody->sX0i*sftmp;
	sf = sf + sf1;
	currentBody->sf = sf - sf2;
	return true;
}


void JointPrivate::SupdateTorqueAndForce()
{
  DynamicBodyPrivate * currentBody = (DynamicBodyPrivate*)(linkedBody());

  DynamicBodyPrivate* currentMotherBody = 0;
  
	if (parentJoint()!=0)
    currentMotherBody = (DynamicBodyPrivate*)(parentJoint()->linkedBody());
	//----------> To develop the operator* that multiplies a matrixNxP by a Spatial::Velocity and gives as output a vectorN
	//pcalc();
	vector3d fi,ni;
	fi=currentBody->sf.f();
	ni=currentBody->sf.n0();
	MAL_VECTOR_RESIZE(currentBody->stau,MAL_MATRIX_NB_ROWS(m_phi));
	for (unsigned int i=0;i<MAL_MATRIX_NB_ROWS(m_phi);i++)
	{
		for (unsigned int j=0;j<3;j++)
		{	
			currentBody->stau[i]=m_phi(i,j)*fi(j)+m_phi(i,j+3)*ni(j);

		}
	}

	currentBody->m_Torque = currentBody->sf.n0();
    currentBody->m_Force = currentBody->sf.f();

	if (currentMotherBody!=0)
    {
	  Spatial::Force sft; 
	  //----------> To develop the transpose of a spatial matrix
	  //sft = Spatial::transpose(currentBody->sXpii)*currentBody->sf;
	  //----------> force = XF^{-1}.f developed such that XF^{-1}.f = (X)^{T}.f or in this case X=currentBody->sXpii 
	  XpiiT = Spatial::PluckerTransformTranspose(currentBody->sXpii);
	  sft = XpiiT*currentBody->sf;
      currentMotherBody->sf = currentMotherBody->sf + sft;
    }

}
