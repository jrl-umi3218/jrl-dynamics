/* \doc implements the spatial body class.
  Copyright (c) 2010
  @author Olivier Stasse
   
  JRL-Japan, CNRS/AIST
 
  All rights reserved.

  Please refers to file License.txt for details on the license.
   
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
Spatial::PluckerTransform JointPrivate::xjcalc()

{	   
  matrix3d lR;
  vector3d lp;
  
  MAL_S3x3_MATRIX_SET_IDENTITY(lR);
  
  for(unsigned int i=0;i<3;i++)
    MAL_S3_VECTOR_ACCESS(lp,i) = 0.0;
  
  return  Spatial::PluckerTransform(lR,lp);
  /* Spatial::PluckerTransform Xj_i;
   if (m_nbDofs == 1)
   {
		MAL_S3x3_MATRIX(,double) R = JointPrivate::rotx(qi); 
		
	else if (m_nbDofs ==  6)
	    MAL_S3_VECTOR(,double) q_ang;
		for (unisgned int i=0;i<3;i++)
		{
			q_ang(i)=qi(i+3);
		}
		MAL_S3x3_MATRIX(,double) R = JointPrivate::eulerXYZ(q_ang); 

	}
		MAL_S3x3_MATRIX(,double) Rt = MAL_S3x3_RET_TRANSPOSE(R);
		MAL_S3_VECTOR(,double) t;
		MAL_S3_VECTOR_FILL(t,0);
		//t(0)=0;t(1)=0;t(2)=0;
		MAL_S3x3_MATRIX(,double) s = JointPrivate::skew(t);
		
			
			//MAL_MATRIX_FILL(Xj_i,0); 
			
			for (unsigned int k=0;k<3;k++)
			{
				Xj_i(k,k) = Rt(k,k); 
				Xj_i(k+3,k) = MAL_S3x3_RET_TRANSPOSE(MAL_S3x3_RET_A_by_B(skew(t),R)); 
				Xj_i(k+3,k+3) = Rt(k,k);
			}

			return Xj;*/
				// Default value.
  // This method should be re-implemented according
  // to the nature of the joint.

}
		

const matrixNxP & JointPrivate::pcalc()
{
   if (m_nbDofs == 1)
		m_phi(2,0)=1;
   else if (m_nbDofs == 6)
   {
	   for(unsigned int i=0;i<6;i++)
		   m_phi(i,i)=1;
   }
   return m_phi;

}

const matrixNxP & JointPrivate::pdcalc()
{
  return m_dotphi;
}
 
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
/*Computation of the skew symmetric matrix of a 3d vector by L.S*/
const MAL_S3x3_MATRIX(,double) & JointPrivate::skew(MAL_S3_VECTOR(,double) & qi_pos)
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
/*Computation of the spatial skew symmetric matrix of a 6d vector by L.S*/
/*Spatial::PluckerTransform & JointPrivate::spatialskew(const vectorN & qi)
{
	for (unsigned int i=0;i<3;i++)
	{
		qipos(i)=qi(i);
		qiang=qi(i+3);
	}

	for (unsigned int j=0;j<3;j++)
	{
		ssk(j,j)=skew(qipos);
		ssk(j+3,j)=skew(qiang);
		ssk(j+3,j+3)=skew(qipos);
	}

	return ssk;
}*/
/*Computation of the world quantities of position, rotation and CoM of a body ---> still in progress by L.S*/
/*void JointPrivate::ComputeWorldPRC(const vectorN & qi)
{
	p_i=currentBody->b;
	
	int dim = qi.size();
	if (dim == 1)
		mp_i = p_i;
    else if (dim == 6)
	{
		for (unsigned int i=0;i<3;i++)
		{
			qipos(i)=qi(i);
			qiang(i)=qi(i+3);
		}
		for(unsigned int j=0;j<3;j++)
		mp_i(j) = p_i(j)+qipos(j); 
	}
  
  RS_i = currentBody->R_static;
  lc_i = currentBody->localCenterofMass();

  if(index==1)
    {
	// mPA_waist et MRA_waist	
	mPA_j.Fill(0);
	MAL_S3x3_MATRIX_SET_IDENTITY(mRA_j);  
  else
    //int j = index-1;
    mPA_j = mPA_parent;
    mRA_j = mRA_parent;
	}

	//Body->p;Body->R are the absolute quantities expressed in world frame

	MAL_S3_VECTOR(Rp,double);
	Rp = mRA_j * mp_i;
	mPA_i = mPA_j + Rp;
	MAL_S3x3_MATRIX RaRs;
	MAL_C_eq_A_by_B(RaRs,mRA_j,RS_i)
	if (dim_i == 6)
		MAL_C_eq_A_by_B(mRA_i,RaRs,eulerXYZ(qiang));
	elseif (dim_i == 1)  
		MAL_C_eq_A_by_B(mRA_i,RaRs,rotx(qi));
    end

	mPA_parent = mPA_i;
	mRA_parent = mRA_i;

	MAL_S3_VECTOR(lc,double);
	lc = mRA_i * lc_i;
  // center of mass of the body in the world frame
  lcW_i = mPA_i + lc;
  index++;
}*/

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

/*by L.S*/
/*
Spatial::PluckerTransform & JointPrivate::computeXL(const vectorN & qi)
{
	MAL_S3_VECTOR(,double) qi_ang,qi_lin;
	MAL_S3_VECTOR(,double) mp_i;
		for (unisgned int i=0;i<3;i++)
		{
			qi_lin(i)=qi(i);
			qi_ang(i)=qi(i+3);
		}
	//int dim = qi.size();
	if (m_nbDofs == 1)
		mp_i=currentBody->b;
	else if (m_nbDofs == 6)
		{
			mp_i = currentBody->b;
			mp_i += qi_lin;
		}

	MAL_MATRIX_FILL(Xl_i,0);

	MAL_S3x3_MATRIX(,double) RSi = currentBody->R_static;
	for (unsigned int i=0;i<6;i++)
	{	
		Xl_i(i,i)=MAL_RET_TRANSPOSE(RSi);
		Xl_i(i+3,i)=MAL_RET_TRANSPOSE(skew(mp_i*RSi));
		Xl_i(i+3,i+3)=MAL_RET_TRANSPOSE(RSi);
	}
	return Xl_i;
}
*/
/*by L.S*/
/*
Spatial::PluckerTransform & JointPrivate::computeXPi()
{
	Xl_i= computeXL();
	Xj_i = xjcalc(inRobotConfigVector);
	Xpi_i=Xj_i*Xl_i;
	return Xpi_i;
}
*/
/*by L.S*/
/*
Spatial::PluckerTransform & JointPrivate::computeX0(const vectorN & qi)
{
	MAL_S3_VECTOR(,double) qi_ang,qi_lin;
	MAL_S3_VECTOR(,double) mp_i;
		
	//int dim = qi.size();
	if (m_nbDofs == 1)
		mp_i=currentBody->b;
	else if (m_nbDofs == 6)
		{
			for (unisgned int i=0;i<3;i++)
			{
				qi_lin(i)=qi(i);
				qi_ang(i)=qi(i+3);
			}
			mp_i = currentBody->b;
			mp_i += qi_lin;
		}
	int parent_index=index-1;
	Xpi_i = computeXPi();
	
	X0i_j.Fill(0);
	MAL_S3x3_MATRIX(,double) RSi = currentBody->R_static;
	if(index==1)
	{
	if (m_nbDofs==1)
	{
		for (unsigned int k=0;k<3;k++)
		{
			X0i_j(k,k)=MAL_RET_TRANSPOSE(RSi*rotx(qi));
			X0i_j(k+3,k)=MAL_RET_TRANSPOSE(skew(mp_i)*RSi*rotx(qi));
			X0i_j(k+3,k+3)=MAL_RET_TRANSPOSE(RSi*rotx(qi));
		}
	else if (m_nbDofs==6)
		for (unsigned int k=0;k<3;k++)
		{
			X0i_j(k,k)=MAL_RET_TRANSPOSE(RSi*eulerXYZ(qi_ang));
			X0i_j(k+3,k)=MAL_RET_TRANSPOSE(skew(mp_i)*RSi*eulerXYZ(qi_ang));
			X0i_j(k+3,k+3)=MAL_RET_TRANSPOSE(RSi*eulerXYZ(qi_ang));
		}
	}
	else
	X0i_jparent=X0i_j;
	}
	if (parent_index!=0) //(m_FatherJoint!=0)
	{
		X0i_i= Xpi_i*X0i_jparent;
	else
		X0i_i= X0i_j;
	}
index++;
return X0i_i;
}
*/


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
bool JointPrivate::SupdateTransformation(const vectorN& inRobotConfigVector)
{
	std::cout << "in FF Spatial update Transfo" << std::endl;
	  DynamicBodyPrivate* body = (DynamicBodyPrivate*)(linkedBody());
  
  DynamicBodyPrivate* parentbody = 0;
  if (parentJoint()!=0)
    parentbody = (DynamicBodyPrivate*)(parentJoint()->linkedBody());
  MAL_VECTOR_RESIZE(body->sq, m_nbDofs);
  // Read body variables in the state vector
  for(unsigned int i=0;i<m_nbDofs;i++)
  body->sq[i] = inRobotConfigVector(rankInConfiguration()+i);
  //quantity( body->sq);
  MAL_VECTOR(,double) qlin,qang;
  MAL_VECTOR_RESIZE(qlin,3);
  MAL_VECTOR_RESIZE(qang,3);

  //Considering the type of the joint: case of ff or rotate there are 2 expressions for the local rotation matrix R of the body frame in the joint frame
  //int dim = body->sq.size();
   if (m_nbDofs == 1)
		rotx(body->sq, body->localR);
   else if (m_nbDofs == 6)
		{	
			for (int i = 0; i <3; i++)
				{
					qlin(i)=body->sq[i];
					qang(i)=body->sq[i+3];
				}

			body->b[0] = qlin(0);
			body->b[1] = qlin(1);
			body->b[2] = qlin(2);
			//body->b = body->b + qlin;
			eulerXYZ(qang, body->localR);
		}
  
  
  // Update position and orientation and worldComPosition
  matrix3d Rtmp;
  if (parentbody!=0)
    {
      // For orientation of the body.
      MAL_S3x3_C_eq_A_by_B(Rtmp ,parentbody->R , body->R_static);
      // For its position.
      body->p = parentbody->p + MAL_S3x3_RET_A_by_B(parentbody->R,body->b);
  
    }
  else
    {
      // For orientation of the body.
      Rtmp = body->R_static;
      // For its position.
      body->p = body->b;
    }

  // Put it in a homogeneous form.
  MAL_S3x3_C_eq_A_by_B(body->R , Rtmp, body->localR);
  
  //update world com position
 /* vector3d NE_cl,lc = body->localCenterOfMass();
  MAL_S3x3_C_eq_A_by_B(m_wlc,
		       body->R,lc);
  currentBody->w_c  = m_wlc + currentBody->p;*/

 /* matrix3d RS = body->R_static;
  matrix3d Sk = skew(body->b);
  matrix3d SkRS;
  MAL_S3x3_C_eq_A_by_B(SkRS,Sk,RS);

  // Compute m_XL: local transformation matrix from body frame to joint frame
  for (unsigned int i=0;i<6;i++)
	{	
		Xl_i(i,i)=MAL_RET_TRANSPOSE(RS);
		Xl_i(i+3,i)=MAL_RET_TRANSPOSE(SkRS);
		Xl_i(i+3,i+3)=MAL_RET_TRANSPOSE(RS);
	}
  */
  Xl_i = Spatial::PluckerTransform(MAL_S3x3_RET_TRANSPOSE(body->R_static),body->b);

  // Spatial::PluckerTransform Xj = xjcalc(inRobotConfigVector);

	/*MAL_S3x3_MATRIX(,double) Rt = MAL_S3x3_RET_TRANSPOSE(body->localR);
	MAL_S3_VECTOR(,double) t;
	MAL_S3_VECTOR_FILL(t,0);
	MAL_S3x3_MATRIX(,double) s = JointPrivate::skew(t);
		
			
			//MAL_MATRIX_FILL(Xj_i,0); 
			
		for (unsigned int k=0;k<3;k++)
			{
				Xj_i(k,k) = Rt(k,k); 
				Xj_i(k+3,k) = MAL_S3x3_RET_TRANSPOSE(MAL_S3x3_RET_A_by_B(skew(t),body->localR)); 
				Xj_i(k+3,k+3) = Rt(k,k);
			}
	*/
	//Xj=PluckerTransform(matrix3d lR,vector3d lp)
    MAL_S3_VECTOR(,double) t;
	MAL_S3_VECTOR_FILL(t,0);
	Xj_i=Spatial::PluckerTransform(MAL_S3x3_RET_TRANSPOSE(body->localR),t);

    body->sXpii=Xj_i*Xl_i;

	/*	for (unsigned int k=0;k<3;k++)
		{
			X0i_j(k,k)=MAL_RET_TRANSPOSE(MAL_S3x3_RET_A_by_B(RS,body->localR));
			X0i_j(k+3,k)=MAL_RET_TRANSPOSE(MAL_S3x3_RET_A_by_B(SkRS,body->localR));
			X0i_j(k+3,k+3)=MAL_RET_TRANSPOSE(MAL_S3x3_RET_A_by_B(RS,body->localR));
		}
	*/
	matrix3d Rtmp2;
	Rtmp2 = MAL_S3x3_RET_A_by_B(body->R_static,body->localR);
	X0i_j=Spatial::PluckerTransform(MAL_S3x3_RET_TRANSPOSE(Rtmp),body->b);

  if (parentbody!=0)
    {
      Spatial::PluckerTransform piX0 = parentbody->sX0i;
      body->sX0i = body->sXpii*piX0;
    }
  else
	  body->sX0i=X0i_j;
  return true;
}

/* Velocity update */
bool JointPrivate::SupdateVelocity(const vectorN& inRobotConfigVector,
				  const vectorN& inRobotSpeedVector)
{
  std::cout << "in FF Spatial update Vel" << std::endl;

  DynamicBodyPrivate* currentBody = (DynamicBodyPrivate*)(linkedBody());
  DynamicBodyPrivate* currentMotherBody = 0;
  vector3d NE_tmp, NE_tmp2; 
  if (parentJoint()!=0)
    currentMotherBody = (DynamicBodyPrivate*)(parentJoint()->linkedBody());

  /*vectorN localSpeed;
  MAL_VECTOR_RESIZE(localSpeed,numberDof());*/
  MAL_VECTOR_RESIZE(currentBody->sdq, m_nbDofs);
  for(unsigned int i=0;i<m_nbDofs;i++)
  {
	//localSpeed(i) = inRobotSpeedVector(rankInConfiguration()+i);
	currentBody->sdq[i] = inRobotSpeedVector(rankInConfiguration()+i);
  }
    // In the global frame.
  ODEBUG("dq: "<< currentBody->sdq );

  matrix3d RstaticT = MAL_S3x3_RET_TRANSPOSE(currentBody->R_static);    
  // Computes the angular velocity

  //pcalc();

  //vectorN a = MAL_RET_A_by_B(m_phi,localSpeed);
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
  //Spatial::PluckerTransform ssv=spatialskew(currentBody->sv);
  //vectorN b = ssv * a;
  vectorN b = currentBody->sv^a;
  //m_Zeta = MAL_RET_A_by_B(m_dotphi,localSpeed); 
  m_Zeta = MAL_RET_A_by_B(m_dotphi,currentBody->sdq); 
  m_Zeta = m_Zeta + b;
  /*old code by O.S*/
  //m_Zeta = MAL_RET_A_by_B(m_dotphi,localSpeed);
  //m_Zeta = m_Zeta + (m_sv^a);
  return true;
}

bool JointPrivate::SupdateAcceleration(const vectorN& inRobotConfigVector,
				      const vectorN& inRobotSpeedVector,
				      const vectorN& inRobotAccelerationVector)
{
	std::cout << "in FF Spatial update Accel" << std::endl;

	DynamicBodyPrivate* currentBody = (DynamicBodyPrivate*)(linkedBody());
	DynamicBodyPrivate* currentMotherBody = 0;
  
	if (parentJoint()!=0)
    currentMotherBody = (DynamicBodyPrivate*)(parentJoint()->linkedBody());

	/*vectorN localAcc;
	MAL_VECTOR_RESIZE(localAcc,numberDof());*/
	MAL_VECTOR_RESIZE(currentBody->sddq,m_nbDofs);
	for(unsigned int i=0;i<m_nbDofs;i++)
	  /*wrong old expression*/
//    localAcc(i) = inRobotSpeedVector(rankInConfiguration()+i);
	 /*new expression to be verified by L.S */
	{
		//localAcc(i) = inRobotAccelerationVector(rankInConfiguration()+i);
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
	//matrixNxP Ia = currentBody->computeInertia();
	MAL_S3x3_MATRIX(,double) lI = currentBody->getInertie();
	double lmass = currentBody->getMass();
	MAL_S3_VECTOR(,double) lc = currentBody->localCenterOfMass();
	MAL_S3_VECTOR(,double) lh=lc*lmass;
	currentBody->sIa = Spatial::Inertia(lI,lh,lmass);

  // MAL_VECTOR(,double) f_ext = ComputeExtForce();

	//update world com position
	MAL_S3x3_C_eq_A_by_B(m_wlc,currentBody->R,lc);
	currentBody->w_c  = m_wlc + currentBody->p;
	vector3d wc = currentBody->w_c;
	const double gravity_cst = -9.81;
	vector3d g;
	MAL_S3_VECTOR_FILL(g,0);
	g(2) = gravity_cst;
	vector3d lfext,wfext;
	MAL_S3_VECTOR_FILL(lfext,0);
	MAL_S3_VECTOR_FILL(wfext,0);
	lfext = wc^g;
	wfext = g;

	f_ext = Spatial::Force(lfext,wfext);

	//Spatial::PluckerTransform X0i = computeX0(inRobotConfigVector);
	//Spatial::PluckerTransform ssv=spatialskew(currentBody->sv);
	// to develop the transpose of a spatial matrix
	//ssvt = transpose(ssv);
	Spatial::Force sf,sf1,sf2;
	sf = currentBody->sIa*currentBody->sa;
	Spatial::Momentum sh;
	sh = currentBody->sIa*currentBody->sv;
	//sf1 -= ssvt*sh;
	sf1 = currentBody->sv^sh;
	Spatial::PluckerTransform invX0i; 
	//----------> To add this function after fixing the pb. with inverse compilation
	invX0i.inverse(currentBody->sX0i);

	sf2 = f_ext*lmass;
	//----------> To develop the transpose of a spatial matrix
	//sf2 = Spatial::transpose(invX0i)*sf2;
	sf = sf + sf1;
	currentBody->sf = sf + sf2;
  // Update force old code by O.S.
 /* Spatial::Force af;
  af = m_sI * m_sa;
  Spatial::Momentum ah;
  ah = m_sI * m_sv;
  Spatial::Force af2;
  af2 = m_sv^ah;
  af = af + af2;*/
  return true;
}


void JointPrivate::SupdateTorqueAndForce()
{
  DynamicBodyPrivate * currentBody = (DynamicBodyPrivate*)(linkedBody());

  DynamicBodyPrivate* currentMotherBody = 0;
  
	if (parentJoint()!=0)
    currentMotherBody = (DynamicBodyPrivate*)(parentJoint()->linkedBody());
	//----------> To develop pcacl and the operator* that multiplies a matrixNxP by a Spatial::Velocity and gives as output a vectorN
	//pcalc();
    //currentBody->stau = m_phi*currentBody->sf;
	currentBody->m_Torque = currentBody->sf.n0();
    currentBody->m_Force = currentBody->sf.f();

	if (currentMotherBody!=0)
    {
	  Spatial::Force sft; 
	  //----------> To develop the transpose of a spatial matrix
	  //sft = Spatial::transpose(currentBody->sXpii)*currentBody->sf;
      //parentBody->sf = parentBody->sf + sft;
    }

}