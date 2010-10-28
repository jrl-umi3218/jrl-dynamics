/*
  Copyright (c) 2010, 
  @author Olivier Stasse, Oussama Kanoun
   
  JRL-Japan, CNRS/AIST
 
  All rights reserved.

  Please refers to file License.txt for details on the license.
   
*/
#include "Debug.h"

#include "JointPrivate.h"
#include "JointFreeFlyerPrivate.h"
#include "DynamicBodyPrivate.h"


using namespace dynamicsJRLJapan;

JointFreeflyerPrivate::JointFreeflyerPrivate()
 :JointPrivate()
{
  m_dof6D.resize(6,false);
  m_nbDofs = 6;
  CreateLimitsArray();
}

JointFreeflyerPrivate::JointFreeflyerPrivate(const MAL_S4x4_MATRIX(,double) &inInitialPosition)
{
  type(JointPrivate::FREE_JOINT);
  m_inGlobalFrame = true;
  m_globalPoseAtConstruction = inInitialPosition;

  ODEBUG2("freeflyer: inInitialPosition" << inInitialPosition);
  MAL_S3_VECTOR(laxis, double);

  MAL_S3_VECTOR_ACCESS(laxis,0) = 1.0;
  MAL_S3_VECTOR_ACCESS(laxis,1) = 0.0;
  MAL_S3_VECTOR_ACCESS(laxis,2) = 0.0;

  axis(laxis);
  m_dof6D.resize(6,false);
   
}

JointFreeflyerPrivate::~JointFreeflyerPrivate()
{
}

bool JointFreeflyerPrivate::updateTransformation(const vectorN & inDofVector)
{
  
  for (unsigned int i=0; i<6; i++)
    m_dof6D(i) = inDofVector(rankInConfiguration() + i);
  
  UpdatePoseFrom6DOFsVector(m_dof6D);
  return true;
}

bool JointFreeflyerPrivate::updateVelocity(const vectorN & inRobotConfigVector,
					   const vectorN & inRobotSpeedVector)
{
  DynamicBodyPrivate* currentBody = (DynamicBodyPrivate*)(linkedBody());
  
  for(unsigned int i=0;i<3;i++)
    {
      currentBody->v0[i] = inRobotSpeedVector(rankInConfiguration()+i);

      currentBody->w[i] = 
	currentBody->lw[i] = inRobotSpeedVector(rankInConfiguration()+i+3);
            
    }
  return true;
}

bool JointFreeflyerPrivate::updateAcceleration(const vectorN & inRobotConfigVector,
					       const vectorN & inRobotSpeedVector,
					       const vectorN & inRobotAccelerationVector)
{
  DynamicBodyPrivate* currentBody = (DynamicBodyPrivate*)(linkedBody());
  
  for(unsigned int i=0;i<3;i++)
    {
      currentBody->dv[i] = 
	currentBody->ldv[i] = inRobotAccelerationVector(rankInConfiguration()+i);
      currentBody->dw[i] = 
	currentBody->ldw[i] = inRobotAccelerationVector(rankInConfiguration()+i+3);
    }
  return true;
}

const matrixNxP & JointFreeflyerPrivate::pcalc(const vectorN & qi)
{
	MAL_S3_VECTOR(qlin,double);
	MAL_S3_VECTOR_FILL(qlin,0);
	MAL_MATRIX_RESIZE(m_phi,6,6);
	//MAL_MATRIX_SET_IDENTITY(m_phi);
	MAL_MATRIX_FILL(m_phi,0);

	DynamicBodyPrivate* CurrentBody = (DynamicBodyPrivate*)(linkedBody());
	MAL_S3x3_MATRIX(,double) currentBodyRt;
	currentBodyRt = MAL_S3x3_RET_TRANSPOSE(CurrentBody->R);
	for (unsigned int i=0;i<3;i++)
	{
		for (unsigned int j=0;j<3;j++)
		{
			m_phi(i,j) = currentBodyRt(i,j);
			m_phi(i+3,j+3) = currentBodyRt(i,j);
		}
	}

	/*
	for (unsigned int i=0;i<3;i++)
	qlin(i)=qi(i);
	MAL_S3x3_MATRIX(,double) Sx = JointPrivate::skew(qlin);
	for (unsigned int i=0;i<3;i++)
	{   
		for (unsigned int j=0;j<3;j++)
		m_phi(i,j+3)=Sx(i,j);
	}
	*/
   return m_phi;

}

const matrixNxP & JointFreeflyerPrivate::pdcalc(const vectorN & qi)
{
	for(unsigned int i=0;i<6;i++)
	{	
		MAL_MATRIX_RESIZE(m_dotphi,6,6);
		MAL_MATRIX_FILL(m_dotphi,0);
	}
  return m_dotphi;

}
 