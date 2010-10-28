/* \file This part implements the generic JRL interface 
  Copyright (c) 2009, 
  @author Olivier Stasse
   
  JRL-Japan, CNRS/AIST
 
  All rights reserved.

  Please refers to file License.txt for details on the license.
   
*/

#include "Debug.h"

#include "JointPrivate.h"
#include "DynamicBodyPrivate.h"

using namespace dynamicsJRLJapan;

CjrlJoint* JointPrivate::parentJoint() const
{
  return m_FatherJoint;
}

bool JointPrivate::addChildJoint(CjrlJoint& aJoint)
{
  JointPrivate * pjoint = (JointPrivate *)&aJoint;
  for(unsigned int li =0; li < m_Children.size();li++)
    {
      if (m_Children[li]==pjoint)
	return true;
    }
  // Make sure I went through this part.
  ODEBUG("Set father joint : " << pjoint->getName()
	 << " "  << getName());
  pjoint->SetFatherJoint(this);
  m_Children.push_back(pjoint);
  return true;
}

unsigned int JointPrivate::countChildJoints() const
{
  return m_Children.size();
}

CjrlJoint* JointPrivate::childJoint(unsigned int givenRank) const
{
  if (givenRank<m_Children.size())
    return m_Children[givenRank];

  return 0;
}

JointPrivate* JointPrivate::child_JointPrivate(unsigned int givenRank) const
{
  if (givenRank<m_Children.size())
    return m_Children[givenRank];

  return 0;
}

std::vector<CjrlJoint*> JointPrivate::jointsFromRootToThis() const
{
  return m_FromRootToThis;
}


const MAL_S4x4_MATRIX(,double) & JointPrivate::currentTransformation() const
{
  DynamicBodyPrivate *m_DBody = (DynamicBodyPrivate *) m_Body;
  if (m_DBody==0)
    return m_globalPoseAtConstruction;
  return m_DBody->m_transformation;
}

CjrlRigidVelocity JointPrivate::jointVelocity()
{

  DynamicBodyPrivate *m_DBody = dynamic_cast<DynamicBodyPrivate *>(m_Body);
  CjrlRigidVelocity ajrlRV(m_DBody->v0,m_DBody->w);
  return ajrlRV;
}

void JointPrivate::computeSubTreeMCom()
{

  for (unsigned int Id = 0; Id< 3;Id++)
    m_STmcom[Id] = linkedDBody()->massCoef()*linkedDBody()->w_c[Id];

  m_STcoef = linkedDBody()->massCoef();

  for (unsigned int Id = 0; Id< countChildJoints();Id++)
    {
      m_Children[Id]->computeSubTreeMCom();
      m_STmcom += m_Children[Id]->subTreeMCom();
      m_STcoef += m_Children[Id]->subTreeCoef();
    }
}

void JointPrivate::computeSubTreeMComExceptChild(const CjrlJoint* inJoint)
{
  for (unsigned int Id = 0; Id< 3;Id++)
    m_STmcom[Id] = linkedDBody()->massCoef()*linkedDBody()->w_c[Id];

  m_STcoef = linkedDBody()->massCoef();

  for (unsigned int Id = 0; Id< countChildJoints();Id++)
    {
      if (inJoint == m_Children[Id])
	continue;
      m_Children[Id]->computeSubTreeMCom();
      m_STmcom += m_Children[Id]->subTreeMCom();
      m_STcoef += m_Children[Id]->subTreeCoef();
    }
}

void JointPrivate::subTreeMCom(const vector3d& inReplacement)
{
  m_STmcom = inReplacement;
}

const vector3d& JointPrivate::subTreeMCom() const
{
  return m_STmcom;
}

double JointPrivate::subTreeCoef()
{
  return m_STcoef;
}

void JointPrivate::subTreeCoef(double inReplacement)
{
  m_STcoef = inReplacement;
}

CjrlRigidAcceleration JointPrivate::jointAcceleration()
{
  MAL_S3_VECTOR(,double) a,b;

  if (m_Body!=0)
    {
      DynamicBodyPrivate *m_DBody = dynamic_cast<DynamicBodyPrivate *>(m_Body);

      a = m_DBody->dv;
      b = m_DBody->dw;
    }
  CjrlRigidAcceleration ajrlRA(a,b);

  return ajrlRA;

}


const MAL_MATRIX(,double) & JointPrivate::jacobianJointWrtConfig() const
{
  return m_J;
}

void JointPrivate::resizeJacobianJointWrtConfig(int lNbDofs)
{
  MAL_MATRIX_RESIZE(m_J,6,lNbDofs);
  MAL_MATRIX_FILL(m_J,0.0);

}


void JointPrivate::computeJacobianJointWrtConfig()
{
  DynamicBodyPrivate * FinalBody = (DynamicBodyPrivate *)m_Body;
  getJacobianWorldPointWrtConfig(FinalBody->p, m_J);
}

void JointPrivate::getJacobianWorldPointWrtConfig(const vector3d& inPointWorldFrame,
						  matrixNxP& outJ) const
{
  vector3d dp,lv;
  
  ODEBUG("Size of the jacobian :" << m_FromRootToThis.size()-1);
  
  for(unsigned int i=0;i<m_FromRootToThis.size();i++)
    {
      MAL_VECTOR_DIM(LinearAndAngularVelocity,double,6);

      DynamicBodyPrivate * aBody= static_cast<DynamicBodyPrivate *>
	(m_FromRootToThis[i]->linkedBody());
      JointPrivate * aJoint = static_cast<JointPrivate *>(m_FromRootToThis[i]);
      
      unsigned int lcol = aJoint->stateVectorPosition();
      ODEBUG("JointPrivate: " << aJoint->getName() << " " << lcol);
      dp = inPointWorldFrame - aBody->p;
	  /*
	  if (i==0)
	  {
		  std::cout << "JointPrivate: " << aJoint->getName() << " " << lcol << std::endl;
		  std::cout << "leg_jj_pos = " << inPointWorldFrame << std::endl;
		  std::cout << "waist_jj_pos = " << aBody->p << std::endl;
		  std::cout << "dp = " << dp << std::endl;
	  }
	  */
      MAL_S3_VECTOR_CROSS_PRODUCT(lv,aBody->w_a,dp);
	 // std::cout << "JointPrivate: " << aJoint->getName() << std::endl;
	 // std::cout << "joint world axis : " << aBody->w_a << std::endl;
      switch (aJoint->type())
        {
	  
        case JointPrivate::REVOLUTE_JOINT:
	  for(int j=0;j<3;j++)
            {
	      outJ(j,lcol) =  lv[j];
	      outJ(j+3,lcol) = aBody->w_a[j];
            }
	  break;
        case JointPrivate::PRISMATIC_JOINT:
	  for(int j=0;j<3;j++)
            {
	      outJ(j,lcol) =  aBody->w_a[j];
	      outJ(j+3,lcol) = 0;
            }
	  break;
        case JointPrivate::FREE_JOINT:
	  //J =  I M = J11 J12
	  //     0 I   J21 J22
	  //
	  // with M = d(w x dp)/dw
	  //
	  for(int j=0;j<3;j++)
            {
	      for(int k=0;k<3;k++)
                {
		  // Computation of J11, J12 and J21
		  if (j!=k)
                    {
		      outJ(     j, lcol + k) =0.0;
		      outJ( j + 3, lcol + k + 3) = 0.0;
                    }
		  else
                    {
		      outJ(     j, lcol + k ) =1.0;
		      outJ( j + 3, lcol + k + 3) = 1.0;
                    }
		  outJ(j+3,k) = 0.0;
                }
            }
	  // Compute M
	  outJ( 0, lcol + 3 ) =      0;
	  outJ( 0 , lcol + 4 ) =   dp(2);
	  outJ( 0 , lcol + 5 ) = -dp(1);
	  outJ( 1, lcol + 3 ) = -dp(2);
	  outJ( 1 , lcol + 4 ) =      0 ;
	  outJ( 1 , lcol + 5 ) =  dp(0);
	  outJ( 2, lcol + 3 ) =  dp(1);
	  outJ( 2 , lcol + 4 ) =  -dp(0);
	  outJ( 2 , lcol + 5 ) =      0;
	  break;
        }
    }
}

/**
   \brief Get the jacobian of the point specified in local frame by inPointJointFrame.
   The output matrix outjacobian is automatically resized if necessary
 
*/
void JointPrivate::getJacobianPointWrtConfig(const vector3d& inPointJointFrame, matrixNxP& outJ) const
{
  if (outJ.size1() !=6 || outJ.size2() != m_J.size2())
    {
      outJ.resize(6,m_J.size2(),false);
    }
  outJ.clear();


  DynamicBodyPrivate * FinalBody = (DynamicBodyPrivate *)m_Body;

  vector3d pn = FinalBody->p + MAL_S3x3_RET_A_by_B(FinalBody->R, inPointJointFrame);
  getJacobianWorldPointWrtConfig(pn, outJ);
}


CjrlBody* JointPrivate::linkedBody() const
{
  return m_Body;
}

DynamicBodyPrivate* JointPrivate::linkedDBody() const
{
  return m_dynBody;
}

void JointPrivate::setLinkedBody(CjrlBody& inBody)
{
  m_Body = &inBody;
  m_dynBody = (DynamicBodyPrivate*)m_Body;
  
}

void JointPrivate::SetFatherJoint(JointPrivate *aFather)
{
  m_FatherJoint = aFather;

  m_FromRootToThis.clear();

  m_FromRootToThis.push_back(this);

  CjrlJoint* aJoint = m_FatherJoint;
  while(aJoint!=0)
    {
      m_FromRootToThis.insert(m_FromRootToThis.begin(),aJoint);
      aJoint = aJoint->parentJoint();
    }
}

const MAL_S4x4_MATRIX(,double) & JointPrivate::initialPosition()
{
  return m_globalPoseAtConstructionNormalized;
}
