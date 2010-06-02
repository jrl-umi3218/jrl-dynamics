/*
 *   Copyright (c) 2006, 2007, 2008, 2009 CNRS-AIST 
 *
 *   Research carried out within the scope of the Associated
 *   International Laboratory: Joint Japanese-French Robotics
 *   Laboratory (JRL)
 *
 *   Author: Olivier Stasse and Florent Lamiraux
 *
 *   Please refers to file License.txt for details on the license.
 *
 */

#include "MatrixAbstractLayer/MatrixAbstractLayer.h"
#include "dynamicsJRLJapan/Joint.h"
#include "../private/JointPrivate.h"
#include "../private/JointAnchorPrivate.h"
#include "../private/JointFreeFlyerPrivate.h"
#include "../private/JointRotationPrivate.h"
#include "../private/JointTranslationPrivate.h"

using namespace dynamicsJRLJapan;

Joint::Joint()
{
  JointPrivate* obj = NULL;
  m_privateObj = boost::shared_ptr<JointPrivate>(obj);
}

Joint::Joint(const Joint& inJoint)
{
  JointPrivate* obj=0;
  
  JointFreeflyerPrivate *aFFP = dynamic_cast<JointFreeflyerPrivate *>(inJoint.m_privateObj.get());
  if (aFFP!=0)
    obj = new JointFreeflyerPrivate(*aFFP);
  else
    {
      JointRotationPrivate *aRP = dynamic_cast<JointRotationPrivate *>(inJoint.m_privateObj.get());
      if (aRP!=0)
	obj = new JointRotationPrivate(*aRP);
      else
	{
	  
	  JointTranslationPrivate *aTP = dynamic_cast<JointTranslationPrivate *>(inJoint.m_privateObj.get());
	  if (aTP!=0)
	    obj = new JointTranslationPrivate(*aTP);
	  else
	    {
	      JointAnchorPrivate *aAP = dynamic_cast<JointAnchorPrivate *>(inJoint.m_privateObj.get());
	      if (aAP!=0)
		obj = new JointAnchorPrivate(*aAP);
	      if (obj!=0)
		m_privateObj = boost::shared_ptr<JointPrivate>(obj);
	      else
		{
		  m_privateObj = boost::shared_ptr<JointPrivate>(obj);
		  std::cerr<< "Type not recognized";
		}
	    }
	}
    }
}

JointFreeflyer::JointFreeflyer(const matrix4d& inInitialPosition)
{
  JointPrivate* obj = new JointFreeflyerPrivate(inInitialPosition);
  m_privateObj = boost::shared_ptr<JointPrivate>(obj);
}

JointRotation::JointRotation(const matrix4d& inInitialPosition)
{
  JointPrivate* obj = new JointRotationPrivate(inInitialPosition);
  m_privateObj = boost::shared_ptr<JointPrivate>(obj);
}

JointTranslation::JointTranslation(const matrix4d& inInitialPosition)
{
  JointPrivate* obj = new JointTranslationPrivate(inInitialPosition);
  m_privateObj = boost::shared_ptr<JointPrivate>(obj);
}

JointAnchor::JointAnchor(const matrix4d& inInitialPosition)
{
  JointPrivate* obj = new JointAnchorPrivate(inInitialPosition);
  m_privateObj = boost::shared_ptr<JointPrivate>(obj);
}


CjrlJoint* Joint::parentJoint() const
{
  return m_privateObj->parentJoint();
}


bool Joint::addChildJoint (CjrlJoint& inJoint)
{
  return m_privateObj->addChildJoint(inJoint);
}


unsigned int Joint::countChildJoints() const
{
  return m_privateObj->countChildJoints();
}


CjrlJoint* Joint::childJoint(unsigned int inJointRank) const
{
  return m_privateObj->childJoint(inJointRank);
}


std::vector<CjrlJoint*> Joint::jointsFromRootToThis() const
{
  return m_privateObj->jointsFromRootToThis();
}


unsigned int Joint::rankInConfiguration() const
{
  return m_privateObj->rankInConfiguration();
}


const matrix4d& Joint::initialPosition()
{
  return m_privateObj->initialPosition();
}


bool Joint::updateTransformation(const vectorN& inDofVector)
{
  return m_privateObj->updateTransformation(inDofVector);
}


const matrix4d &Joint::currentTransformation() const
{
  return m_privateObj->currentTransformation();
}


CjrlRigidVelocity Joint::jointVelocity()
{
  return m_privateObj->jointVelocity();
}


CjrlRigidAcceleration Joint::jointAcceleration()
{
  return m_privateObj->jointAcceleration();
}


unsigned int Joint::numberDof() const
{
  return m_privateObj->numberDof();
}


double Joint::lowerBound(unsigned int inDofRank) const
{
  return m_privateObj->lowerBound(inDofRank);
}


double Joint::upperBound(unsigned int inDofRank) const
{
  return m_privateObj->upperBound(inDofRank);
}


void Joint::lowerBound(unsigned int inDofRank, double inLowerBound)
{
  m_privateObj->lowerBound(inDofRank, inLowerBound);
}

void Joint::upperBound(unsigned int inDofRank, double inUpperBound)
{
  m_privateObj->upperBound(inDofRank, inUpperBound);
}


double Joint::lowerVelocityBound(unsigned int inDofRank) const
{
  return m_privateObj->lowerVelocityBound(inDofRank);
}

double Joint::upperVelocityBound(unsigned int inDofRank) const
{
  return m_privateObj->upperVelocityBound(inDofRank);
}

void Joint::lowerVelocityBound(unsigned int inDofRank, double inLowerBound)
{
  m_privateObj->lowerVelocityBound(inDofRank, inLowerBound);
}

void Joint::upperVelocityBound(unsigned int inDofRank, double inUpperBound)
{
  m_privateObj->upperVelocityBound(inDofRank, inUpperBound);
}

const matrixNxP& Joint::jacobianJointWrtConfig() const
{
  return m_privateObj->jacobianJointWrtConfig();
}

void Joint::computeJacobianJointWrtConfig()
{
  m_privateObj->computeJacobianJointWrtConfig();
}

void Joint::getJacobianPointWrtConfig(const vector3d& inPointJointFrame, 
				      matrixNxP& outjacobian) const
{
  m_privateObj->getJacobianPointWrtConfig(inPointJointFrame, outjacobian);
}

CjrlBody* Joint::linkedBody() const
{
  return m_privateObj->linkedBody();
}

void Joint::setLinkedBody (CjrlBody& inBody)
{
  m_privateObj->setLinkedBody(inBody);
}


