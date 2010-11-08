/*
 * Copyright 2006, 2007, 2008, 2009, 2010,
 *
 * Fumio Kanehiro,
 * Florent Lamiraux,
 * Olivier Stasse,
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

#include "jrl/mal/matrixabstractlayer.hh"
#include "jrl/dynamics/joint.hh"
#include "../private/JointPrivate.h"
#include "../private/JointAnchorPrivate.h"
#include "../private/JointFreeFlyerPrivate.h"
#include "../private/JointRotationPrivate.h"
#include "../private/JointTranslationPrivate.h"

using namespace dynamicsJRLJapan;

Joint::Joint()
{
  CjrlJoint* obj = NULL;
  m_privateObj = boost::shared_ptr<CjrlJoint>(obj);
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
		m_privateObj = boost::shared_ptr<CjrlJoint>(obj);
	      else
		{
		  m_privateObj = boost::shared_ptr<CjrlJoint>(obj);
		  std::cerr<< "Type not recognized";
		}
	    }
	}
    }
}

JointFreeflyer::JointFreeflyer(const matrix4d& inInitialPosition)
{
  CjrlJoint* obj = new JointFreeflyerPrivate(inInitialPosition);
  m_privateObj = boost::shared_ptr<CjrlJoint>(obj);
}

JointRotation::JointRotation(const matrix4d& inInitialPosition)
{
  CjrlJoint* obj = new JointRotationPrivate(inInitialPosition);
  m_privateObj = boost::shared_ptr<CjrlJoint>(obj);
}

JointTranslation::JointTranslation(const matrix4d& inInitialPosition)
{
  CjrlJoint* obj = new JointTranslationPrivate(inInitialPosition);
  m_privateObj = boost::shared_ptr<CjrlJoint>(obj);
}

JointAnchor::JointAnchor(const matrix4d& inInitialPosition)
{
  CjrlJoint* obj = new JointAnchorPrivate(inInitialPosition);
  m_privateObj = boost::shared_ptr<CjrlJoint>(obj);
}


#define DERIVPRIVATE dynamic_cast<JointPrivate *>(m_privateObj.get())

CjrlJoint* Joint::parentJoint() const
{
  return DERIVPRIVATE->parentJoint();
}


bool Joint::addChildJoint (CjrlJoint& inJoint)
{
  return DERIVPRIVATE->addChildJoint(inJoint);
}


unsigned int Joint::countChildJoints() const
{
  return DERIVPRIVATE->countChildJoints();
}


CjrlJoint* Joint::childJoint(unsigned int inJointRank) const
{
  return DERIVPRIVATE->childJoint(inJointRank);
}


std::vector<CjrlJoint*> Joint::jointsFromRootToThis() const
{
  return DERIVPRIVATE->jointsFromRootToThis();
}


unsigned int Joint::rankInConfiguration() const
{
  return DERIVPRIVATE->rankInConfiguration();
}


const matrix4d& Joint::initialPosition()
{
  return DERIVPRIVATE->initialPosition();
}


bool Joint::updateTransformation(const vectorN& inDofVector)
{
  return DERIVPRIVATE->updateTransformation(inDofVector);
}


const matrix4d &Joint::currentTransformation() const
{
  return DERIVPRIVATE->currentTransformation();
}


CjrlRigidVelocity Joint::jointVelocity()
{
  return DERIVPRIVATE->jointVelocity();
}


CjrlRigidAcceleration Joint::jointAcceleration()
{
  return DERIVPRIVATE->jointAcceleration();
}


unsigned int Joint::numberDof() const
{
  return DERIVPRIVATE->numberDof();
}


double Joint::lowerBound(unsigned int inDofRank) const
{
  return DERIVPRIVATE->lowerBound(inDofRank);
}


double Joint::upperBound(unsigned int inDofRank) const
{
  return DERIVPRIVATE->upperBound(inDofRank);
}


void Joint::lowerBound(unsigned int inDofRank, double inLowerBound)
{
  DERIVPRIVATE->lowerBound(inDofRank, inLowerBound);
}

void Joint::upperBound(unsigned int inDofRank, double inUpperBound)
{
  DERIVPRIVATE->upperBound(inDofRank, inUpperBound);
}


double Joint::lowerVelocityBound(unsigned int inDofRank) const
{
  return DERIVPRIVATE->lowerVelocityBound(inDofRank);
}

double Joint::upperVelocityBound(unsigned int inDofRank) const
{
  return DERIVPRIVATE->upperVelocityBound(inDofRank);
}

void Joint::lowerVelocityBound(unsigned int inDofRank, double inLowerBound)
{
  DERIVPRIVATE->lowerVelocityBound(inDofRank, inLowerBound);
}

void Joint::upperVelocityBound(unsigned int inDofRank, double inUpperBound)
{
  DERIVPRIVATE->upperVelocityBound(inDofRank, inUpperBound);
}

const matrixNxP& Joint::jacobianJointWrtConfig() const
{
  return DERIVPRIVATE->jacobianJointWrtConfig();
}

void Joint::computeJacobianJointWrtConfig()
{
  DERIVPRIVATE->computeJacobianJointWrtConfig();
}

void Joint::getJacobianPointWrtConfig(const vector3d& inPointJointFrame,
				      matrixNxP& outjacobian) const
{
  DERIVPRIVATE->getJacobianPointWrtConfig(inPointJointFrame, outjacobian);
}

CjrlBody* Joint::linkedBody() const
{
  return DERIVPRIVATE->linkedBody();
}

void Joint::setLinkedBody (CjrlBody& inBody)
{
  DERIVPRIVATE->setLinkedBody(inBody);
}


