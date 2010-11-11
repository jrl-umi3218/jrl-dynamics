/*
 * Copyright 2010,
 *
 * Francois Keith
 * Olivier Stasse
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

JointFreeflyerPrivate::JointFreeflyerPrivate(const JointFreeflyerPrivate &a)
 :JointPrivate(a)
{
  m_dof6D.resize(6,false);
}

JointFreeflyerPrivate::JointFreeflyerPrivate(const MAL_S4x4_MATRIX_TYPE(double) &inInitialPosition)
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

bool JointFreeflyerPrivate::updateVelocity(const vectorN &,
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

bool JointFreeflyerPrivate::updateAcceleration(const vectorN & ,
					       const vectorN & ,
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

