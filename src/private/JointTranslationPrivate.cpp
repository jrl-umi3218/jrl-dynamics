/*
 * Copyright 2010,
 *
 * Francois Keith
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

#include "JointTranslationPrivate.h"
#include "DynamicBodyPrivate.h"


using namespace dynamicsJRLJapan;

JointTranslationPrivate::JointTranslationPrivate()
  :JointPrivate()
{
  setnumberDof(1);
  CreateLimitsArray();
}

JointTranslationPrivate::JointTranslationPrivate(const JointTranslationPrivate &a)
  :JointPrivate(a)
{
}

JointTranslationPrivate::
JointTranslationPrivate(const MAL_S4x4_MATRIX_TYPE(double) &inInitialPosition)
  :JointPrivate()
{
  setnumberDof(1);
  CreateLimitsArray();

  type(JointPrivate::PRISMATIC_JOINT);
  m_inGlobalFrame = true;
  m_globalPoseAtConstruction = inInitialPosition;

  ODEBUG2("translation: inInitialPosition" << inInitialPosition);
  MAL_S3_VECTOR(laxis, double);

  MAL_S3_VECTOR_ACCESS(laxis,0) = 1.0;
  MAL_S3_VECTOR_ACCESS(laxis,1) = 0.0;
  MAL_S3_VECTOR_ACCESS(laxis,2) = 0.0;

  axis(laxis);
}

JointTranslationPrivate::~JointTranslationPrivate()
{}

bool JointTranslationPrivate::updateVelocity(const vectorN&,
					     const vectorN&)
{
  return true;
}

bool JointTranslationPrivate::updateAcceleration(const vectorN&,
						 const vectorN&,
						 const vectorN&)
{
  return true;
}

const matrixNxP & JointTranslationPrivate::pcalc(const vectorN & /*qi*/)
{
  MAL_MATRIX_RESIZE(m_phi,6,1);
  MAL_MATRIX_FILL(m_phi,0);
  m_phi(3,0)=1;
  return m_phi;
}

const matrixNxP & JointTranslationPrivate::pdcalc(const vectorN & /*qi*/)
{
  MAL_MATRIX_RESIZE(m_dotphi,6,1);
  MAL_MATRIX_FILL(m_dotphi,0);
  return m_dotphi;
}
