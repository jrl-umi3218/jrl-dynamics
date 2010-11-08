/*
 * Copyright 2006, 2007, 2008, 2009, 2010,
 *
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
#include "jrl/dynamics/dynamicbody.hh"
#include "../private/DynamicBodyPrivate.h"

using namespace dynamicsJRLJapan;

DynamicBody::DynamicBody()
{
  CjrlBody* obj = new DynamicBodyPrivate();
  m_privateObj = boost::shared_ptr<CjrlBody>(obj);
}

DynamicBody::DynamicBody(const DynamicBody& inBody)
{
  DynamicBodyPrivate *aDBP = dynamic_cast<DynamicBodyPrivate *>(inBody.m_privateObj.get());
  CjrlBody* obj = 0;
  if (aDBP!=0)
    obj= new DynamicBodyPrivate(*aDBP);
  m_privateObj = boost::shared_ptr<CjrlBody>(obj);
}

#define DERIVPRIVATE dynamic_cast<DynamicBodyPrivate *>(m_privateObj.get())

const vector3d& DynamicBody::localCenterOfMass() const
{
  return DERIVPRIVATE->localCenterOfMass();
}

void DynamicBody::localCenterOfMass(const vector3d& inlocalCenterOfMass)
{
  DERIVPRIVATE->localCenterOfMass(inlocalCenterOfMass);
}

const matrix3d& DynamicBody::inertiaMatrix() const
{
  return DERIVPRIVATE->inertiaMatrix();
}

void DynamicBody::inertiaMatrix(const matrix3d& inInertiaMatrix)
{
  DERIVPRIVATE->inertiaMatrix(inInertiaMatrix);
}

double DynamicBody::mass() const
{
  return DERIVPRIVATE->mass();
}

void DynamicBody::mass(double inMass)
{
  DERIVPRIVATE->mass(inMass);
}

const CjrlJoint* DynamicBody::joint() const
{
  return DERIVPRIVATE->joint();
}

