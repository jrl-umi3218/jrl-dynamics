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
#include "dynamicsJRLJapan/DynamicBody.h"
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

