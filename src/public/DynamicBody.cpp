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
  DynamicBodyPrivate* obj = new DynamicBodyPrivate();
  m_privateObj = boost::shared_ptr<DynamicBodyPrivate>(obj);
}

DynamicBody::DynamicBody(const DynamicBody& inBody)
{
  DynamicBodyPrivate* obj = new DynamicBodyPrivate(*inBody.m_privateObj);
  m_privateObj = boost::shared_ptr<DynamicBodyPrivate>(obj);
}

const vector3d& DynamicBody::localCenterOfMass() const
{
  return m_privateObj->localCenterOfMass();
}

void DynamicBody::localCenterOfMass(const vector3d& inlocalCenterOfMass)
{
  m_privateObj->localCenterOfMass(inlocalCenterOfMass);
}

const matrix3d& DynamicBody::inertiaMatrix() const
{
  return m_privateObj->inertiaMatrix();
}

void DynamicBody::inertiaMatrix(const matrix3d& inInertiaMatrix)
{
  m_privateObj->inertiaMatrix(inInertiaMatrix);
}

double DynamicBody::mass() const
{
  return m_privateObj->mass();
}

void DynamicBody::mass(double inMass)
{
  m_privateObj->mass(inMass);
}

const CjrlJoint* DynamicBody::joint() const
{
  return m_privateObj->joint();
}

