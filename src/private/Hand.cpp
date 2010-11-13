/*
 * Copyright 2009, 2010,
 *
 * Oussama Kannoun
 * Florent Lamiraux
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
/* @doc Object used to handle hand
*/
#include "jrl/dynamics/hand.hh"

using namespace dynamicsJRLJapan;

Hand::Hand()
{
  attAssociatedWrist = 0;
}

void Hand::setAssociatedWrist(const CjrlJoint * inWristJoint)
{
  attAssociatedWrist=inWristJoint;
}

Hand::~Hand()
{
}

const CjrlJoint* Hand::associatedWrist() const
{
    return attAssociatedWrist;
}

void Hand::getCenter(vector3d& outCenter) const
{
  outCenter = attCenter;
}

void Hand::setCenter(const vector3d& inCenter)
{
  attCenter = inCenter;
}

void Hand::getThumbAxis(vector3d& outThumbAxis) const
{
  outThumbAxis = attOkayAxis;
}

void Hand::setThumbAxis(const vector3d& inThumbAxis)
{
  attOkayAxis = inThumbAxis;
}

void Hand::getForeFingerAxis(vector3d& outForeFingerAxis) const
{
  outForeFingerAxis = attShowingAxis;
}

void Hand::setForeFingerAxis(const vector3d& inForeFingerAxis)
{
  attShowingAxis = inForeFingerAxis;
}

void Hand::getPalmNormal(vector3d& outPalmNormal) const
{
  outPalmNormal = attPalmAxis;
}

void Hand::setPalmNormal(const vector3d& inPalmNormal)
{
  attPalmAxis = inPalmNormal;
}

