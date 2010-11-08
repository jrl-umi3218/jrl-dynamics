/*
 * Copyright 2009, 2010,
 *
 * Florent Lamiraux
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

/* @doc Object used to handle a foot
*/
#include "jrl/dynamics/foot.hh"

using namespace dynamicsJRLJapan;

Foot::Foot()
{
}

Foot::Foot(const Foot &inFoot)
{

}

Foot::~Foot()
{
}

const CjrlJoint * Foot::associatedAnkle() const
{
  return m_Ankle;
}

void Foot::setAssociatedAnkle(const CjrlJoint * inAssociatedAnkle)
{
  m_Ankle = inAssociatedAnkle;
}

void Foot::getSoleSize(double &outLength, double &outWidth) const
{
  outLength = m_SoleLength;
  outWidth  = m_SoleWidth;
}

void Foot::setSoleSize(const double &inLength, const double &inWidth)
{
  m_SoleLength = inLength;
  m_SoleWidth = inWidth;
}

void Foot::getAnklePositionInLocalFrame(vector3d& outCoordinates) const
{
  outCoordinates = m_AnklePositionInFootFrame;
}

void Foot::setAnklePositionInLocalFrame(const vector3d& inCoordinates)
{
  m_AnklePositionInFootFrame = inCoordinates;
}
