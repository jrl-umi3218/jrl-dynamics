/* @doc Object used to handle a foot 

   Copyright (c) 2009, 

   @author : 
   Olivier Stasse.

   JRL-Japan, CNRS/AIST

   All rights reserved.
   
   Please refers to file License.txt for details on the license.

*/
#include "dynamicsJRLJapan/Foot.h"

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

void Foot::getSoleCenterInLocalFrame(vector3d & outCoordinates) const
{
  outCoordinates = m_CenterInFootFrame;
}

void Foot::setSoleCenterInLocalFrame(const vector3d &inCoordinates)
{
  m_CenterInFootFrame = inCoordinates;
}

void Foot::getProjectionCenterLocalFrameInSole(vector3d& outCoordinates) const
{
  outCoordinates = m_ProjectionCenterInSoleFrame;
}

void Foot::setProjectionCenterLocalFrameInSole(const vector3d& inCoordinates)
{
  m_ProjectionCenterInSoleFrame = inCoordinates;
}
