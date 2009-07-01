/* @doc Object used to handle a foot 

   Copyright (c) 2009, 

   @author : 
   Olivier Stasse.

   JRL-Japan, CNRS/AIST

   All rights reserved.
   
   Please refers to file License.txt for details on the license.

*/
#include "Foot.h"

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

CjrlJoint * Foot::associatedAnkle()
{
  return m_Ankle;
}

void Foot::setAssociatedAnkle(CjrlJoint * inAssociatedAnkle)
{
  m_Ankle = inAssociatedAnkle;
}

void Foot::soleSize(double &outLength, double &outWidth)
{
  outLength = m_SoleLength;
  outWidth  = m_SoleWidth;
}

void Foot::setSoleSize(double &inLength, double &inWidth)
{
  m_SoleLength = inLength;
  m_SoleWidth = inWidth;
}

void Foot::anklePositionInLocalFrame(vector3d& outCoordinates)
{
  outCoordinates = m_AnklePositionInFootFrame;
}

void Foot::setAnklePositionInLocalFrame(vector3d& inCoordinates)
{
  m_AnklePositionInFootFrame = inCoordinates;
}

void Foot::soleCenterInLocalFrame(vector3d & outCoordinates)
{
  outCoordinates = m_CenterInFootFrame;
}

void Foot::setSoleCenterInLocalFrame(vector3d &inCoordinates)
{
  m_CenterInFootFrame = inCoordinates;
}

void Foot::projectionCenterLocalFrameInSole(vector3d& outCoordinates)
{
  outCoordinates = m_ProjectionCenterInSoleFrame;
}

void Foot::setProjectionCenterLocalFrameInSole(vector3d& inCoordinates)
{
  m_ProjectionCenterInSoleFrame = inCoordinates;
}
