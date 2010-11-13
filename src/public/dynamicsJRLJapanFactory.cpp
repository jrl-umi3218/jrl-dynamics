/*
 * Copyright 2010,
 *
 * Francois Keith,
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
#include "dynamicsJRLJapan.h"
#include "jrl/dynamics/dynamicsfactory.hh"
#include "abstract-robot-dynamics/robot-dynamics-object-constructor.hh"
#include "../private/HumDynMultiBodyPrivate.h"
#include "jrl/dynamics/humanoiddynamicrobot.hh"
#include "jrl/dynamics/hand.hh"
#include "jrl/dynamics/foot.hh"
#include "../private/JointFreeFlyerPrivate.h"
#include "../private/JointRotationPrivate.h"
#include "../private/JointTranslationPrivate.h"
#include "../private/JointAnchorPrivate.h"

namespace dynamicsJRLJapan
{

  CjrlHumanoidDynamicRobot * ObjectFactory::createHumanoidDynamicRobot()
  {
    CjrlHumanoidDynamicRobot * aHDR = new HumDynMultiBodyPrivate();
    return aHDR;
  }

  CjrlDynamicRobot * ObjectFactory::createDynamicRobot()
  {
    CjrlDynamicRobot * aDR = new DynMultiBodyPrivate();
    return aDR;
  }

  CjrlJoint * ObjectFactory::createJointFreeflyer(const matrix4d& inInitialPosition)
  {
    CjrlJoint * aHDR = new JointFreeflyerPrivate(inInitialPosition);
    return aHDR;
  }

  CjrlJoint * ObjectFactory::createJointRotation(const matrix4d& inInitialPosition)
  {
    CjrlJoint * aHDR = new JointRotationPrivate(inInitialPosition);
    return aHDR;
  }

  CjrlJoint * ObjectFactory::createJointAnchor(const matrix4d& inInitialPosition)
  {
    CjrlJoint * aHDR = new JointAnchorPrivate(inInitialPosition);
    return aHDR;
  }

  CjrlJoint * ObjectFactory::createJointTranslation(const matrix4d& inInitialPosition)
  {
    CjrlJoint * aHDR = new JointTranslationPrivate(inInitialPosition);
    return aHDR;
  }

  CjrlBody * ObjectFactory::createBody()
  {
    CjrlBody * aHDR = new DynamicBody();
    return aHDR;
  }

  CjrlHand* ObjectFactory::createHand(const CjrlJoint* inWrist)
  {
    CjrlHand* hand = new Hand();
    hand->setAssociatedWrist(inWrist);
    return hand;
  }

  CjrlFoot* ObjectFactory::createFoot(const CjrlJoint* inAnkle)
  {
    Foot* foot = new Foot();
    foot->setAssociatedAnkle(inAnkle);
    return foot;
  }


  int parseOpenHRPVRMLFile(CjrlHumanoidDynamicRobot &ajrlHumanoidDynamicRobot,
			   std::string &OpenHRPVRMLFile,
			   std::string &MapJointToRankFileName,
			   std::string &SpecificitiesFileName)
  {
    std::vector<BodyGeometricalData> lVectorOfURLs;
    return parseOpenHRPVRMLFile(ajrlHumanoidDynamicRobot,
				OpenHRPVRMLFile,
				MapJointToRankFileName,
				SpecificitiesFileName,
				lVectorOfURLs);

  }
  int parseOpenHRPVRMLFile(CjrlHumanoidDynamicRobot &ajrlHumanoidDynamicRobot,
			   std::string &OpenHRPVRMLFile,
			   std::string &MapJointToRankFileName,
			   std::string &SpecificitiesFileName,
			   std::vector<BodyGeometricalData> &VectorOfURLs,
			   bool ReadGeometricalInformation)
  {
    dynamicsJRLJapan::HumDynMultiBodyPrivate *aHDR =
      dynamic_cast<HumDynMultiBodyPrivate *>(&ajrlHumanoidDynamicRobot);

    // TO DO : Not be limited to dynamicsJRLJapan implementation.
    // but right now it is easier.
    // The following is purely heretic and will disappear sooner or later.
    if (aHDR!=0)
      {
	aHDR->parserVRML(OpenHRPVRMLFile, 
			 MapJointToRankFileName.c_str(),
			 VectorOfURLs,
			 ReadGeometricalInformation);
	aHDR->SetHumanoidSpecificitiesFile(SpecificitiesFileName);
	return 0;
      }
    else
      {
	jrlDelegate::humanoidDynamicRobot *a3HDR =
	  dynamic_cast<jrlDelegate::humanoidDynamicRobot *>(&ajrlHumanoidDynamicRobot);
	if (a3HDR!=0)
	  {
	    dynamicsJRLJapan::HumDynMultiBodyPrivate *a4HDR =
	      dynamic_cast<HumDynMultiBodyPrivate *>(a3HDR->m_HDR);

	    if (a4HDR!=0)
	      {
		a4HDR->parserVRML(OpenHRPVRMLFile,
				  MapJointToRankFileName.c_str(),
				  VectorOfURLs,
				  ReadGeometricalInformation);
		a4HDR->SetHumanoidSpecificitiesFile(SpecificitiesFileName);
		return 0;
	      }

	  }
      }

    return -1;
  }
}

