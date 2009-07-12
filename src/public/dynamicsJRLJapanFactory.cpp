/*
 *   Copyright (c) 2006, 2007, 2008, 2009 CNRS-AIST 
 *
 *   Research carried out within the scope of the Associated
 *   International Laboratory: Joint Japanese-French Robotics
 *   Laboratory (JRL)
 *
 *   Author: Olivier Stasse
 *
 *   Please refers to file License.txt for details on the license.
 *
 */

#include "MatrixAbstractLayer/MatrixAbstractLayer.h"
#include "dynamicsJRLJapan.h"
#include "dynamicsJRLJapan/dynamicsJRLJapanFactory.h"
#include "robotDynamics/jrlRobotDynamicsObjectConstructor.h"
#include "../private/HumDynMultiBodyPrivate.h"
#include "dynamicsJRLJapan/humanoidDynamicRobot.h"

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
    CjrlJoint * aHDR = new JointFreeflyer(inInitialPosition);
    return aHDR;
  }

  CjrlJoint * ObjectFactory::createJointRotation(const matrix4d& inInitialPosition)
  {
    CjrlJoint * aHDR = new JointRotation(inInitialPosition);
    return aHDR;
  }

  CjrlJoint * ObjectFactory::createJointTranslation(const matrix4d& inInitialPosition)
  {
    CjrlJoint * aHDR = new JointTranslation(inInitialPosition);
    return aHDR;
  }

  CjrlBody * ObjectFactory::createBody()
  {
    CjrlBody * aHDR = new Body();
    return aHDR;
  }


  int parseOpenHRPVRMLFile(CjrlHumanoidDynamicRobot &ajrlHumanoidDynamicRobot,
			   std::string &OpenHRPVRMLFile,
			   std::string &MapJointToRankFileName,
			   std::string &SpecificitiesFileName)
  {
    dynamicsJRLJapan::HumDynMultiBodyPrivate *aHDR = 
      dynamic_cast<HumDynMultiBodyPrivate *>(&ajrlHumanoidDynamicRobot);

    // TO DO : Not be limited to dynamicsJRLJapan implementation.
    // but right now it is easier.
    // The following is purely heretic and will disappear sooner or later.
    if (aHDR!=0)
      {
	aHDR->parserVRML(OpenHRPVRMLFile,
				       (char *)MapJointToRankFileName.c_str());
	aHDR->SetHumanoidSpecificitiesFile(SpecificitiesFileName);
	return 0;
      }
    else
      {
	CjrlHumanoidDynamicRobotNA *a3HDR = 
	  dynamic_cast<CjrlHumanoidDynamicRobotNA *>(&ajrlHumanoidDynamicRobot);
	if (a3HDR!=0)
	  {
	    dynamicsJRLJapan::HumDynMultiBodyPrivate *a4HDR = 
	      dynamic_cast<HumDynMultiBodyPrivate *>(a3HDR->m_HDR);
	    
	    if (a4HDR!=0)
	      {
		a4HDR->parserVRML(OpenHRPVRMLFile,
				  (char *)MapJointToRankFileName.c_str());
		a4HDR->SetHumanoidSpecificitiesFile(SpecificitiesFileName);
		return 0;
	      }
	    
	  }
      }
    
    return -1;
  }
};

