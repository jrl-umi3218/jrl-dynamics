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

namespace dynamicsJRLJapan
{

 CjrlHumanoidDynamicRobot * ObjectFactory::createHumanoidDynamicRobot()
{
  CjrlHumanoidDynamicRobot * aHDR = new HumanoidDynamicMultiBody();
  return aHDR;
}

 CjrlDynamicRobot * ObjectFactory::createDynamicRobot()
{
  CjrlDynamicRobot * aDR = new DynamicMultiBody();
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
  dynamicsJRLJapan::HumanoidDynamicMultiBody *aHDR = 
    dynamic_cast<HumanoidDynamicMultiBody *>(&ajrlHumanoidDynamicRobot);

  // TO DO : Not be limited to dynamicsJRLJapan implementation.
  // but right now it is easier.
  if (aHDR!=0)
    {
      aHDR->m_privateObj->parserVRML(OpenHRPVRMLFile,
		       (char *)MapJointToRankFileName.c_str());
      aHDR->m_privateObj->SetHumanoidSpecificitiesFile(SpecificitiesFileName);
      return 0;
    }
  return -1;
}
};

