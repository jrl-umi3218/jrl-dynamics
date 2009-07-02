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

 CjrlHumanoidDynamicRobot * ObjectFactory::createhumanoidDynamicRobot()
{
  CjrlRobotDynamicsObjectConstructor<
  dynamicsJRLJapan::DynamicMultiBody, 
    dynamicsJRLJapan::HumanoidDynamicMultiBody, 
    dynamicsJRLJapan::JointFreeflyer, 
    dynamicsJRLJapan::JointRotation,
    dynamicsJRLJapan::JointTranslation,
    dynamicsJRLJapan::Body> aRobotDynamicsObjectConstructor;
  
  CjrlHumanoidDynamicRobot * aHDR = aRobotDynamicsObjectConstructor.createhumanoidDynamicRobot();
  return aHDR;
}

 CjrlDynamicRobot * ObjectFactory::createDynamicRobot()
{
  CjrlRobotDynamicsObjectConstructor<
  dynamicsJRLJapan::DynamicMultiBody, 
    dynamicsJRLJapan::HumanoidDynamicMultiBody, 
    dynamicsJRLJapan::JointFreeflyer, 
    dynamicsJRLJapan::JointRotation,
    dynamicsJRLJapan::JointTranslation,
    dynamicsJRLJapan::Body> aRobotDynamicsObjectConstructor;
  
  CjrlDynamicRobot * aDR = aRobotDynamicsObjectConstructor.createDynamicRobot();
  return aDR;
}

 CjrlJoint * ObjectFactory::createJointFreeflyer(const matrix4d& inInitialPosition)
{
  CjrlRobotDynamicsObjectConstructor<
  dynamicsJRLJapan::DynamicMultiBody, 
    dynamicsJRLJapan::HumanoidDynamicMultiBody, 
    dynamicsJRLJapan::JointFreeflyer, 
    dynamicsJRLJapan::JointRotation,
    dynamicsJRLJapan::JointTranslation,
    dynamicsJRLJapan::Body> aRobotDynamicsObjectConstructor;
  
  CjrlJoint * aHDR = aRobotDynamicsObjectConstructor.createJointFreeflyer(inInitialPosition);
  return aHDR;
}

 CjrlJoint * ObjectFactory::createJointRotation(const matrix4d& inInitialPosition)
{
  CjrlRobotDynamicsObjectConstructor<
  dynamicsJRLJapan::DynamicMultiBody, 
    dynamicsJRLJapan::HumanoidDynamicMultiBody, 
    dynamicsJRLJapan::JointFreeflyer, 
    dynamicsJRLJapan::JointRotation,
    dynamicsJRLJapan::JointTranslation,
    dynamicsJRLJapan::Body> aRobotDynamicsObjectConstructor;
  
  CjrlJoint * aHDR = aRobotDynamicsObjectConstructor.createJointRotation(inInitialPosition);
  return aHDR;
}

 CjrlJoint * ObjectFactory::createJointTranslation(const matrix4d& inInitialPosition)
{
  CjrlRobotDynamicsObjectConstructor<
  dynamicsJRLJapan::DynamicMultiBody, 
    dynamicsJRLJapan::HumanoidDynamicMultiBody, 
    dynamicsJRLJapan::JointFreeflyer, 
    dynamicsJRLJapan::JointRotation,
    dynamicsJRLJapan::JointTranslation,
    dynamicsJRLJapan::Body> aRobotDynamicsObjectConstructor;
  
  CjrlJoint * aHDR = aRobotDynamicsObjectConstructor.createJointTranslation(inInitialPosition);
  return aHDR;
}

 CjrlBody * ObjectFactory::createBody()
{
  CjrlRobotDynamicsObjectConstructor<
  dynamicsJRLJapan::DynamicMultiBody, 
    dynamicsJRLJapan::HumanoidDynamicMultiBody, 
    dynamicsJRLJapan::JointFreeflyer, 
    dynamicsJRLJapan::JointRotation,
    dynamicsJRLJapan::JointTranslation,
    dynamicsJRLJapan::Body> aRobotDynamicsObjectConstructor;
  
  CjrlBody * aHDR = aRobotDynamicsObjectConstructor.createBody();
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

