#include "dynamicsJRLJapan.h"
#include <dynamicsJRLJapan/dynamicsJRLJapanFactory.h>
#include "robotDynamics/jrlRobotDynamicsObjectConstructor.h"
namespace dynamicsJRLJapan
{

 CjrlHumanoidDynamicRobot * ObjectFactory::createhumanoidDynamicRobot()
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
      aHDR->parserVRML(OpenHRPVRMLFile,
		       (char *)MapJointToRankFileName.c_str());
      aHDR->SetHumanoidSpecificitiesFile(SpecificitiesFileName);
      return 0;
    }
  return -1;
}
};

