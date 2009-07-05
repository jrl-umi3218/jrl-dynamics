#include <string>
#include <robotDynamicsJRLJapan/Joint.h>
#include <robotDynamicsJRLJapan/HumanoidDynamicMultiBody.h>
#include <robotDynamics/jrlRobotDynamicsObjectConstructor.h>
using namespace std;
using namespace dynamicsJRLJapan;

int main(void)
{
  CjrlRobotDynamicsObjectConstructor<
    dynamicsJRLJapan::DynamicMultiBody,
    dynamicsJRLJapan::HumanoidDynamicMultiBody,
    dynamicsJRLJapan::JointFreeflyer,
    dynamicsJRLJapan::JointRotation,
    dynamicsJRLJapan::JointTranslation,
    dynamicsJRLJapan::Body>
    jrlRobotFactory;
  
  CjrlHumanoidDynamicRobot *attRobot=0;
  attRobot = jrlRobotFactory.createHumanoidDynamicRobot();
  dynamicsJRLJapan::HumanoidDynamicMultiBody *aHDMB;
  aHDMB = (dynamicsJRLJapan::HumanoidDynamicMultiBody*) attRobot;
  
  aHDMB->parserVRML("/home/stasse/src/OpenHRP/etc/HRP2JRL/","HRP2JRLmain.wrl","");
  std::string aName = "/home/stasse/src/OpenHRP/JRL/src/PatternGeneratorJRL/src/data/HRP2Specificities.xml";
  aHDMB->SetHumanoidSpecificitiesFile(aName);
  unsigned int nDof = attRobot->numberDof();
  //Initial joint configuration
  double dInitPos[40] = {
    0.0, 0.0, -26.0, 50.0, -24.0, 0.0, 0.0, 0.0, -26.0, 50.0, -24.0, 0.0,  // legs

    0.0, 0.0, 0.0, 0.0, // chest and head

    15.0, -10.0, 0.0, -30.0, 0.0, 0.0, 10.0, // right arm
    15.0,  10.0, 0.0, -30.0, 0.0, 0.0, 10.0, // left arm

    -20.0, 20.0, -20.0, 20.0, -20.0, // right hand
    -10.0, 10.0, -10.0, 10.0, -10.0  // left hand
  };

  vectorN config(nDof);
  //waist x y z
  config(0) = 0.0;
  config(1) = 0.0;
  config(2) = 0.6487;
  //waist roll pitch yaw
  config(3) = 0.0;
  config(4) = 0.0;
  config(5) = 0.0;

  //joints
  for(unsigned int i=6;i<nDof;i++)
    config(i) = dInitPos[i-6];

  attRobot->currentConfiguration(config);
  attRobot->computeForwardKinematics();

  CjrlJoint* leftfoot= attRobot->leftFoot();
  std::vector<CjrlJoint*> root2jointVector = leftfoot->jointsFromRootToThis();
  
  cout << "LeftFoot joints:" << endl;
  for(int i=0;i<root2jointVector.size();i++)
    {
      Joint * aJoint = (Joint *)root2jointVector[i];
      cout << i << " : " << aJoint->getName();
    }

  CjrlJoint* rightfoot= attRobot->rightFoot();
  root2jointVector = rightfoot->jointsFromRootToThis();
  cout << "RightFoot joints:" << endl;
  for(int i=0;i<root2jointVector.size();i++)
    {
      Joint * aJoint = (Joint *) root2jointVector[i];
      cout << i << " : " << aJoint->getName();
    }


    CjrlJoint* leftwrist= attRobot->leftWrist();
  root2jointVector = leftwrist->jointsFromRootToThis();
  cout << "leftwrist joints:" << endl;
  for(int i=0;i<root2jointVector.size();i++)
    {
      Joint * aJoint = (Joint *) root2jointVector[i];
      cout << i << " : " << aJoint->getName();
    }

    CjrlJoint* rightwrist= attRobot->rightWrist();
    root2jointVector = rightwrist->jointsFromRootToThis();
    cout << "rightwrist joints:" << endl;
  for(int i=0;i<root2jointVector.size();i++)
    {
      Joint * aJoint = (Joint *) root2jointVector[i];
      cout << i << " : " << aJoint->getName();
    }


}
