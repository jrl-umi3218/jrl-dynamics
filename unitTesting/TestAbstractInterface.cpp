#include <string>
#include <fstream>
#include <dynamicsJRLJapan/dynamicsJRLJapanFactory.h>
#include "CommonTools.h"
#include "HumanoidCopy.h"

dynamicsJRLJapan::ObjectFactory robotDynamicsObjectConstructor;


using namespace std;
using namespace dynamicsJRLJapan;

#define POSITION_STILL 0
#define POSITION_HALF_SITTING 1

int main(int argc, char *argv[])
{
  string aPath;
  string aName;
  string JointToRank;
  string aSpecificitiesFileName;

  int InitialPosition = POSITION_STILL;

  if (argc!=5)
    {
      aPath="./";
      aName="sample.wrl";
      aSpecificitiesFileName = "sampleSpecificities.xml";
      JointToRank = "sampleLinkJointRank.xml";
    }
  else
    {
      aPath=argv[1];
      aName=argv[2];
      JointToRank = argv[4];
      aSpecificitiesFileName = argv[3];
    }	

  int VerboseMode = 0;
  // Read the first humanoid.
  CjrlHumanoidDynamicRobot * aHDR = robotDynamicsObjectConstructor.createHumanoidDynamicRobot();
  string RobotFileName = aPath+aName;
  dynamicsJRLJapan::parseOpenHRPVRMLFile(*aHDR,RobotFileName,JointToRank, aSpecificitiesFileName);
  
  // The second humanoid is constructed through the abstract interface
  //
  CjrlHumanoidDynamicRobot* a2HDR = robotDynamicsObjectConstructor.createHumanoidDynamicRobot();

  // The third humanoid is also constructed through the abstract interface
  //
  CjrlHumanoidDynamicRobot* a3HDR = robotDynamicsObjectConstructor.createHumanoidDynamicRobot();

  // Test the new humanoid structure.
  double dInitPos[40] = { 
    0.0, 0.0, -26.0, 50.0, -24.0, 0.0, 0.0, 0.0, -26.0, 50.0, -24.0, 0.0,  // legs

    0.0, 0.0, 0.0, 0.0, // chest and head

    15.0, -10.0, 0.0, -30.0, 0.0, 0.0, 10.0, // right arm
    15.0,  10.0, 0.0, -30.0, 0.0, 0.0, 10.0, // left arm 

    -20.0, 20.0, -20.0, 20.0, -20.0, // right hand
    -10.0, 10.0, -10.0, 10.0, -10.0  // left hand
  };


  // This is mandatory for this implementation of computeForwardKinematics
  // to compute the derivative of the momentum.
  {
    string inProperty[4]={"TimeStep","ComputeAcceleration",
			  "ComputeBackwardDynamics", "ComputeZMP"};
    string inValue[4]={"0.005","false","false","true"};
    for(unsigned int i=0;i<4;i++)
      {
	aHDR->setProperty(inProperty[i],inValue[i]);
	a2HDR->setProperty(inProperty[i],inValue[i]);
	a3HDR->setProperty(inProperty[i],inValue[i]);
      }
  }

  int NbOfDofs = aHDR->numberDof();

  MAL_VECTOR_DIM(aCurrentConf,double,NbOfDofs);
  MAL_VECTOR_DIM(aCurrentVel,double,NbOfDofs); 
  MAL_VECTOR_FILL(aCurrentVel,0.0);
  MAL_VECTOR_DIM(aCurrentAcc,double,NbOfDofs);
  MAL_VECTOR_FILL(aCurrentAcc,0.0);

  int lindex=0;
  for(int i=0;i<6;i++)
    aCurrentConf[lindex++] = 0.0;
  
  for(int i=0;i<(NbOfDofs-6 < 40 ? NbOfDofs-6 : 40) ;i++)
    {
      if (InitialPosition==POSITION_STILL)
	aCurrentConf[lindex++] = 0.0;
      else if (InitialPosition==POSITION_HALF_SITTING)
	aCurrentConf[lindex++] = dInitPos[i]*M_PI/180.0;
    }
  
  aHDR->currentConfiguration(aCurrentConf);
  aHDR->currentVelocity(aCurrentVel);
  aHDR->currentAcceleration(aCurrentAcc);
  aHDR->computeForwardKinematics();
  
  HumanoidCopy aHumanoidCopier;
  aHumanoidCopier.PerformCopyFromJointsTree(aHDR, a2HDR);

  aHumanoidCopier.PerformCopyFromJointsTree(a2HDR, a3HDR);

  NbOfDofs = a2HDR->numberDof();
  if (VerboseMode>2)
    std::cout << "NbOfDofs :" << NbOfDofs << std::endl;

  a2HDR->currentConfiguration(aCurrentConf);
  a2HDR->currentVelocity(aCurrentVel);
  a2HDR->currentAcceleration(aCurrentAcc);
  a2HDR->computeForwardKinematics();

  a3HDR->currentConfiguration(aCurrentConf);
  a3HDR->currentVelocity(aCurrentVel);
  a3HDR->currentAcceleration(aCurrentAcc);
  a3HDR->computeForwardKinematics();
  
  // Initial Humanoid
  ofstream initialhumanoid;
  initialhumanoid.open("initialhumanoid.output");
  if (initialhumanoid.is_open())
    {
      DisplayHumanoid(aHDR,initialhumanoid);
      initialhumanoid.close();
    }

  // Copied Humanoid 
  ofstream copiedhumanoid;
  copiedhumanoid.open("copiedhumanoid.output");
  if (copiedhumanoid.is_open())
    {
      DisplayHumanoid(a2HDR,copiedhumanoid);
      copiedhumanoid.close();
    }
  // 2nd copied humanoid
  ofstream copied2ndhumanoid;
  copied2ndhumanoid.open("copied2ndhumanoid.output");
  if (copied2ndhumanoid.is_open())
    {
      DisplayHumanoid(a3HDR,copied2ndhumanoid);
      copied2ndhumanoid.close();
    }

  MAL_S3_VECTOR(ZMPval,double);

  for(int i=0;i<4;i++)
    {
      aHDR->currentVelocity(aCurrentVel);
      aHDR->currentAcceleration(aCurrentAcc);
      aHDR->computeForwardKinematics();
      ZMPval = aHDR->zeroMomentumPoint();
      if (VerboseMode>4)
	{
	  cout << i << "-th value of ZMP : " << ZMPval <<endl;
	  cout << "Should be equal to the CoM: " << aHDR->positionCenterOfMass() << endl;
	}

      a2HDR->currentVelocity(aCurrentVel);
      a2HDR->currentAcceleration(aCurrentAcc);
      a2HDR->computeForwardKinematics();
      ZMPval = a2HDR->zeroMomentumPoint();
      if(VerboseMode>4)
	{
	  cout << i << "-th value of ZMP : " << ZMPval <<endl;
	  cout << "Should be equal to the CoM: " << aHDR->positionCenterOfMass() << endl;
	}

    }


  delete aHDR;
  delete a2HDR;
  delete a3HDR;
  
  string iho("copiedhumanoid.output");
  string cho("copied2ndhumanoid.output");
  string rho("reportcopy.output");
  if (CompareTwoFiles((char *)iho.c_str(),
		      (char *)cho.c_str(),
		      (char *)rho.c_str()))
    {
      return 0;
    }
  return -1;

}
