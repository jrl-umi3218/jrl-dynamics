#include <string>
#include "dynamicsJRLJapan/Joint.h"
#include "dynamicsJRLJapan/HumanoidDynamicMultiBody.h"
#include "robotDynamics/jrlRobotDynamicsObjectConstructor.h"

using namespace std;
using namespace dynamicsJRLJapan;

void PerformCopyFromJointsTree(HumanoidDynamicMultiBody *aHDR,
			       DynamicMultiBody *aDMB,
			       HumanoidDynamicMultiBody *a2HDR,
			       DynamicMultiBody *a2DMB )
{
  Joint * InitJoint = (Joint *)aHDR->rootJoint(),
    * NextInitJoint = 0;
  Joint * CopyJoint = new Joint(*InitJoint),
    * a2CopyJoint = 0, * NextCopyJoint=0;
  Joint * FatherJoint = 0, *FatherCopyJoint=0;


  a2HDR->rootJoint(*CopyJoint);
  

  // Copy the tree structure.
  while(InitJoint!=0)
  {
    // Copy the children
    int lNbOfChildren= InitJoint->countChildJoints();
    for(int li=0;li<lNbOfChildren;li++)
      {
	Joint *Child = (Joint *)InitJoint->childJoint(li);
	if (Child!=0)
	  {
	    a2CopyJoint = new Joint(*Child);
	    CopyJoint->addChildJoint(*a2CopyJoint);
	    a2CopyJoint->SetFatherJoint(CopyJoint);
	  }
      }

    // Find the next one.      
    // 1. A children.
    NextInitJoint = (dynamicsJRLJapan::Joint *)InitJoint->childJoint(0);
    NextCopyJoint = (dynamicsJRLJapan::Joint *)CopyJoint->childJoint(0);

    // No child.
    if (NextInitJoint==0)
      {
	// If a father exist.
	FatherJoint = (dynamicsJRLJapan::Joint *)InitJoint->parentJoint();	  
	FatherCopyJoint = (dynamicsJRLJapan::Joint *)CopyJoint->parentJoint();	  

	while( (FatherJoint!=0) &&
	       (NextInitJoint==0))
	  {
	    // Find the location of the current node
	    // in the father tree.
	    int NbOfChildren= FatherJoint->countChildJoints();
	    int InitJointPosition=-1;
	    for(int li=0;li<NbOfChildren;li++)
	      if (FatherJoint->childJoint(li)==InitJoint)
		{
		  InitJointPosition = li;
		  break;
		}
	    
	    // If a sibling has not been explored
	    if(InitJointPosition<NbOfChildren-1)
	      {
		// take it !
		NextInitJoint = (dynamicsJRLJapan::Joint *)FatherJoint->childJoint(InitJointPosition+1);
		NextCopyJoint = (dynamicsJRLJapan::Joint *)FatherCopyJoint->childJoint(InitJointPosition+1);
	      }
	    else
	      {
		// otherwise move upward.
		InitJoint =FatherJoint;
		FatherJoint=(dynamicsJRLJapan::Joint *)FatherJoint->parentJoint();
		CopyJoint = FatherCopyJoint;
		FatherCopyJoint=(dynamicsJRLJapan::Joint *)FatherCopyJoint->parentJoint();
	      }
	  }
	// If finally FatherJoint==0 then NextInitJoint too is equal to 0.
	
      }
    
    InitJoint=NextInitJoint;
    CopyJoint=NextCopyJoint;

  }
  

  // Initialize the second humanoid from the joint tree.

  std::vector<NameAndRank_t> LinkJointNameAndRank;
  aDMB->getLinksBetweenJointNamesAndRank(LinkJointNameAndRank);
  a2DMB->setLinksBetweenJointNamesAndRank(LinkJointNameAndRank);
  
  a2DMB->InitializeFromJointsTree();


  // Copy the bodies.
  std::vector<CjrlJoint *> VecOfInitJoints = aDMB->jointVector();
  std::vector<CjrlJoint *> VecOfCopyJoints = a2DMB->jointVector();


  std::cout << VecOfInitJoints.size()<< " " << VecOfCopyJoints.size() << endl;
  
  if (VecOfInitJoints.size()!=VecOfCopyJoints.size())
    {
      std::cout << "Problem while copying the joints. " <<endl;
      exit(-1);
    }
  for(int i=0;i<VecOfInitJoints.size();i++)
    {
      if ((VecOfCopyJoints[i]!=0) &&
	  (VecOfInitJoints[i]!=0))
      *((DynamicBody *)VecOfCopyJoints[i]->linkedBody()) = 
	*((DynamicBody *)VecOfInitJoints[i]->linkedBody());
    }
  

}

int main(int argc, char *argv[])
{
  string aPath="/home/stasse/src/OpenHRP/etc/HRP2JRL/";
  string aName="HRP2JRLmain.wrl";
  string JointToRank = "/home/stasse/src/OpenHRP/JRL/src/PatternGeneratorJRL/src/data/HRP2LinkJointRank.xml";
  string aSpecificitiesFileName =
    "/home/stasse/src/OpenHRP/JRL/src/PatternGeneratorJRL/src/data/HRP2Specificities.xml";

  int argindex=1;
  if (argc>1)
    {
      aPath=argv[argindex++];
      argc--;
    }
  if (argc>1)
    {
      aName=argv[argindex++];
      argc--;
    }
  if (argc>1)
    {
      JointToRank=argv[argindex++];
      argc--;
    }
  if (argc>1)
    {
      aSpecificitiesFileName=argv[argindex++];
      argc--;
    }

  

  CjrlRobotDynamicsObjectConstructor <
  dynamicsJRLJapan::DynamicMultiBody, 
    dynamicsJRLJapan::HumanoidDynamicMultiBody, 
    dynamicsJRLJapan::JointFreeflyer, 
    dynamicsJRLJapan::JointRotation,
    dynamicsJRLJapan::JointTranslation,
    dynamicsJRLJapan::Body >  aRobotDynamicsObjectConstructor;
  
  // Read the first humanoid.
  HumanoidDynamicMultiBody * aHDR = (HumanoidDynamicMultiBody *)
    aRobotDynamicsObjectConstructor.createhumanoidDynamicRobot();

  DynamicMultiBody *aDMB =(DynamicMultiBody *)aHDR->getDynamicMultiBody();

  aDMB->parserVRML(aPath,
		   aName,
		   (char *)JointToRank.c_str());
  
  aHDR->SetHumanoidSpecificitiesFile(aSpecificitiesFileName);


  // Pretend to build the second humanoid.
  HumanoidDynamicMultiBody * a2HDR = (HumanoidDynamicMultiBody *)
    aRobotDynamicsObjectConstructor.createhumanoidDynamicRobot();
  
  DynamicMultiBody *a2DMB= (DynamicMultiBody *)a2HDR->getDynamicMultiBody();

  PerformCopyFromJointsTree(aHDR, aDMB, a2HDR, a2DMB);

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
  aDMB->SetTimeStep(0.005);
  aDMB->setComputeAcceleration(false);
  aDMB->setComputeBackwardDynamics(false);
  aDMB->setComputeZMP(true);
  
  a2DMB->SetTimeStep(0.005);
  a2DMB->setComputeAcceleration(false);
  a2DMB->setComputeBackwardDynamics(false);
  a2DMB->setComputeZMP(true);

  int NbOfDofs = a2DMB->numberDof();
  std::cout << "NbOfDofs :" << NbOfDofs << std::endl;
  MAL_VECTOR_DIM(aCurrentConf,double,NbOfDofs);
  int lindex=0;
  for(int i=0;i<6;i++)
    aCurrentConf[lindex++] = 0.0;
  
  for(int i=0;i<(NbOfDofs-6 < 40 ? NbOfDofs-6 : 40) ;i++)
    aCurrentConf[lindex++] = dInitPos[i]*M_PI/180.0;
  
  aDMB->currentConfiguration(aCurrentConf);
  a2DMB->currentConfiguration(aCurrentConf);

  MAL_VECTOR_DIM(aCurrentVel,double,NbOfDofs); 
  lindex=0;
  for(int i=0;i<NbOfDofs;i++)
    aCurrentVel[lindex++] = 0.0;

  MAL_VECTOR_DIM(aCurrentAcc,double,NbOfDofs);
  MAL_VECTOR_FILL(aCurrentAcc,0.0);
  
  MAL_S3_VECTOR(ZMPval,double);

  for(int i=0;i<4;i++)
    {
      aHDR->currentVelocity(aCurrentVel);
      aHDR->currentAcceleration(aCurrentAcc);
      aHDR->computeForwardKinematics();
      ZMPval = aHDR->zeroMomentumPoint();
      cout << i << "-th value of ZMP : " << ZMPval <<endl;
      cout << "Should be equal to the CoM: " << aDMB->positionCenterOfMass() << endl;

      a2HDR->currentVelocity(aCurrentVel);
      a2HDR->currentAcceleration(aCurrentAcc);
      a2HDR->computeForwardKinematics();
      ZMPval = a2HDR->zeroMomentumPoint();
      cout << i << "-th value of ZMP : " << ZMPval <<endl;
      cout << "Should be equal to the CoM: " << aDMB->positionCenterOfMass() << endl;

    }

  
  delete aHDR;
  delete a2HDR;
}
