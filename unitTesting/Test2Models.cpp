#include <string>
#include "dynamicsJRLJapan/Joint.h"
#include "dynamicsJRLJapan/HumanoidDynamicMultiBody.h"
#include "robotDynamics/jrlRobotDynamicsObjectConstructor.h"
using namespace std;
using namespace dynamicsJRLJapan;

#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void RecursiveDisplayOfJoints(CjrlJoint *aJoint)
{
  if (aJoint==0)
    return;

  Joint *a2Joint=0;

  int NbChildren = aJoint->countChildJoints();

  a2Joint = dynamic_cast<Joint *>( aJoint);
  if (a2Joint==0)
    return;

  cout << a2Joint->getName() << " rank : " << a2Joint->rankInConfiguration() << endl;

  cout << "CurrentTransformation " <<
    aJoint->currentTransformation() << endl;

#if 0
  cout << "Number of child  :" << NbChildren << endl;
  for(int i=0;i<NbChildren;i++)
    {
      a2Joint = (Joint *)aJoint->childJoint(i);

      cout << " Child " << i << " " <<a2Joint->getName() << endl;
    }


  cout << "Nb of degree of freedom " << 
    aJoint->numberDof() << endl;

  cout << "Initial Position " <<
    aJoint->initialPosition();

  cout << "CurrentTransformation " <<
    aJoint->currentTransformation() << endl;

  cout << " Joint from root to here:" << endl;
  std::vector<CjrlJoint*> JointsFromRootToHere = aJoint->jointsFromRootToThis();

  cout << " Nb of nodes: " << JointsFromRootToHere.size() << endl;
  for(int i=0;i<JointsFromRootToHere.size();i++)
    {
      Joint * a3Joint = dynamic_cast<Joint *>(JointsFromRootToHere[i]);
      if (a3Joint==0)
	continue;

      cout << a3Joint->getName() << endl;

    }
  CjrlRigidVelocity aRV = aJoint->jointVelocity();
  cout << " Linear Velocity " << aRV.linearVelocity() << endl;
  cout << " Angular Velocity " << aRV.rotationVelocity() << endl;
  CjrlRigidAcceleration aRA = aJoint->jointAcceleration();
  cout << " Linear Acceleration " << aRA.linearAcceleration() << endl;
  cout << " Angular Acceleration " << aRA.rotationAcceleration() << endl;

  cout << "***********************************************" << endl;
  cout << " Display Now information related to children :" << endl;
#endif
  for(int i=0;i<NbChildren;i++)
    {
      // Returns a const so we have to force the casting/
      RecursiveDisplayOfJoints((CjrlJoint *)aJoint->childJoint(i)); 
    }
  //cout << " End for Joint: " << a2Joint->getName() << endl;
}


void DisplayDynamicRobotInformation(CjrlDynamicRobot *aDynamicRobot)
{
  std::vector<CjrlJoint *> aVec = aDynamicRobot->jointVector();
  int r = aVec.size();
  cout << "Number of joints :" << r << endl;
  for(int i=0;i<r;i++)
    {
      Joint * aJoint = dynamic_cast<Joint *>(aVec[i]);
      cout << aJoint->getName();
    }	

  
}

void DisplayMatrix(MAL_MATRIX(,double) &aJ)
{
  for(int i=0;i<6;i++)
    {
      for(unsigned int j=0;j<MAL_MATRIX_NB_COLS(aJ);j++)
	{
	  if (aJ(i,j)==0.0)
	    printf("0 ");
	  else
	    printf("%10.5f ",aJ(i,j));
	}
      printf("\n");
    }

}

int main(int argc, char *argv[])
{

  if (argc!=6)
    {
      cerr << " This program takes 5 arguments: " << endl;
      cerr << "./TestHumanoidDynamicRobot PATH_TO_VRML_FILE VRML_FILE_NAME "<< endl;
      cerr << " PATH_TO_SPECIFICITIES_XML PATH PATH_TO_MAP_JOINT_2_RANK REDUCED_PATH_TO_MAP_JOINT_2_RANK" << endl;
      exit(-1);
    }	
  

  string aSpecificitiesFileName = argv[3];
  string aPath=argv[1];
  string aName=argv[2];
  string aMapFromJointToRank=argv[4];
  string aMapFromJointToRankSmall=argv[5];

  CjrlJoint* rootJoint=0;

  CjrlRobotDynamicsObjectConstructor<
  dynamicsJRLJapan::DynamicMultiBody, 
    dynamicsJRLJapan::HumanoidDynamicMultiBody, 
    dynamicsJRLJapan::JointFreeflyer, 
    dynamicsJRLJapan::JointRotation,
    dynamicsJRLJapan::JointTranslation,
    dynamicsJRLJapan::Body> aRobotDynamicsObjectConstructor;
  
  CjrlHumanoidDynamicRobot * aHDR = aRobotDynamicsObjectConstructor.createhumanoidDynamicRobot();
  
  HumanoidDynamicMultiBody *aHDMB;
  aHDMB = dynamic_cast<dynamicsJRLJapan::HumanoidDynamicMultiBody*>(aHDR);

  CjrlHumanoidDynamicRobot * aHDRSmall = aRobotDynamicsObjectConstructor.createhumanoidDynamicRobot();
  HumanoidDynamicMultiBody *aHDMBSmall;
  aHDMBSmall = dynamic_cast<dynamicsJRLJapan::HumanoidDynamicMultiBody*>(aHDRSmall);

  if (aHDMB==0)
    { 
      cerr<< "Dynamic cast on HDR failed " << endl;
      exit(-1);
    }
  aHDMB->parserVRML(aPath,aName,(char *)aMapFromJointToRank.c_str());
  cout << "Here in between" << endl;
  aHDMB->SetHumanoidSpecificitiesFile(aSpecificitiesFileName);
  cout << " Finished the initialization"<< endl;
  
  aHDMBSmall->parserVRML(aPath,aName,(char *)aMapFromJointToRankSmall.c_str());
  cout << "Here in between" << endl;
  aHDMBSmall->SetHumanoidSpecificitiesFile(aSpecificitiesFileName);
  cout << " Finished the initialization"<< endl;

  // Display tree of the joints.
  cout << "Small model" << endl;
  rootJoint = aHDMBSmall->rootJoint();  

  // Test the tree.
  RecursiveDisplayOfJoints(rootJoint);


  // Tes the computation of the jacobian.
  double dInitPos[40] = { 
    0.0, 0.0, -26.0, 50.0, -24.0, 0.0, 0.0, 0.0, -26.0, 50.0, -24.0, 0.0,  // legs

    0.0, 0.0, 0.0, 0.0, // chest and head

    15.0, -10.0, 0.0, -30.0, 0.0, 0.0, 10.0, // right arm
    15.0,  10.0, 0.0, -30.0, 0.0, 0.0, 10.0, // left arm 

    -20.0, 20.0, -20.0, 20.0, -20.0, // right hand
    -10.0, 10.0, -10.0, 10.0, -10.0  // left hand
  };

  int NbOfDofs = aHDMB->numberDof();
  std::cout << "NbOfDofs :" << NbOfDofs << std::endl;
  MAL_VECTOR_DIM(aCurrentConf,double,NbOfDofs);
  int lindex=0;
  for(int i=0;i<6;i++)
    aCurrentConf[lindex++] = 0.0;
  
  for(int i=0;i<(NbOfDofs-6 < 40 ? NbOfDofs-6 : 40) ;i++)
    aCurrentConf[lindex++] = dInitPos[i]*M_PI/180.0;
  //aCurrentConf[lindex++] = 0.0;
  
  aHDMB->currentConfiguration(aCurrentConf);

  int NbOfDofsSmall = aHDMBSmall->numberDof();
  std::cout << "NbOfDofs :" << NbOfDofsSmall << std::endl;
  MAL_VECTOR_DIM(aCurrentConfSmall,double,NbOfDofsSmall);
  lindex=0;
  for(int i=0;i<6;i++)
    aCurrentConfSmall[lindex++] = 0.0;
  
  for(int i=0;i<(NbOfDofsSmall-6 < 40 ? NbOfDofsSmall-6 : 40) ;i++)
    aCurrentConfSmall[lindex++] = dInitPos[i]*M_PI/180.0;
  //aCurrentConf[lindex++] = 0.0;
  
  aHDMBSmall->currentConfiguration(aCurrentConfSmall);
  
  MAL_VECTOR_DIM(aCurrentVel,double,NbOfDofs); 
  MAL_VECTOR_DIM(aCurrentVelSmall,double,NbOfDofsSmall); 
  lindex=0;
  for(int i=0;i<NbOfDofsSmall;i++)
    aCurrentVelSmall[i] = aCurrentVel[i] =0.0;
  
  MAL_S3_VECTOR(ZMPval,double);

  aHDMB->currentVelocity(aCurrentVel);
  aHDMBSmall->currentVelocity(aCurrentVelSmall);
  
  //  aHDMB->setComputeZMP(true);
  string inProperty="ComputeZMP"; string aValue="true";
  aHDR->setProperty(inProperty,aValue);
  aHDR->computeForwardKinematics();
  
  aHDRSmall->setProperty(inProperty,aValue);
  aHDRSmall->computeForwardKinematics();

  // Display tree of the joints.
  rootJoint = aHDMB->rootJoint();  

  // Test the tree.
  cout << "Normal model" << endl;
  RecursiveDisplayOfJoints(rootJoint);

  // Display tree of the joints.
  cout << "Small model" << endl;
  rootJoint = aHDMBSmall->rootJoint();  

  // Test the tree.
  RecursiveDisplayOfJoints(rootJoint);
  
  ZMPval = aHDMB->zeroMomentumPoint();
  cout << "Normal model" << endl;
  cout << "Value of ZMP : " << ZMPval <<endl;
  cout << "Should be equal to the CoM: " << aHDMB->positionCenterOfMass() << endl;

  ZMPval = aHDMBSmall->zeroMomentumPoint();
  cout << "Small model" << endl;
  cout << "Value of ZMP : " << ZMPval <<endl;
  cout << "Should be equal to the CoM: " << aHDMBSmall->positionCenterOfMass() << endl;

  std::vector<CjrlJoint *> aVec = aHDMBSmall->jointVector();
  Joint  * aJoint = (Joint *)aVec[22]; // Try to get the hand.
  cout << aJoint->getName() << endl;  
  aJoint->computeJacobianJointWrtConfig();

  MAL_MATRIX(,double) aJ = aJoint->jacobianJointWrtConfig();

  DisplayMatrix(aJ);

  delete aHDMB;
  
}
