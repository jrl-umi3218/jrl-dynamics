#include <string>
#include "dynamicsJRLJapan/Joint.h"
#include "dynamicsJRLJapan/HumanoidDynamicMultiBody.h"
#include "robotDynamics/jrlRobotDynamicsObjectConstructor.h"

#include "jrlMathTools/jrlConstants.h"

using namespace std;
using namespace dynamicsJRLJapan;

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

void GoDownTree(const CjrlJoint * startJoint)
{
  std::cout << "joint ranked :" << startJoint->rankInConfiguration() << std::endl;
  std::cout << "Joint name :" << ((Joint *)startJoint)->getName() << std::endl;
  std::cout << "Force on the related Body : " << ((DynamicBody *)(startJoint->linkedBody()))->m_Force << std::endl;
  std::cout << "Torque on the related Body : " << ((DynamicBody *)(startJoint->linkedBody()))->m_Torque << std::endl;
  std::cout << "Mass of the body: " << ((DynamicBody *)(startJoint->linkedBody()))->getMasse() << std::endl;
  std::cout << "Name of the body: " << ((DynamicBody *)(startJoint->linkedBody()))->getName() << std::endl;
  std::cout << "llimit: " << ((Joint *)startJoint)->lowerBound(0)*180/M_PI << " " 
	    << "ulimit: " << ((Joint *)startJoint)->upperBound(0)*180/M_PI << " " << endl;
  std::cout << startJoint->currentTransformation() << std::endl;
  
  if (startJoint->countChildJoints()!=0)
    {
      const CjrlJoint * childJoint = startJoint->childJoint(0);
      GoDownTree(childJoint);
    }
}

int main(int argc, char *argv[])
{
  if (argc!=5)
    {
      cerr << " This program takes 4 arguments: " << endl;
      cerr << "./TestHumanoidDynamicRobot PATH_TO_VRML_FILE VRML_FILE_NAME "<< endl;
      cerr << " PATH_TO_SPECIFICITIES_XML PATH PATH_TO_MAP_JOINT_2_RANK" << endl;
      exit(-1);
    }	

  string aSpecificitiesFileName = argv[3];
  string aPath=argv[1];
  string aName=argv[2];
  string aMapFromJointToRank=argv[4];

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

  if (aHDMB==0)
    { 
      cerr<< "Dynamic cast on HDR failed " << endl;
      exit(-1);
    }
  aHDMB->parserVRML(aPath,aName,(char *)aMapFromJointToRank.c_str());
  cout << "Here in between" << endl;
  aHDMB->SetHumanoidSpecificitiesFile(aSpecificitiesFileName);
  cout << " Finished the initialization"<< endl;
  
  // Display tree of the joints.
  CjrlJoint* rootJoint = aHDMB->rootJoint();  

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
  
  aCurrentConf[2]=0.56;

  for(int i=0;i<(NbOfDofs-6 < 40 ? NbOfDofs-6 : 40) ;i++)
    aCurrentConf[lindex++] = dInitPos[i]*M_PI/180.0;
  //aCurrentConf[lindex++] = 0.0;
  
  aHDMB->currentConfiguration(aCurrentConf);

  MAL_VECTOR_DIM(aCurrentVel,double,NbOfDofs); 
  lindex=0;
  for(int i=0;i<NbOfDofs;i++)
    aCurrentVel[lindex++] = 0.0;
  
  MAL_S3_VECTOR(ZMPval,double);

  aHDMB->currentVelocity(aCurrentVel);
  //  aHDMB->setComputeZMP(true);
  string inProperty="ComputeZMP"; string aValue="true";
  aHDR->setProperty(inProperty,aValue);
  aHDR->computeForwardKinematics();
  ZMPval = aHDMB->zeroMomentumPoint();
  cout << "First value of ZMP : " << ZMPval <<endl;
  cout << "Should be equal to the CoM: " << aHDMB->positionCenterOfMass() << endl;

  aHDMB->LinkBetweenJointsAndEndEffectorSemantic();

  std::vector<CjrlJoint *> aVec = aHDMB->jointVector();
  
  Joint  * aJoint = (Joint *)aVec[22]; // Try to get the hand.
  cout << aJoint->getName() << endl;  
  aJoint->computeJacobianJointWrtConfig();

  MAL_MATRIX(,double) aJ = aJoint->jacobianJointWrtConfig();
  
  //  DisplayMatrix(aJ);
  cout << "****************************" << endl;
  cout << "Root: " << ((Joint *)rootJoint)->getName() << endl;
  rootJoint->computeJacobianJointWrtConfig();
  aJ = rootJoint->jacobianJointWrtConfig();  
  cout << "Rank of Root: " << rootJoint->rankInConfiguration() << endl;

  //  DisplayMatrix(aJ);

  aJoint = (Joint *)aHDMB->waist();
  cout << "Name of the WAIST joint :" << endl;
  cout << aJoint->getName() << endl;
  cout << "****************************" << endl;
  aHDMB->computeJacobianCenterOfMass();
  cout << "Value of the CoM's Jacobian:" << endl
       << aHDMB->jacobianCenterOfMass() << endl;
  cout << "****************************" << endl;
  GoDownTree(aHDMB->rootJoint());

  cout << "Mass of the robot " << aHDMB->getMasse() << endl;
  cout << "Force " << aHDMB->getMasse()*9.81 << endl;

  cout << "****************************" << endl;
  // Test rank of the left hand.
  cout << "Rank of the left hand "<< endl;
  cout << aHDMB->leftWrist()->rankInConfiguration() << endl;
  cout << ((Joint *)aHDMB->leftWrist())->getName() << endl;
  cout << ((Joint *)aHDMB->leftWrist())->getIDinVRML() << endl;

  cout << "Position of the right hand "<< endl;
  cout << ((Joint *)aHDMB->rightWrist())->currentTransformation() << endl;


  MAL_VECTOR_FILL(aCurrentVel,0.0);
  MAL_VECTOR_DIM(aCurrentAcc,double,NbOfDofs);
  MAL_VECTOR_FILL(aCurrentAcc,0.0);

  // This is mandatory for this implementation of computeForwardKinematics
  // to compute the derivative of the momentum.
  aHDMB->SetTimeStep(0.005);
  aHDMB->setComputeAcceleration(false);
  aHDMB->setComputeBackwardDynamics(false);
  aHDMB->setComputeZMP(true);
  for(int i=0;i<4;i++)
    {
      aHDMB->currentVelocity(aCurrentVel);
      aHDMB->currentAcceleration(aCurrentAcc);
      aHDMB->computeForwardKinematics();
      ZMPval = aHDMB->zeroMomentumPoint();
      cout << i << "-th value of ZMP : " << ZMPval <<endl;
      cout << "Should be equal to the CoM: " << aHDMB->positionCenterOfMass() << endl;
    }

  // Height of the foot. 
  cout << "Height foot: "<< aHDMB->footHeight() << endl;


  
  delete aHDMB;
  
}
