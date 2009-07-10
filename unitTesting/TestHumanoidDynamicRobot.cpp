#include <string>
#include <iostream>
#include <fstream>
#include <dynamicsJRLJapan/dynamicsJRLJapanFactory.h>

using namespace std;
using namespace dynamicsJRLJapan;
void dv3d(vector3d &av3d, ostream &os) 
{
  for(unsigned int i=0;i<3;i++)
    os << av3d(i) << " ";
  os << endl;
}

void dm3d(const matrix3d &todisplay, ostream &os, string shifttab)
{
  for(unsigned int i=0;i<3;i++)
    {
      for(unsigned int j=0;j<3;j++)
	{
	  os << MAL_S4x4_MATRIX_ACCESS_I_J(todisplay,i,j) << " " ;
	}
      os << endl;
      if (i!=2)
	os << shifttab;
    }
}

void dm4d(const matrix4d &todisplay, ostream &os, string shifttab)
{
  for(unsigned int i=0;i<4;i++)
    {
      for(unsigned int j=0;j<4;j++)
	{
	  os << MAL_S4x4_MATRIX_ACCESS_I_J(todisplay,i,j) << " " ;
	}
      os << endl;
      if (i!=3)
	os << shifttab;
    }
}

void DisplayBody(CjrlBody *aBody, string &shifttab)
{
  
  cout << shifttab << "Related body informations" << endl;
  vector3d alcm = aBody->localCenterOfMass();
  cout << shifttab << "Local center of Mass : ";
  dv3d(alcm,cout);
  matrix3d aim = aBody->inertiaMatrix();
  cout << shifttab << "Inertia Matrix: " ;
  dm3d(aim,cout,shifttab);
  cout << shifttab << "mass: " << aBody->mass() << endl;
      
}
void RecursiveDisplayOfJoints(CjrlJoint *aJoint, 
			      unsigned int verbosedisplay=0,
			      unsigned int ldepth=0)
{
  if (aJoint==0)
    return;

  int NbChildren = aJoint->countChildJoints();
  string shifttab="";
  for(unsigned int i=0;i<ldepth;i++)
	shifttab+=" ";
  
  cout << shifttab << "Rank : " 
       << aJoint->rankInConfiguration() << endl;
  cout << shifttab << "Number of child  :" 
       << aJoint->countChildJoints() << endl;
  cout << shifttab << "Nb of degree of freedom " 
       <<  aJoint->numberDof() << endl;
  

  if (verbosedisplay>3)
    {
      cout << shifttab << "Initial Position " ;
      matrix4d iP = aJoint->initialPosition();
      dm4d(iP,cout,shifttab);
      
      cout << shifttab << "CurrentTransformation ";
      matrix4d cT = aJoint->currentTransformation();
      dm4d(cT,cout,shifttab);

      // Limits
      std::cout << shifttab << "llimit: " 
		<< aJoint->lowerBound(0)*180/M_PI << " " 
		<< shifttab << "ulimit: " 
		<< aJoint->upperBound(0)*180/M_PI << " " << endl;

      std::cout << shifttab << "lvlimit: " 
		<< aJoint->lowerVelocityBound(0)*180/M_PI << " " 
		<< shifttab << "uvlimit: " 
		<< aJoint->upperVelocityBound(0)*180/M_PI << " " << endl;
      
      // Path from the root to this joint.
      std::vector<CjrlJoint*> JointsFromRootToHere = aJoint->jointsFromRootToThis();
      cout << shifttab << "Nb of nodes: " << JointsFromRootToHere.size() << endl
	   << shifttab << "Joint from root to here:" << endl << shifttab;
      for(unsigned int i=0;i<JointsFromRootToHere.size();i++)
	cout << JointsFromRootToHere[i]->rankInConfiguration() << " ";
      cout << endl;

      // Current state of the joint.

      // Rigid velocity:
      CjrlRigidVelocity aRV = aJoint->jointVelocity();
      cout << shifttab << "Linear Velocity ";
      vector3d av3d = aRV.linearVelocity();
      dv3d(av3d,cout);
      cout << shifttab << "Angular Velocity ";
      av3d = aRV.rotationVelocity();
      dv3d(av3d,cout);

      // Rigit Acceleration.
      CjrlRigidAcceleration aRA = aJoint->jointAcceleration();
      cout << shifttab << "Linear Acceleration ";
      av3d = aRA.linearAcceleration();
      dv3d(av3d,cout);
      cout << shifttab << "Angular Acceleration ";
      av3d = aRA.rotationAcceleration();
      dv3d(av3d,cout);
      
      CjrlBody * aBody = aJoint->linkedBody();
      DisplayBody(aBody,shifttab);
    }

  if (NbChildren!=0)
    {
      cout << shifttab << "***********************************************" << endl;
      cout << shifttab << " Display Now information related to children :" << endl;
      
      for(int i=0;i<NbChildren;i++)
	{
	  // Returns a const so we have to force the casting/
	  RecursiveDisplayOfJoints(aJoint->childJoint(i),verbosedisplay,ldepth+1); 
	}
    }
}


void DisplayDynamicRobotInformation(CjrlDynamicRobot *aDynamicRobot)
{
  std::vector<CjrlJoint *> aVec = aDynamicRobot->jointVector();
  int r = aVec.size();
  cout << "Number of joints :" << r << endl;
  
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
  std::cout << "llimit: " << startJoint->lowerBound(0)*180/M_PI << " " 
	    << "ulimit: " << startJoint->upperBound(0)*180/M_PI << " " << endl;
  std::cout << startJoint->currentTransformation() << std::endl;
  
  if (startJoint->countChildJoints()!=0)
    {
      const CjrlJoint * childJoint = startJoint->childJoint(0);
      GoDownTree(childJoint);
    }
}

int main(int argc, char *argv[])
{
  string aSpecificitiesFileName;
  string aPath;
  string aName;
  string aMapFromJointToRank;

  if (argc!=5)
    {
      aPath="./";
      aName="sample.wrl";
      aSpecificitiesFileName = "sampleSpecificities.xml";
      aMapFromJointToRank = "sampleLinkJointRank.xml";
    }
  else 
    {
      aSpecificitiesFileName = argv[3];
      aPath=argv[1];
      aName=argv[2];
      aMapFromJointToRank=argv[4];
    }
  dynamicsJRLJapan::ObjectFactory aRobotDynamicsObjectConstructor;
  
  CjrlHumanoidDynamicRobot * aHDR = aRobotDynamicsObjectConstructor.createHumanoidDynamicRobot();
  

  if (aHDR==0)
    { 
      cerr<< "Dynamic cast on HDR failed " << endl;
      exit(-1);
    }
  string RobotFileName = aPath+aName;
  dynamicsJRLJapan::parseOpenHRPVRMLFile(*aHDR,RobotFileName,aMapFromJointToRank,aSpecificitiesFileName);
  
  // Display tree of the joints.
  CjrlJoint* rootJoint = aHDR->rootJoint();  

  int NbOfDofs = aHDR->numberDof();
  std::cout << "NbOfDofs :" << NbOfDofs << std::endl;
  if (NbOfDofs==0)
    {
      cerr << "Empty Robot..."<< endl;
      return -1;
    }
  MAL_VECTOR_DIM(aCurrentConf,double,NbOfDofs);
  int lindex=0;
  for(int i=0;i<6;i++)
    aCurrentConf[lindex++] = 0.0;
  
  for(int i=0;i<(NbOfDofs-6 < 40 ? NbOfDofs-6 : 40) ;i++)
    aCurrentConf[lindex++] = 0.0;
  
  aHDR->currentConfiguration(aCurrentConf);

  MAL_VECTOR_DIM(aCurrentVel,double,NbOfDofs); 
  lindex=0;
  for(int i=0;i<NbOfDofs;i++)
    aCurrentVel[lindex++] = 0.0;
  
  MAL_S3_VECTOR(ZMPval,double);

  aHDR->currentVelocity(aCurrentVel);
  //  aHDR->setComputeZMP(true);
  string inProperty="ComputeZMP"; string aValue="true";
  aHDR->setProperty(inProperty,aValue);
  aHDR->computeForwardKinematics();
  ZMPval = aHDR->zeroMomentumPoint();
  cout << "First value of ZMP : " << ZMPval <<endl;
  cout << "Should be equal to the CoM: " << aHDR->positionCenterOfMass() << endl;


  matrixNxP InertiaMatrix;
  aHDR->computeInertiaMatrix();
  InertiaMatrix = aHDR->inertiaMatrix();
 
  cout << "InertiaMatrix("
       << MAL_MATRIX_NB_ROWS(InertiaMatrix)<< "," 
       << MAL_MATRIX_NB_COLS(InertiaMatrix)<< ")"<< endl;
    
  cout << InertiaMatrix << endl;
  matrixNxP IMt;
  MAL_TRANSPOSE_A_in_At(InertiaMatrix,IMt);
  matrixNxP IMxIMt;
  MAL_C_eq_A_by_B(IMxIMt,InertiaMatrix,IMt);
  cout << IMxIMt << endl;
  
  ofstream aof;
  aof.open("InertiaMatrix.dat");
  for(unsigned int i=0;i<MAL_MATRIX_NB_ROWS(InertiaMatrix);i++)
    {
      for(unsigned int j=0;j<MAL_MATRIX_NB_COLS(InertiaMatrix);j++)
       {
         aof << InertiaMatrix(i,j) << " ";
       }
      aof << endl;
    }
  aof.close();

  std::vector<CjrlJoint *> aVec = aHDR->jointVector();
  
  CjrlJoint  * aJoint = aVec[3]; // Try to get the hand.
  aJoint->computeJacobianJointWrtConfig();

  MAL_MATRIX(,double) aJ = aJoint->jacobianJointWrtConfig();
  
  //  DisplayMatrix(aJ);
  cout << "****************************" << endl;
  rootJoint->computeJacobianJointWrtConfig();
  aJ = rootJoint->jacobianJointWrtConfig();  
  cout << "Rank of Root: " << rootJoint->rankInConfiguration() << endl;

  //  DisplayMatrix(aJ);

  aJoint = aHDR->waist();
  cout << "****************************" << endl;
  aHDR->computeJacobianCenterOfMass();
  cout << "Value of the CoM's Jacobian:" << endl
       << aHDR->jacobianCenterOfMass() << endl;
  cout << "****************************" << endl;
  RecursiveDisplayOfJoints(rootJoint,10);

  cout << "****************************" << endl;
  // Test rank of the left hand.
  cout << "Rank of the left hand "<< endl;
  cout << aHDR->leftWrist()->rankInConfiguration() << endl;

  MAL_VECTOR_FILL(aCurrentVel,0.0);
  MAL_VECTOR_DIM(aCurrentAcc,double,NbOfDofs);
  MAL_VECTOR_FILL(aCurrentAcc,0.0);

  // This is mandatory for this implementation of computeForwardKinematics
  // to compute the derivative of the momentum.
  {
    string inProperty[4]={"TimeStep","ComputeAcceleration",
			  "ComputeBackwardDynamics", "ComputeZMP"};
    string inValue[4]={"0.005","false","false","true"};
    for(unsigned int i=0;i<4;i++)
      aHDR->setProperty(inProperty[i],inValue[i]);

  }
  for(int i=0;i<4;i++)
    {
      aHDR->currentVelocity(aCurrentVel);
      aHDR->currentAcceleration(aCurrentAcc);
      aHDR->computeForwardKinematics();
      ZMPval = aHDR->zeroMomentumPoint();
      cout << i << "-th value of ZMP : " << ZMPval <<endl;
      cout << "Should be equal to the CoM: " << aHDR->positionCenterOfMass() << endl;
    }

  delete aHDR;
  
}
