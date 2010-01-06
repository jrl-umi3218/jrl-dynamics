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
	  os << MAL_S3x3_MATRIX_ACCESS_I_J(todisplay,i,j) << " " ;
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

void DisplayBody(CjrlBody *aBody, string &shifttab,ostream &tcout)
{
  
  tcout << shifttab << "Related body informations" << endl;
  vector3d alcm = aBody->localCenterOfMass();
  tcout << shifttab << "Local center of Mass : ";
  dv3d(alcm,tcout);
  matrix3d aim = aBody->inertiaMatrix();
  tcout << shifttab << "Inertia Matrix: " ;
  dm3d(aim,tcout,shifttab);
  tcout << shifttab << "mass: " << aBody->mass() << endl;
      
}

void DisplayHand(CjrlHand *ajrlHand,string &shifttab,ostream &tcout)
{
  vector3d outCenter,outThumbAxis, outForeFinger,
    outPalmNormal;
  ajrlHand->getCenter(outCenter);
  tcout << shifttab << "Center:" 
	<< outCenter(0) << " " 
	<< outCenter(1) << " "
	<< outCenter(2) << endl;
  
  ajrlHand->getThumbAxis(outThumbAxis);
  tcout << shifttab << "Center:" 
	<< outThumbAxis(0) << " " 
	<< outThumbAxis(1) << " "
	<< outThumbAxis(2) << endl;
  
  ajrlHand->getForeFingerAxis(outForeFinger);
  tcout << shifttab << "Center:" 
	<< outForeFinger(0) << " " 
	<< outForeFinger(1) << " "
	<< outForeFinger(2) << endl;
  
  ajrlHand->getPalmNormal(outPalmNormal);
  tcout << shifttab << "Center:" 
	<< outForeFinger(0) << " " 
	<< outForeFinger(1) << " "
	<< outForeFinger(2) << endl;
  
}

void DisplayFoot(CjrlFoot *aFoot,string &shifttab,ostream &tcout)
{
  vector3d AnklePositionInLocalFrame,
    SoleCenterInLocalFrame,
    ProjectionCenterLocalFrameInSole;

  double SoleLength, SoleWidth;
  aFoot->getSoleSize(SoleLength,SoleWidth);
  
  tcout << shifttab << "SoleLength: " << SoleLength << " " 
	<< "SoleWidth: " << SoleWidth << std::endl;
  
  aFoot->getAnklePositionInLocalFrame(AnklePositionInLocalFrame);
  tcout << shifttab << "AnklePositionInLocalFrame=("
	<< AnklePositionInLocalFrame(0) << " , "
	<< AnklePositionInLocalFrame(1) << " , "
	<< AnklePositionInLocalFrame(2) << ")" << std::endl;

  aFoot->getSoleCenterInLocalFrame(SoleCenterInLocalFrame);
  tcout << shifttab << "SoleCenterInLocalFrame=("
	<< SoleCenterInLocalFrame(0) << " , "
	<< SoleCenterInLocalFrame(1) << " , "
	<< SoleCenterInLocalFrame(2) << " ) " << std::endl;

  aFoot->getSoleCenterInLocalFrame(ProjectionCenterLocalFrameInSole);
  tcout << shifttab << "ProjectCenterLocalFrameInSole=("
	<< ProjectionCenterLocalFrameInSole(0) << " , " 
	<< ProjectionCenterLocalFrameInSole(1) << " , " 
	<< ProjectionCenterLocalFrameInSole(2) << " ) " << std::endl;
}

void RecursiveDisplayOfJoints(CjrlJoint *aJoint, 
			      ostream &tcout,
			      unsigned int verbosedisplay=0,
			      unsigned int ldepth=0)
{
  if (aJoint==0)
    return;

  int NbChildren = aJoint->countChildJoints();
  string shifttab="";
  for(unsigned int i=0;i<ldepth;i++)
	shifttab+=" ";
  
  tcout << shifttab << "Rank : " 
       << aJoint->rankInConfiguration() << endl;
  tcout << shifttab << "Number of child  :" 
       << aJoint->countChildJoints() << endl;
  tcout << shifttab << "Nb of degree of freedom " 
       <<  aJoint->numberDof() << endl;
  

  if (verbosedisplay>3)
    {
      tcout << shifttab << "Initial Position " ;
      matrix4d iP = aJoint->initialPosition();
      dm4d(iP,tcout,shifttab);
      
      tcout << shifttab << "CurrentTransformation ";
      matrix4d cT = aJoint->currentTransformation();
      dm4d(cT,tcout,shifttab);

      // Limits
      tcout << shifttab << "llimit: " 
		<< aJoint->lowerBound(0)*180/M_PI << " " 
		<< shifttab << "ulimit: " 
		<< aJoint->upperBound(0)*180/M_PI << " " << endl;

      tcout << shifttab << "lvlimit: " 
		<< aJoint->lowerVelocityBound(0)*180/M_PI << " " 
		<< shifttab << "uvlimit: " 
		<< aJoint->upperVelocityBound(0)*180/M_PI << " " << endl;
      
      // Path from the root to this joint.
      std::vector<CjrlJoint*> JointsFromRootToHere = aJoint->jointsFromRootToThis();
      tcout << shifttab << "Nb of nodes: " << JointsFromRootToHere.size() << endl
	   << shifttab << "Joint from root to here:" << endl << shifttab;
      for(unsigned int i=0;i<JointsFromRootToHere.size();i++)
	tcout << JointsFromRootToHere[i]->rankInConfiguration() << " ";
      tcout << endl;

      // Current state of the joint.

      // Rigid velocity:
      CjrlRigidVelocity aRV = aJoint->jointVelocity();
      tcout << shifttab << "Linear Velocity ";
      vector3d av3d = aRV.linearVelocity();
      dv3d(av3d,tcout);
      tcout << shifttab << "Angular Velocity ";
      av3d = aRV.rotationVelocity();
      dv3d(av3d,tcout);

      // Rigit Acceleration.
      CjrlRigidAcceleration aRA = aJoint->jointAcceleration();
      tcout << shifttab << "Linear Acceleration ";
      av3d = aRA.linearAcceleration();
      dv3d(av3d,tcout);
      tcout << shifttab << "Angular Acceleration ";
      av3d = aRA.rotationAcceleration();
      dv3d(av3d,tcout);
      
      CjrlBody * aBody = aJoint->linkedBody();
      DisplayBody(aBody,shifttab,tcout);
    }

  if (NbChildren!=0)
    {
      tcout << shifttab << "***********************************************" << endl;
      tcout << shifttab << " Display Now information related to children :" << endl;
      
      for(int i=0;i<NbChildren;i++)
	{
	  // Returns a const so we have to force the casting/
	  RecursiveDisplayOfJoints(aJoint->childJoint(i),tcout,verbosedisplay,ldepth+1); 
	}
    }
}


void DisplayDynamicRobotInformation(CjrlDynamicRobot *aDynamicRobot,ostream &tcout)
{
  std::vector<CjrlJoint *> aVec = aDynamicRobot->jointVector();
  int r = aVec.size();
  tcout << "Number of joints :" << r << endl;
  
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

bool CompareTwoFiles(char *RefFileName, char *OurFileName)
{
  ifstream reffile(RefFileName,ifstream::in), 
    ourfile(OurFileName,ifstream::in);
  unsigned int NbLine=0;
  bool readingok=true;

  while(!reffile.eof())
    {

      char refline[25536],ourline[25536];
      reffile.getline(refline,25536);
      ourfile.getline(ourline,25536);
      if (reffile.gcount()!=ourfile.gcount())
	readingok=false;
      else
	{
	  for(int i=0;i<reffile.gcount();i++)
	    if (refline[i]!=ourline[i])
	      {
		readingok=false;
		cerr << "Error at column " << i << endl;
		break;
	      }
	}
      if (readingok==false)
	{
	  cerr << "Error at line "<< NbLine 
	       << endl << "ref: " << refline 
	       << endl << "our: " << ourline <<endl;
	  break;
	}
      NbLine++;
    }
  return readingok;
}
int main(int argc, char *argv[])
{
  string aSpecificitiesFileName;
  string aPath;
  string aName;
  string aMapFromJointToRank;

  ofstream tcout("output.txt",ofstream::out);

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
  tcout << "NbOfDofs :" << NbOfDofs << std::endl;
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
  tcout << "First value of ZMP : " 
	<< ZMPval(0) << " " 
	<< ZMPval(1) << " " 
	<< ZMPval(2) << endl;
  MAL_S3_VECTOR(poscom,double);
  poscom = aHDR->positionCenterOfMass();
  tcout << "Should be equal to the CoM: " 
	<< poscom(0) << " "
	<< poscom(1) << " "  
	<< poscom(2) << endl;


  matrixNxP InertiaMatrix;
  aHDR->computeInertiaMatrix();
  InertiaMatrix = aHDR->inertiaMatrix();
 
  tcout << "InertiaMatrix("
       << MAL_MATRIX_NB_ROWS(InertiaMatrix)<< "," 
       << MAL_MATRIX_NB_COLS(InertiaMatrix)<< ")"<< endl;
    
  tcout << InertiaMatrix << endl;
  
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
  tcout << "****************************" << endl;
  rootJoint->computeJacobianJointWrtConfig();
  aJ = rootJoint->jacobianJointWrtConfig();  
  tcout << "Rank of Root: " << rootJoint->rankInConfiguration() << endl;

  //  DisplayMatrix(aJ);

  aJoint = aHDR->waist();
  tcout << "****************************" << endl;
  aHDR->computeJacobianCenterOfMass();
  tcout << "Value of the CoM's Jacobian:" << endl
       << aHDR->jacobianCenterOfMass() << endl;
  tcout << "****************************" << endl;
  RecursiveDisplayOfJoints(rootJoint,tcout,10);

  tcout << "****************************" << endl;

  // Test rank of the hands.
  tcout << "Rank of the right hand "<< endl;
  tcout << aHDR->rightWrist()->rankInConfiguration() << endl;
  CjrlHand *rightHand = aHDR->rightHand();
  string empty("");
  DisplayHand(rightHand,empty,tcout);

  tcout << "Rank of the left hand "<< endl;
  tcout << aHDR->leftWrist()->rankInConfiguration() << endl;
  CjrlHand *leftHand = aHDR->leftHand();
  DisplayHand(leftHand,empty,tcout);

  // Test rank of the feet.
  tcout << "Rank of the right foot "<< endl;
  tcout << aHDR->rightFoot()->associatedAnkle()->rankInConfiguration() << endl;
  CjrlFoot *rightFoot = aHDR->rightFoot();
  DisplayFoot(rightFoot,empty,tcout);

  tcout << "Rank of the left foot "<< endl;
  tcout << aHDR->leftFoot()->associatedAnkle()->rankInConfiguration() << endl;
  CjrlFoot *leftFoot = aHDR->leftFoot();
  DisplayFoot(leftFoot,empty,tcout);
  
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
      tcout << i << "-th value of ZMP : " 	
	    << ZMPval(0) << " " 
	    << ZMPval(1) << " " 
	    << ZMPval(2) << endl;
      poscom = aHDR->positionCenterOfMass();
      tcout << "Should be equal to the CoM: "  
	    << poscom(0) << " "
	    << poscom(1) << " "  
	    << poscom(2) << endl;
    }


  // Check the information on actuated joints.
  std::vector<CjrlJoint *> ActuatedJoints = aHDR->getActuatedJoints();
  
  tcout << "Size of actuated Joints:" << ActuatedJoints.size() << endl;
  for(unsigned int i=0;i<ActuatedJoints.size();i++)
    tcout << "Rank of actuated joints ("<<i<< ") in configuration :" 
	  << ActuatedJoints[i]->rankInConfiguration() << endl;

  tcout << "Humanoid mass:" << aHDR->mass() << endl;

  tcout.close();
  // ASCII Comparison between the generated output and the reference one
  // given in argument.
  if (argc==2)
    {
      char OurFileName[120]="output.txt";
      if (CompareTwoFiles(argv[1],OurFileName))
	{
	  delete aHDR;
	  return 0;
	}
    }
  delete aHDR;
  return -1;
}
