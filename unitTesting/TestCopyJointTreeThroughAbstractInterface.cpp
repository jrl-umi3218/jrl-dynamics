#include <string>
#include <fstream>
#include <dynamicsJRLJapan/dynamicsJRLJapanFactory.h>

dynamicsJRLJapan::ObjectFactory robotDynamicsObjectConstructor;


using namespace std;
using namespace dynamicsJRLJapan;

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

/*
  This function computes an homogeneous matrix moving 
    (0,0,0) to inCenter and
    x-axis to vector inAxis
*/

void DisplayMatrix4x4(matrix4d & todisplay, ostream &os)
{
  for(unsigned int i=0;i<4;i++)
    {
      for(unsigned int j=0;j<4;j++)
	{
	  os << MAL_S4x4_MATRIX_ACCESS_I_J(todisplay,i,j) << " " ;
	}
      os << endl;
    }
}

void DisplayBody(CjrlBody *aBody,
		 matrix4d &ct,
		 ostream &os)
{
  matrix3d InertiaMatrix = aBody->inertiaMatrix();
  
  os << "Attached body:" << endl;
  os << "Mass of the attached body: " << aBody->mass() << endl;
  os << "Local center of mass: " << aBody->localCenterOfMass() << endl;
  os << "Inertia matrix:" << endl;
  os << InertiaMatrix << endl;
  const vector3d lcm3d = aBody->localCenterOfMass();
  vector4d lcm4d;
  lcm4d[0] = lcm3d[0]; lcm4d[1] = lcm3d[1];
  lcm4d[2] = lcm3d[2]; lcm4d[3] = 1.0;
  vector4d lcmingref;
  os << "Current Position: "<< ct <<endl;
  MAL_S4x4_C_eq_A_by_B(lcmingref, ct,lcm4d);
  os << "Local center of mass:" << lcmingref <<endl;
			     
}
void DisplayJoint(CjrlJoint * aJoint,
		  ostream &os)
{
  unsigned int NbCJ = aJoint->countChildJoints();
  os << "Number of children : " << NbCJ << endl;
  matrix4d ct = aJoint->currentTransformation();
  DisplayMatrix4x4(ct,os);		    
  CjrlBody * linkedBody = aJoint->linkedBody();
  DisplayBody(linkedBody,ct,os);
  

}

void RecursiveDisplayOfJoints(CjrlJoint *aJoint,
			      ostream &os)
{
  if (aJoint==0)
    return;

  int NbChildren = aJoint->countChildJoints();

  DisplayJoint(aJoint,os);

  for(int i=0;i<NbChildren;i++)
    {
      // Returns a const so we have to force the casting/
      RecursiveDisplayOfJoints(aJoint->childJoint(i),os); 
    }

}

void DisplayHumanoid(CjrlHumanoidDynamicRobot *aHDR,
		     ostream &os)
{
  RecursiveDisplayOfJoints(aHDR->rootJoint(),
			   os);
  

  os << "total mass " << aHDR->mass() 
     << " COM: " << aHDR->positionCenterOfMass() << endl;
}

void CopyAndInstanciateBody(CjrlJoint *initJoint, 
			    CjrlJoint *newJoint)
{
  CjrlBody * CopiedBody =newJoint->linkedBody();
  CjrlBody * OriginalBody = initJoint->linkedBody();
  
  // Instanciate Copied Body if needed.
  if (CopiedBody==0)
    {
      CopiedBody= robotDynamicsObjectConstructor.createBody();
      newJoint->setLinkedBody(*CopiedBody);
    }

  // Copying the mass.
  double origmass= OriginalBody->mass();
  CopiedBody->mass(origmass);
  
  // Copying the inertia matrix.
  const matrix3d anInertiaMatrix = OriginalBody->inertiaMatrix();
  CopiedBody->inertiaMatrix(anInertiaMatrix);
  
  // Copying the local center of mass.
  const vector3d aLocalCenterOfMass = OriginalBody->localCenterOfMass();
  CopiedBody->localCenterOfMass(aLocalCenterOfMass);
	  
}
void recursiveMultibodyCopy(CjrlJoint *initJoint, CjrlJoint *newJoint)
{
  int lNbOfChildren= initJoint->countChildJoints();

  // stop test
  if (lNbOfChildren == 0) return ;

  for(int li=0;li<lNbOfChildren;li++) {
    CjrlJoint *Child = initJoint->childJoint(li) ;

    if (Child != 0) {
      
      CjrlJoint* a2newJoint=0;
      const matrix4d pose = Child->initialPosition();
      a2newJoint = robotDynamicsObjectConstructor.createJointRotation(pose);
      newJoint->addChildJoint(*a2newJoint);
      CopyAndInstanciateBody(Child, a2newJoint);

      recursiveMultibodyCopy(Child, a2newJoint) ;
    }
  }
}


void PerformCopyFromJointsTree(CjrlHumanoidDynamicRobot* aHDR,
			       CjrlHumanoidDynamicRobot* a2HDR,
			       string &JointToRank)
{
  CjrlJoint* InitJoint = aHDR->rootJoint() ;
  
  CjrlJoint* newJoint=0;
  matrix4d pose=InitJoint->initialPosition();
  newJoint = robotDynamicsObjectConstructor.createJointFreeflyer(pose);
  CopyAndInstanciateBody(InitJoint,newJoint);
  a2HDR->rootJoint(*newJoint);
  
  recursiveMultibodyCopy(InitJoint, newJoint) ;

  string aProperty("FileJointRank");
  a2HDR->initialize();
  // Copy the bodies.
  std::vector<CjrlJoint *> VecOfInitJoints = aHDR->jointVector();
  std::vector<CjrlJoint *> VecOfCopyJoints = a2HDR->jointVector();
  
  if (VecOfInitJoints.size()!=VecOfCopyJoints.size())
    {
      std::cout << "Problem while copying the joints. size : "<< VecOfInitJoints.size() 
		<< " size copy : " << VecOfCopyJoints.size()  << endl;
      cout << endl << endl << "There is a probleme the new joints vector is not updated" << endl << endl ;
      exit(-1);
    }

}

int main(int argc, char *argv[])
{
  string aPath;
  string aName;
  string JointToRank;
  string aSpecificitiesFileName;

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
      // aCurrentConf[lindex++] = dInitPos[i]*M_PI/180.0;
      aCurrentConf[lindex++] = 0.0;
    }
  
  aHDR->currentConfiguration(aCurrentConf);
  aHDR->currentVelocity(aCurrentVel);
  aHDR->currentAcceleration(aCurrentAcc);
  aHDR->computeForwardKinematics();

  PerformCopyFromJointsTree(aHDR, a2HDR,JointToRank);
  PerformCopyFromJointsTree(a2HDR, a3HDR,JointToRank);


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
  if (CompareTwoFiles((char *)iho.c_str(),
		      (char *)cho.c_str()))
    {
      return 0;
    }
  
  return -1;

}
