#include <string>
#include <dynamicsJRLJapan/dynamicsJRLJapanFactory.h>

dynamicsJRLJapan::ObjectFactory robotDynamicsObjectConstructor;


using namespace std;
using namespace dynamicsJRLJapan;

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

static matrix4d getPoseFromAxisAndCenter(const vector3d inAxis, const vector3d inCenter)
{
  matrix4d outPose;
  vector3d x(inAxis);
  vector3d y;
  vector3d z;
  int smallestComponent(0);

  x.normalize();

  if((fabs(x[0]) <= fabs(x[1])) && (fabs(x[0]) <= fabs(x[2]))) {
    smallestComponent = 0;
  }
  else if((fabs(x[1]) <= fabs(x[0])) && (fabs(x[1]) <= fabs(x[2]))) {
    smallestComponent = 1;
  }
  else if((fabs(x[2]) <= fabs(x[0])) && (fabs(x[2]) <= fabs(x[1]))) {
    smallestComponent = 2;
  }

  y[smallestComponent] = 1.;


  MAL_S4x4_MATRIX_ACCESS_I_J(outPose,0,0) = x[0];
  MAL_S4x4_MATRIX_ACCESS_I_J(outPose,1,0) = x[1];
  MAL_S4x4_MATRIX_ACCESS_I_J(outPose,2,0) = x[2];
  MAL_S4x4_MATRIX_ACCESS_I_J(outPose,3,0) = inCenter[0];

  MAL_S4x4_MATRIX_ACCESS_I_J(outPose,0,1) = y[0];
  MAL_S4x4_MATRIX_ACCESS_I_J(outPose,1,1) = y[1];
  MAL_S4x4_MATRIX_ACCESS_I_J(outPose,2,1) = y[2];
  MAL_S4x4_MATRIX_ACCESS_I_J(outPose,3,1) = inCenter[1];

  MAL_S4x4_MATRIX_ACCESS_I_J(outPose,0,2) = z[0];
  MAL_S4x4_MATRIX_ACCESS_I_J(outPose,1,2) = z[1];
  MAL_S4x4_MATRIX_ACCESS_I_J(outPose,2,2) = z[2];
  MAL_S4x4_MATRIX_ACCESS_I_J(outPose,3,2) = inCenter[2];

  MAL_S4x4_MATRIX_ACCESS_I_J(outPose,0,3) = 0;
  MAL_S4x4_MATRIX_ACCESS_I_J(outPose,1,3) = 0;
  MAL_S4x4_MATRIX_ACCESS_I_J(outPose,2,3) = 0;
  MAL_S4x4_MATRIX_ACCESS_I_J(outPose,3,3) = 1;

  return outPose;
}


void DisplayBody(CjrlBody *aBody)
{
  matrix3d InertiaMatrix = aBody->inertiaMatrix();
  
  cout << "Attached body:" << endl;
  cout << "Mass of the attached body: " << aBody->mass() << endl;
  cout << "Local center of mass: " << aBody->localCenterOfMass() << endl;
  cout << "Inertia matrix:" << endl;
  cout << InertiaMatrix << endl;
  
}
void DisplayJoint(CjrlJoint * aJoint)
{
  cout << "Joint : " << aJoint << endl;
  unsigned int NbCJ = aJoint->countChildJoints();
  cout << "Number of children : " << NbCJ << endl;
  for(unsigned int li=0;li<NbCJ;li++)
    cout << "Child("<<li<<")=" 
	 << aJoint->childJoint(li) << endl;
  
  cout << "Rank in configuration " << aJoint->rankInConfiguration() << endl;
  matrix4d ct = aJoint->currentTransformation();
  DisplayMatrix4x4(ct,cout);

}

void RecursiveDisplayOfJoints(CjrlJoint *aJoint)
{
  if (aJoint==0)
    return;


  int NbChildren = aJoint->countChildJoints();
 

  DisplayJoint(aJoint);

  for(int i=0;i<NbChildren;i++)
    {
      // Returns a const so we have to force the casting/
      RecursiveDisplayOfJoints(aJoint->childJoint(i)); 
    }

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
      matrix4d pose = Child->currentTransformation();
      a2newJoint = robotDynamicsObjectConstructor.createJointRotation(pose);
      
      newJoint->addChildJoint(*a2newJoint);
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
  matrix4d pose = InitJoint->currentTransformation();
  newJoint = robotDynamicsObjectConstructor.createJointRotation(pose);
  
  a2HDR->rootJoint(*newJoint);
  
  cout << " ================== BEGIN COPY BY CONSTRUCTOR  =================== " << endl ;

  recursiveMultibodyCopy(InitJoint, newJoint) ;
 
  cout << " ================== END COPY BY CONSTRUCTOR  =================== " << endl ;
  RecursiveDisplayOfJoints(a2HDR->rootJoint());

  // Initialize the second humanoid from the joint tree.
  //  a2HDR->setLinksBetweenJointNamesAndRank(LinkJointNameAndRank);

  string aProperty("FileJointRank");
  cout << "JointToRank:" << JointToRank << endl;
  a2HDR->setProperty(aProperty,JointToRank);
  a2HDR->initialize();
  cout << "Finito" << endl;
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

  

  for(unsigned int i=0;i<VecOfInitJoints.size();i++)
    {
      if ((VecOfCopyJoints[i]!=0) &&
	  (VecOfInitJoints[i]!=0))
	*VecOfCopyJoints[i]->linkedBody() = 
	  *VecOfInitJoints[i]->linkedBody();
    }

}

int main(int argc, char *argv[])
{
  if (argc!=5)
    {
      cerr << " This program takes 4 arguments: " << endl;
      cerr << "./TestCopyJointTreeThroughAbstractInterface PATH_TO_VRML_FILE VRML_FILE_NAME "<< endl;
      cerr << " PATH_TO_SPECIFICITIES_XML PATH PATH_TO_MAP_JOINT_2_RANK" << endl;
      exit(-1);
    }	

  int VerboseMode = 3;
  string aPath=argv[1];
  string aName=argv[2];

  string JointToRank = argv[4];
  string aSpecificitiesFileName = argv[3];
  // Read the first humanoid.
  CjrlHumanoidDynamicRobot * aHDR = robotDynamicsObjectConstructor.createHumanoidDynamicRobot();
  string RobotFileName = aPath+aName;
  dynamicsJRLJapan::parseOpenHRPVRMLFile(*aHDR,RobotFileName,JointToRank, aSpecificitiesFileName);
  

  // The second humanoid is constructed through the abstract interface
  //
  CjrlHumanoidDynamicRobot* a2HDR = robotDynamicsObjectConstructor.createHumanoidDynamicRobot();
  
  PerformCopyFromJointsTree(aHDR, a2HDR,JointToRank);


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
      aHDR->setProperty(inProperty[i],inValue[i]);
  }
  

  int NbOfDofs = a2HDR->numberDof();
  if (VerboseMode>2)
    std::cout << "NbOfDofs :" << NbOfDofs << std::endl;

  MAL_VECTOR_DIM(aCurrentConf,double,NbOfDofs);
  int lindex=0;
  for(int i=0;i<6;i++)
    aCurrentConf[lindex++] = 0.0;
  
  for(int i=0;i<(NbOfDofs-6 < 40 ? NbOfDofs-6 : 40) ;i++)
    aCurrentConf[lindex++] = dInitPos[i]*M_PI/180.0;
  
  aHDR->currentConfiguration(aCurrentConf);
  a2HDR->currentConfiguration(aCurrentConf);

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

  if (VerboseMode>2)
    cout << " ================== AFTER PATGEN  =================== " << endl ;
     RecursiveDisplayOfJoints(a2HDR->rootJoint());
  delete aHDR;
  delete a2HDR;

}
