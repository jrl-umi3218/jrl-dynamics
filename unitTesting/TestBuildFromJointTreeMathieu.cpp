#include <string>
#include "dynamicsJRLJapan/Joint.h"
#include "dynamicsJRLJapan/HumanoidDynamicMultiBody.h"
#include "robotDynamics/jrlRobotDynamicsObjectConstructor.h"

using namespace std;
using namespace dynamicsJRLJapan;

void DisplayBody(CjrlBody *aBody)
{
  matrix3d InertiaMatrix = aBody->inertiaMatrix();
  
  cout << "Attached body:" << endl;
  cout << "Mass of the attached body: " << aBody->mass() << endl;
  cout << "Local center of mass: " << aBody->localCenterOfMass() << endl;
  cout << "Inertia matrix:" << endl;
  cout << InertiaMatrix << endl;
  
}
void DisplayJoint(Joint * aJoint)
{
  cout << "Joint : " << aJoint->getName() << endl;
  unsigned int NbCJ = aJoint->countChildJoints();
  if (aJoint->parentJoint()!=0)
    cout << "Father: " << ((Joint *)(aJoint->parentJoint()))->getName() << endl;
  cout << "Number of children : " << NbCJ << endl;
  for(unsigned int li=0;li<NbCJ;li++)
    cout << "Child("<<li<<")=" 
	 << ((Joint*)(aJoint->childJoint(li)))->getName() << endl;
  
  cout << "Current transformation " << aJoint->currentTransformation() << endl;

}

void RecursiveDisplayOfJoints(CjrlJoint *aJoint)
{
  if (aJoint==0)
    return;

  Joint *a2Joint=0;

  int NbChildren = aJoint->countChildJoints();
 
  a2Joint = dynamic_cast<Joint *>( aJoint);
  if (a2Joint==0)
    return;

  DisplayJoint(a2Joint);
  //DisplayBody(aJoint->linkedBody());
  for(int i=0;i<NbChildren;i++)
    {
      // Returns a const so we have to force the casting/
      RecursiveDisplayOfJoints((CjrlJoint *)a2Joint->childJoint(i)); 
    }
  //cout << " End for Joint: " << a2Joint->getName() << endl;
}


void recursiveMultibodyCopy(Joint *initJoint, Joint *newJoint)
{
  int lNbOfChildren= initJoint->countChildJoints();

  // stop test
  if (lNbOfChildren == 0) return ;

  for(int li=0;li<lNbOfChildren;li++)
    {

      Joint *Child = dynamic_cast<Joint *> (initJoint->childJoint(li)) ;
           
      if (Child != 0) {
	
	int type = Child->type();
	vector3d axe = Child->axe();
	float q = Child->quantity();
	matrix4d pose = Child->pose();
	string name = Child->getName() ;
	//	int IDinVRML = Child->getIDinVRML();

	Joint *a2newJoint = new Joint(type, axe, q, pose);
	a2newJoint->setName(name) ;
	//	a2newJoint->setIDinVRML(IDinVRML);
	
	newJoint->addChildJoint(*a2newJoint);
	a2newJoint->SetFatherJoint(newJoint);
	recursiveMultibodyCopy(Child, a2newJoint) ;
      }
	
    }
}


void PerformCopyFromJointsTree(HumanoidDynamicMultiBody *aHDR,
			       HumanoidDynamicMultiBody *a2HDR)
{
  Joint *InitJoint = dynamic_cast<Joint *>( aHDR->rootJoint()) ;
  
  int type =  InitJoint->type();
  vector3d axe = InitJoint->axe();
  float q = InitJoint->quantity();
  matrix4d pose = InitJoint->pose();
  string name = InitJoint->getName() ;
  
  Joint *newJoint = new Joint(type, axe, q, pose);
  newJoint->setName(name) ;
  
  a2HDR->rootJoint(*newJoint);
  
  recursiveMultibodyCopy(InitJoint, newJoint) ;
 
  
  
  cout << " ================== COPY BY CONSTRUCTOR  =================== " << endl ;
  RecursiveDisplayOfJoints(a2HDR->rootJoint());

  // Initialize the second humanoid from the joint tree.

  std::vector<NameAndRank_t> LinkJointNameAndRank;
  aHDR->getLinksBetweenJointNamesAndRank(LinkJointNameAndRank);
  a2HDR->setLinksBetweenJointNamesAndRank(LinkJointNameAndRank);

 
  a2HDR->InitializeFromJointsTree();
  
 
  // Copy the bodies.
  std::vector<CjrlJoint *> VecOfInitJoints = aHDR->jointVector();
  std::vector<CjrlJoint *> VecOfCopyJoints = a2HDR->jointVector();
  
  if (VecOfInitJoints.size()!=VecOfCopyJoints.size())
    {
      std::cout << "Problem while copying the joints. size : " <<VecOfInitJoints.size() << " size copy : " << VecOfCopyJoints.size()  <<endl;
      cout << endl << endl << "There is a probleme the new joints vector is not updated" << endl << endl ;
      exit(-1);
    }

  

  for(unsigned int i=0;i<VecOfInitJoints.size();i++)
    {
      if ((VecOfCopyJoints[i]!=0) &&
	  (VecOfInitJoints[i]!=0))
      *((DynamicBody *)VecOfCopyJoints[i]->linkedBody()) = 
	*((DynamicBody *)VecOfInitJoints[i]->linkedBody());
    }

}

int main(int argc, char *argv[])
{
  int VerboseMode = 3;
#if 0
  string aPath="/home/mpoirier/openrobots/OpenHRP/etc/HRP2JRL/";
  string aName="HRP2JRLmain.wrl";
  string JointToRank = "/home/mpoirier/openrobots/share/walkgenjrl/HRP2LinkJointRank.xml";
  string aSpecificitiesFileName =  "/home/mpoirier/openrobots/share/walkgenjrl/HRP2Specificities.xml";
#else
  string aPath="/home/stasse/src/OpenHRP/etc/HRP2JRL/";
  string aName="HRP2JRLmain.wrl";
  string JointToRank = "/home/stasse/src/OpenHRP/JRL/src/PatternGeneratorJRL/src/data/HRP2LinkJointRank.xml";
  string aSpecificitiesFileName =  "/home/stasse/src/OpenHRP/JRL/src/PatternGeneratorJRL/src/data/HRP2Specificities.xml";

#endif

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
  HumanoidDynamicMultiBody * aHDR = 
    dynamic_cast<HumanoidDynamicMultiBody*>(aRobotDynamicsObjectConstructor.createhumanoidDynamicRobot());

  aHDR->parserVRML(aPath,
		   aName,
		   (char *)JointToRank.c_str());
  
  aHDR->SetHumanoidSpecificitiesFile(aSpecificitiesFileName);


  // Pretend to build the second humanoid.
  HumanoidDynamicMultiBody *a2HDR = 
    dynamic_cast<HumanoidDynamicMultiBody*>(aRobotDynamicsObjectConstructor.createhumanoidDynamicRobot());
  
  PerformCopyFromJointsTree(aHDR, a2HDR);


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
  aHDR->SetTimeStep(0.005);
  aHDR->setComputeAcceleration(false);
  aHDR->setComputeBackwardDynamics(false);
  aHDR->setComputeZMP(true);
  
  a2HDR->SetTimeStep(0.005);
  a2HDR->setComputeAcceleration(false);
  a2HDR->setComputeBackwardDynamics(false);
  a2HDR->setComputeZMP(true);

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
