/* @doc Object used to handle hand

   Copyright (c) 2005-2009, 

   @author : 
   Olivier Stasse, Oussama Kanoun, Fumio Kanehiro, Florent Lamiraux
   
   JRL-Japan, CNRS/AIST

   All rights reserved.
   
   Please refers to file License.txt for details on the license.

*/

#include "dynamicsJRLJapan/DynamicMultiBody.h"
#include "dynamicsJRLJapan/Hand.h"
#include "dynamicsJRLJapan/HumanoidDynamicMultiBody.h"

#if 0

#define RESETDEBUG4(y) { ofstream DebugFile; \
    DebugFile.open(y,ofstream::out); \
    DebugFile.close();}

#define ODEBUG4(x,y) { ofstream DebugFile; \
    DebugFile.open(y,ofstream::app); \
    DebugFile << "HumanoidDynamicMultiBody: " << x << endl; \
    DebugFile.close();}

#else

#define RESETDEBUG4(y) 
#define ODEBUG4(x,y) 

#endif

#define RESETDEBUG6(y) 
#define ODEBUG6(x,y) 

#define RESETDEBUG5(y) { ofstream DebugFile; \
    DebugFile.open(y,ofstream::out); \
    DebugFile.close();}

#define ODEBUG5(x,y) { ofstream DebugFile; \
    DebugFile.open(y,ofstream::app); \
    DebugFile << "HumanoidDynamicMultiBody: " << x << endl; \
    DebugFile.close();}

#if 1
#define ODEBUG(x)
#else
#define ODEBUG(x)  std::cout << x << endl;
#endif

#define ODEBUG3(x)  std::cout << x << endl;

using namespace dynamicsJRLJapan;


HumanoidDynamicMultiBody::HumanoidDynamicMultiBody() : DynamicMultiBody()
{
  m_rightHand = m_leftHand = 0;
  m_RightFootJoint = m_LeftFootJoint = 0;
  m_HS = NULL;
  m_IK=0;
}

HumanoidDynamicMultiBody::HumanoidDynamicMultiBody(const DynamicMultiBody& inDynamicMultiBody,
						   string aFileNameForHumanoidSpecificities) :
  DynamicMultiBody(inDynamicMultiBody)
{
  m_IK =0;
  SetHumanoidSpecificitiesFile(aFileNameForHumanoidSpecificities);
  
}

void HumanoidDynamicMultiBody::SetHumanoidSpecificitiesFile(string &aFileNameForHumanoidSpecificities)
{
  string aHumanoidName="HRP2JRL";
  m_HS = new HumanoidSpecificities();

  if (m_HS!=0)
    {
      m_HS->ReadXML(aFileNameForHumanoidSpecificities,aHumanoidName);
	
      double AnklePosition[3];
      // Take the right ankle position (should be equivalent)
      m_HS->GetAnklePosition(-1,AnklePosition);
      m_AnkleSoilDistance = AnklePosition[2];
      ODEBUG("AnkleSoilDistance =" << m_AnkleSoilDistance);

      // Lenght of the hip (necessary for 
      double HipLength[3];
      // Takes the left one.
      m_HS->GetHipLength(1,HipLength);

      ODEBUG(WaistToHip[0] << " "
	      << WaistToHip[1] << " "
	      << WaistToHip[2] << " ");
      m_Dt(0) = HipLength[0];
      m_Dt(1) = HipLength[1];
      m_Dt(2) = HipLength[2];

      MAL_S3_VECTOR(StaticToTheLeftHip,double);
      MAL_S3_VECTOR(StaticToTheRightHip,double);
      
      // Displacement between the hip and the waist.
      double WaistToHip[3];
      m_HS->GetWaistToHip(1,WaistToHip);
      m_StaticToTheLeftHip(0) = WaistToHip[0];
      m_StaticToTheLeftHip(1) = WaistToHip[1];
      m_StaticToTheLeftHip(2) = WaistToHip[2]; 

      m_TranslationToTheLeftHip = m_StaticToTheLeftHip;
      
      m_HS->GetWaistToHip(-1,WaistToHip);
      m_StaticToTheRightHip(0) = WaistToHip[0];
      m_StaticToTheRightHip(1) = WaistToHip[1];
      m_StaticToTheRightHip(2) = WaistToHip[2];
      m_TranslationToTheRightHip = m_StaticToTheRightHip;      
      

      // If the Dynamic Multibody object is already loaded
      // create the link between the joints and the effector semantic.
      if (numberDof()!=0)
	LinkBetweenJointsAndEndEffectorSemantic();
      
      //hard-coding HRP2 hand specificities
      vector3d center,okayAxis,showingAxis,palmAxis;
      center[0] = 0;
      center[1] = 0;
      center[2] = -0.17;
      okayAxis[0] = 1;
      okayAxis[1] = 0;
      okayAxis[2] = 0;
      showingAxis[0] = 0;
      showingAxis[1] = 0;
      showingAxis[2] = -1;
      palmAxis[0] = 0;
      palmAxis[1] = 1;
      palmAxis[2] = 0;
  
      rightHand(new Hand(rightWrist(), center, okayAxis, showingAxis, palmAxis));
      palmAxis[1] = -1;
      leftHand(new Hand(leftWrist(), center, okayAxis, showingAxis, palmAxis));
    }
  else
    {
      cerr << "Warning: No appropriate definition of Humanoid Specifities" << endl;
      cerr << "Use default value: " << 0.1 << endl;
      m_AnkleSoilDistance = 0.1;

      // Displacement between the hip and the waist.
      m_Dt(0) = 0.0;
      m_Dt(1) = 0.04;
      m_Dt(2) = 0.0;

    }

  m_IK = new InverseKinematics(this);
  
}

HumanoidDynamicMultiBody::~HumanoidDynamicMultiBody()
{
  if (m_HS!=0)
    delete m_HS;

  if (m_IK!=0)
    delete m_IK;
  
  delete m_rightHand;
  delete m_leftHand;
}

void HumanoidDynamicMultiBody::LinkBetweenJointsAndEndEffectorSemantic()
{
  if (m_HS==0)
    return;

  // Link the correct joints.
  
  // Get the left hand.
  std::vector<int> JointForOneLimb = m_HS->GetArmJoints(1);
  int ListeJointsSize = JointForOneLimb.size();
  //int EndIndex = JointForOneLimb[ListeJointsSize-1];// corresponds to hand opening joint (HRP2)
  int EndIndex = JointForOneLimb[ListeJointsSize-2];//this is the wrist joint

  m_LeftWristJoint = GetJointFromVRMLID(EndIndex);
  
  // Get the right hand.
  JointForOneLimb.clear();
  JointForOneLimb = m_HS->GetArmJoints(-1);
  ListeJointsSize = JointForOneLimb.size();
  //EndIndex = JointForOneLimb[ListeJointsSize-1];// corresponds to hand opening joint (HRP2)
  EndIndex = JointForOneLimb[ListeJointsSize-2];//this is the wrist joint

  m_RightWristJoint = GetJointFromVRMLID(EndIndex);
  
  
  // Get the left foot.
  JointForOneLimb.clear();
  JointForOneLimb = m_HS->GetFootJoints(1);
  ListeJointsSize = JointForOneLimb.size();
  EndIndex = JointForOneLimb[ListeJointsSize-1];
  ODEBUG("Joints for the left foot:" << EndIndex);

  m_LeftFootJoint = GetJointFromVRMLID(EndIndex);
  
  // Get the right foot.
  JointForOneLimb.clear();
  JointForOneLimb = m_HS->GetFootJoints(-1);
  ListeJointsSize = JointForOneLimb.size();
  EndIndex = JointForOneLimb[ListeJointsSize-1];
  ODEBUG("Joints for the right foot:" << EndIndex);

  m_RightFootJoint = GetJointFromVRMLID(EndIndex);

  
  // Get the gaze joint (head) of the humanoid.
  JointForOneLimb.clear();
  JointForOneLimb = m_HS->GetHeadJoints();
  ListeJointsSize = JointForOneLimb.size();
  EndIndex = JointForOneLimb[ListeJointsSize-1];

  m_GazeJoint = GetJointFromVRMLID(EndIndex);

  // Get the waist joint of the humanoid.
  std::vector<int> JointsForWaist = m_HS->GetWaistJoints();
  if (JointsForWaist.size()==1)
    m_WaistJoint = GetJointFromVRMLID(JointsForWaist[0]);
}

const MAL_S3_VECTOR(,double) & HumanoidDynamicMultiBody::zeroMomentumPoint() const
{

  return m_ZeroMomentumPoint;
}

void HumanoidDynamicMultiBody::ComputingZeroMomentumPoint()
{
  m_ZeroMomentumPoint = getZMP();
}

/***************************************************/
/* Implementation of the proxy design pattern for  */
/* the part inherited from jrlDynamicRobot.        */
/***************************************************/

bool HumanoidDynamicMultiBody::computeForwardKinematics()
{
  bool r;
  r= DynamicMultiBody::computeForwardKinematics();
  ComputingZeroMomentumPoint();
  return r;

}


bool HumanoidDynamicMultiBody::jacobianJointWrtFixedJoint(CjrlJoint* inJoint, 
							  MAL_MATRIX(,double) & outJacobian)
{
  cerr<< " The method HumanoidDynamicMultiBody::jacobianJointWrtFixedJoint " <<endl
      << " is not implemented yet. " << endl;
  return true;
}

double HumanoidDynamicMultiBody::footHeight() const
{
  if (m_HS){
    double lWidth, lHeight, lDepth;
    m_HS->GetFootSize(1, lDepth, lWidth, lHeight);
  
    return lHeight;
  }else{
    return 0;
  }  
}


void HumanoidDynamicMultiBody::waist(CjrlJoint * inWaist)
{
  m_WaistJoint = inWaist;
}

CjrlJoint* HumanoidDynamicMultiBody::waist()
{
  return m_WaistJoint;
}

double HumanoidDynamicMultiBody::getHandClench(CjrlHand* inHand)
{
    // default implementation. always returns 0
    return 0;
}

bool HumanoidDynamicMultiBody::setHandClench(CjrlHand* inHand, double inClenchingValue)
{
    // default implementation. always returns false
    return false;
}

int HumanoidDynamicMultiBody::GetFootSize(int WhichFoot, double &Depth, double &Width,double &Height)
{
  if (m_HS!=0)
    return m_HS->GetFootSize(WhichFoot, Depth, Width, Height);
  return -1;
}

double HumanoidDynamicMultiBody::GetTibiaLength(int WhichSide)
{
  if (m_HS!=0)
    return m_HS->GetTibiaLength(WhichSide);
  return 0.0;
}

double HumanoidDynamicMultiBody::GetFemurLength(int WhichSide)
{
  if (m_HS!=0)
    return m_HS->GetFemurLength(WhichSide);
  return 0.0;
}

double HumanoidDynamicMultiBody::GetUpperArmLength(int WhichSide)
{
  if (m_HS!=0)
    return m_HS->GetUpperArmLength(WhichSide);
  return 0.0;
}

double HumanoidDynamicMultiBody::GetForeArmLength(int WhichSide)
{
  if (m_HS!=0)
    return m_HS->GetForeArmLength(WhichSide);
  return 0.0;
}

void HumanoidDynamicMultiBody::GetAnklePosition(int WhichSide, double AnklePosition[3])
{
  if (m_HS!=0)
    m_HS->GetAnklePosition(WhichSide,AnklePosition);
}

void HumanoidDynamicMultiBody::GetWaistToHip(int WhichSide, double WaistToHip[3])
{
  if (m_HS!=0)
    m_HS->GetWaistToHip(WhichSide,WaistToHip);
}

void HumanoidDynamicMultiBody::GetHipLength(int WhichSide, double HipLength[3])
{
  if (m_HS!=0)
    m_HS->GetHipLength(WhichSide,HipLength);
}

int HumanoidDynamicMultiBody::GetArmJointNb(int WhichSide)
{
  if (m_HS!=0)
    return m_HS->GetArmJointNb(WhichSide);
  return -1;
}

const std::vector<int> & HumanoidDynamicMultiBody::GetArmJoints(int WhichSide)
{
  return m_HS->GetArmJoints(WhichSide);
}

int HumanoidDynamicMultiBody::GetLegJointNb(int WhichSide)
{
  if (m_HS!=0)
    return m_HS->GetLegJointNb(WhichSide);
  return -1;
}

const std::vector<int> & HumanoidDynamicMultiBody::GetLegJoints(int WhichSide)
{
  return m_HS->GetLegJoints(WhichSide);
}

int HumanoidDynamicMultiBody::GetFootJointNb(int WhichSide)
{
  if (m_HS!=0)
    return m_HS->GetFootJointNb(WhichSide);
  return -1;
}

const std::vector<int> & HumanoidDynamicMultiBody::GetFootJoints(int WhichSide)
{
  return m_HS->GetFootJoints(WhichSide);
}

int HumanoidDynamicMultiBody::GetHeadJointNb()
{
  if (m_HS!=0)
    return m_HS->GetHeadJointNb();
  return -1;
}

const std::vector<int> & HumanoidDynamicMultiBody::GetHeadJoints()
{
  return m_HS->GetHeadJoints();
}

int HumanoidDynamicMultiBody::GetChestJointNb()
{
  if (m_HS!=0)
    return m_HS->GetChestJointNb();
  return -1;
}

const std::vector<int> & HumanoidDynamicMultiBody::GetChestJoints()
{
  return m_HS->GetChestJoints();
}

int HumanoidDynamicMultiBody::GetWaistJointNb()
{
  return m_HS->GetWaistJointNb();
}

const std::vector<int> & HumanoidDynamicMultiBody::GetWaistJoints()
{
  return m_HS->GetWaistJoints();
}

const std::vector<int> & HumanoidDynamicMultiBody::GetUpperBodyJoints()
{
  return m_HS->GetUpperBodyJoints();
}

int HumanoidDynamicMultiBody::GetUpperBodyJointNb()
{
  return m_HS->GetUpperBodyJointNb();
}



int HumanoidDynamicMultiBody::ComputeInverseKinematicsForLegs(matrix3d & Body_R,
							      vector3d &Body_P,
							      vector3d &Dt,
							      matrix3d &Foot_R,
							      vector3d &Foot_P,
							      vectorN &q)
{
  if (m_IK!=0)
    m_IK->ComputeInverseKinematicsForLegs(Body_R, Body_P,
					  Dt, Foot_R, Foot_P,q);
  return -1;
}

int HumanoidDynamicMultiBody::ComputeInverseKinematicsForArms(double X,
							      double Z,
							      double &Alpha,
							      double &Beta)
{
  if (m_IK!=0)
    return m_IK->ComputeInverseKinematicsForArms(X,Z,Alpha,Beta);
  return -1;
}

double HumanoidDynamicMultiBody::ComputeXmax(double &lZ)
{
  if (m_IK!=0)
    return m_IK->ComputeXmax(lZ);
  return 0.0;
}      

/***************************************************/
/* End of the implementation                       */
/***************************************************/

