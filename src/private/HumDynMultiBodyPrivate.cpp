/* @doc Object used to handle hand

   Copyright (c) 2005-2009, 

   @author : 
   Olivier Stasse, Oussama Kanoun, Fumio Kanehiro, Florent Lamiraux
   
   JRL-Japan, CNRS/AIST

   All rights reserved.
   
   Please refers to file License.txt for details on the license.

*/

#include "Debug.h"

#include "DynMultiBodyPrivate.h"
#include "Hand.h"
#include "HumDynMultiBodyPrivate.h"
#include "HumanoidSpecificities.h"


using namespace dynamicsJRLJapan;


HumDynMultiBodyPrivate::HumDynMultiBodyPrivate() : DynMultiBodyPrivate()
{
  m_rightHand = m_leftHand = 0;
  m_RightFoot = m_LeftFoot = 0;
  m_HS = NULL;
}

HumDynMultiBodyPrivate::HumDynMultiBodyPrivate(const DynMultiBodyPrivate& inDynamicMultiBody,
						   string aFileNameForHumanoidSpecificities) :
  DynMultiBodyPrivate(inDynamicMultiBody)
{
  SetHumanoidSpecificitiesFile(aFileNameForHumanoidSpecificities);
}

void HumDynMultiBodyPrivate::SetHumanoidSpecificitiesFile(string &aFileNameForHumanoidSpecificities)
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

}

HumDynMultiBodyPrivate::~HumDynMultiBodyPrivate()
{
  if (m_HS!=0)
    delete m_HS;
  
  delete m_rightHand;
  delete m_leftHand;
  delete m_RightFoot;
  delete m_LeftFoot;
}

void HumDynMultiBodyPrivate::LinkBetweenJointsAndEndEffectorSemantic()
{
  if (m_HS==0)
    return;

  // Link the correct joints.
  
  // Get the left hand.
  std::vector<int> JointForOneLimb = m_HS->GetArmJoints(1);
  int ListeJointsSize = JointForOneLimb.size();
  //int EndIndex = JointForOneLimb[ListeJointsSize-1];// corresponds to hand opening joint (HRP2)
  int EndIndex = JointForOneLimb[ListeJointsSize-2];//this is the wrist joint

  m_LeftWristJoint = GetJointFromActuatedID(EndIndex);
  
  // Get the right hand.
  JointForOneLimb.clear();
  JointForOneLimb = m_HS->GetArmJoints(-1);
  ListeJointsSize = JointForOneLimb.size();
  //EndIndex = JointForOneLimb[ListeJointsSize-1];// corresponds to hand opening joint (HRP2)
  EndIndex = JointForOneLimb[ListeJointsSize-2];//this is the wrist joint

  m_RightWristJoint = GetJointFromActuatedID(EndIndex);
  
  
  // Get the left foot.
  JointForOneLimb.clear();
  JointForOneLimb = m_HS->GetFootJoints(1);
  ListeJointsSize = JointForOneLimb.size();
  EndIndex = JointForOneLimb[ListeJointsSize-1];
  ODEBUG("Joints for the left foot:" << EndIndex);

  Foot *lLeftFoot = new Foot();
  lLeftFoot->setAssociatedAnkle(GetJointFromActuatedID(EndIndex));
  leftFoot(lLeftFoot);

  // Get the right foot.
  JointForOneLimb.clear();
  JointForOneLimb = m_HS->GetFootJoints(-1);
  ListeJointsSize = JointForOneLimb.size();
  EndIndex = JointForOneLimb[ListeJointsSize-1];
  ODEBUG("Joints for the right foot:" << EndIndex);

  Foot *lRightFoot = new Foot();
  lRightFoot->setAssociatedAnkle(GetJointFromActuatedID(EndIndex));
  rightFoot(lRightFoot);

  // Get the gaze joint (head) of the humanoid.
  JointForOneLimb.clear();
  JointForOneLimb = m_HS->GetHeadJoints();
  ListeJointsSize = JointForOneLimb.size();
  EndIndex = JointForOneLimb[ListeJointsSize-1];

  m_GazeJoint = GetJointFromActuatedID(EndIndex);

  // Get the waist joint of the humanoid.
  std::vector<int> JointsForWaist = m_HS->GetWaistJoints();
  if (JointsForWaist.size()==1)
    m_WaistJoint = GetJointFromActuatedID(JointsForWaist[0]);
}

const MAL_S3_VECTOR(,double) & HumDynMultiBodyPrivate::zeroMomentumPoint() const
{

  return m_ZeroMomentumPoint;
}

void HumDynMultiBodyPrivate::ComputingZeroMomentumPoint()
{
  m_ZeroMomentumPoint = getZMP();
}

/***************************************************/
/* Implementation of the proxy design pattern for  */
/* the part inherited from jrlDynamicRobot.        */
/***************************************************/

bool HumDynMultiBodyPrivate::computeForwardKinematics()
{
  bool r;
  r= DynMultiBodyPrivate::computeForwardKinematics();
  ComputingZeroMomentumPoint();
  return r;

}


bool HumDynMultiBodyPrivate::jacobianJointWrtFixedJoint(CjrlJoint* inJoint, 
							  MAL_MATRIX(,double) & outJacobian)
{
  cerr<< " The method HumDynMultiBodyPrivate::jacobianJointWrtFixedJoint " <<endl
      << " is not implemented yet. " << endl;
  return true;
}

double HumDynMultiBodyPrivate::footHeight() const
{
  if (m_HS){
    double lWidth, lHeight, lDepth;
    m_HS->GetFootSize(1, lDepth, lWidth, lHeight);
  
    return lHeight;
  }else{
    return 0;
  }  
}


void HumDynMultiBodyPrivate::waist(CjrlJoint * inWaist)
{
  m_WaistJoint = inWaist;
}

CjrlJoint* HumDynMultiBodyPrivate::waist()
{
  return m_WaistJoint;
}

double HumDynMultiBodyPrivate::getHandClench(CjrlHand* inHand)
{
    // default implementation. always returns 0
    return 0;
}

bool HumDynMultiBodyPrivate::setHandClench(CjrlHand* inHand, double inClenchingValue)
{
    // default implementation. always returns false
    return false;
}

void HumDynMultiBodyPrivate::leftWrist(CjrlJoint *inLeftWrist)
{ 
  m_LeftWristJoint = inLeftWrist;
}

CjrlJoint *HumDynMultiBodyPrivate::leftWrist()
{ 
  return m_LeftWristJoint;
}

void HumDynMultiBodyPrivate::rightWrist(CjrlJoint *inRightWrist)
{ 
  m_RightWristJoint = inRightWrist;
}

CjrlJoint *HumDynMultiBodyPrivate::rightWrist()
{ 
  return m_RightWristJoint;
}

void HumDynMultiBodyPrivate::rightHand(CjrlHand* inRightHand)
{
  m_rightHand = inRightHand;
}

CjrlHand * HumDynMultiBodyPrivate::rightHand()
{
  return m_rightHand;
}


void HumDynMultiBodyPrivate::leftHand(CjrlHand* inLeftHand)
{
  m_leftHand = inLeftHand;
}

CjrlHand * HumDynMultiBodyPrivate::leftHand()
{ 
  return m_leftHand;
}

void HumDynMultiBodyPrivate::leftFoot(CjrlFoot *inLeftFoot)
{ 
  m_LeftFoot = inLeftFoot;
}

CjrlFoot * HumDynMultiBodyPrivate::leftFoot() 
{ 
  return m_LeftFoot;
}

void HumDynMultiBodyPrivate::rightFoot(CjrlFoot *inRightFoot)
{ 
  m_RightFoot = inRightFoot;
}

CjrlFoot * HumDynMultiBodyPrivate::rightFoot() 
{ 
  return m_RightFoot;
}

void HumDynMultiBodyPrivate::chest(CjrlJoint *inChest)
{
  m_Chest = inChest;
}

CjrlJoint * HumDynMultiBodyPrivate::chest()
{
  return m_Chest;
}

/***************************************************/
/* End of the implementation                       */
/***************************************************/
