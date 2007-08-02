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


HumanoidDynamicMultiBody::HumanoidDynamicMultiBody()
{

  string aFileName = "HumanoidSpecificities.xml";
  DynamicMultiBody *aDMB = new DynamicMultiBody();
  m_DMB = aDMB;
  m_rightHand = m_leftHand = 0;
}

HumanoidDynamicMultiBody::HumanoidDynamicMultiBody(CjrlDynamicRobot* aDMB,
						   string aFileNameForHumanoidSpecificities)
{
  m_DMB = aDMB;
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
      if (m_DMB->numberDof()!=0)
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
  
      m_rightHand = new Hand(rightWrist(), center, okayAxis, showingAxis, palmAxis);
      palmAxis[1] = -1;
      m_leftHand = new Hand(leftWrist(), center, okayAxis, showingAxis, palmAxis);
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

HumanoidDynamicMultiBody::~HumanoidDynamicMultiBody()
{
  if (m_HS!=0)
    delete m_HS;
  
  delete m_rightHand, m_leftHand;
  //why isn't m_DMB deleted ?
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
  DynamicMultiBody *m_SDMB = dynamic_cast<DynamicMultiBody *>(m_DMB);
  if (m_SDMB!=0)
      m_LeftWristJoint = m_SDMB->GetJointFromVRMLID(EndIndex);
  
  // Get the right hand.
  JointForOneLimb.clear();
  JointForOneLimb = m_HS->GetArmJoints(-1);
  ListeJointsSize = JointForOneLimb.size();
  //EndIndex = JointForOneLimb[ListeJointsSize-1];// corresponds to hand opening joint (HRP2)
  EndIndex = JointForOneLimb[ListeJointsSize-2];//this is the wrist joint
  if (m_SDMB!=0)
      m_RightWristJoint = m_SDMB->GetJointFromVRMLID(EndIndex);
  
  
  // Get the left foot.
  JointForOneLimb.clear();
  JointForOneLimb = m_HS->GetFootJoints(1);
  ListeJointsSize = JointForOneLimb.size();
  EndIndex = JointForOneLimb[ListeJointsSize-1];
  ODEBUG("Joints for the left foot:" << EndIndex);
  if (m_SDMB!=0)
    m_LeftFootJoint = m_SDMB->GetJointFromVRMLID(EndIndex);
  
  // Get the right foot.
  JointForOneLimb.clear();
  JointForOneLimb = m_HS->GetFootJoints(-1);
  ListeJointsSize = JointForOneLimb.size();
  EndIndex = JointForOneLimb[ListeJointsSize-1];
  ODEBUG("Joints for the right foot:" << EndIndex);
  if (m_SDMB!=0)
    m_RightFootJoint = m_SDMB->GetJointFromVRMLID(EndIndex);

  
  // Get the gaze joint (head) of the humanoid.
  JointForOneLimb.clear();
  JointForOneLimb = m_HS->GetHeadJoints();
  ListeJointsSize = JointForOneLimb.size();
  EndIndex = JointForOneLimb[ListeJointsSize-1];
  if (m_SDMB!=0)
    m_GazeJoint = m_SDMB->GetJointFromVRMLID(EndIndex);

  // Get the waist joint of the humanoid.
  std::vector<int> JointsForWaist = m_HS->GetWaistJoints();
  if (JointsForWaist.size()==1)
    m_WaistJoint = m_SDMB->GetJointFromVRMLID(JointsForWaist[0]);
  
  
}

void HumanoidDynamicMultiBody::GetJointIDInConfigurationFromVRMLID(std::vector<int> &aVector)
{
  DynamicMultiBody *a_SDMB = dynamic_cast<DynamicMultiBody *>(m_DMB);
  if (a_SDMB!=0)
    a_SDMB->GetJointIDInConfigurationFromVRMLID(aVector);
}

const MAL_S3_VECTOR(,double) & HumanoidDynamicMultiBody::zeroMomentumPoint() const
{

  return m_ZeroMomentumPoint;
}

void HumanoidDynamicMultiBody::ComputingZeroMomentumPoint()
{
  DynamicMultiBody * aDMB = (DynamicMultiBody *)m_DMB;
  m_ZeroMomentumPoint = aDMB->getZMP();
}

/* Methods related to the fixed joints */

void HumanoidDynamicMultiBody::addFixedJoint(CjrlJoint *inFixedJoint)
{
  m_DMB->addFixedJoint(inFixedJoint);
}

unsigned int HumanoidDynamicMultiBody::countFixedJoints() const
{
  return m_DMB->countFixedJoints();
}

void HumanoidDynamicMultiBody::removeFixedJoint(CjrlJoint * inFixedJoint)
{
  m_DMB->removeFixedJoint(inFixedJoint);
}

void HumanoidDynamicMultiBody::clearFixedJoints()
{
  m_DMB->clearFixedJoints();
}

CjrlJoint& HumanoidDynamicMultiBody::fixedJoint(unsigned int inJointRank)
{
  m_DMB->fixedJoint(inJointRank);
}
/* End of Methods related to the fixed joints */


/***************************************************/
/* Implementation of the proxy design pattern for  */
/* the part inherited from jrlDynamicRobot.        */
/***************************************************/

void HumanoidDynamicMultiBody::rootJoint(CjrlJoint &inJoint)
{
  if (m_DMB!=0)
    m_DMB->rootJoint(inJoint);
}

CjrlJoint *HumanoidDynamicMultiBody::rootJoint() const
{
  if (m_DMB==0)
    return 0;
  return m_DMB->rootJoint();
}

std::vector< CjrlJoint* > HumanoidDynamicMultiBody::jointVector()
{  
  return  m_DMB->jointVector();
}

unsigned int HumanoidDynamicMultiBody::numberDof() const
{
  return m_DMB->numberDof();
}

bool HumanoidDynamicMultiBody::currentConfiguration(const MAL_VECTOR(,double) & inConfig)
{
  return m_DMB->currentConfiguration(inConfig);
}

const MAL_VECTOR(,double) & HumanoidDynamicMultiBody::currentConfiguration() const
{
  return m_DMB->currentConfiguration();
}

bool HumanoidDynamicMultiBody::currentVelocity(const MAL_VECTOR(,double) & inVelocity) 
{
  return m_DMB->currentVelocity(inVelocity);
}

const MAL_VECTOR(,double) & HumanoidDynamicMultiBody::currentVelocity()  const
{
  return m_DMB->currentVelocity();
}

bool HumanoidDynamicMultiBody::currentAcceleration(const MAL_VECTOR(,double) & inAcceleration)
{
  return m_DMB->currentAcceleration(inAcceleration);
}

const MAL_VECTOR(,double) & HumanoidDynamicMultiBody::currentAcceleration() const
{
  return m_DMB->currentAcceleration();
}

bool HumanoidDynamicMultiBody::computeForwardKinematics()
{
  bool r;
  r= m_DMB->computeForwardKinematics();
  ComputingZeroMomentumPoint();
  return r;

}

bool HumanoidDynamicMultiBody::applyConfiguration(const vectorN& inConfiguration)
{
    return m_DMB->applyConfiguration(inConfiguration);
}

void HumanoidDynamicMultiBody::FiniteDifferenceStateUpdate(double inTimeStep)
{
    m_DMB->FiniteDifferenceStateUpdate(inTimeStep);
    ComputingZeroMomentumPoint();
}

void HumanoidDynamicMultiBody::FiniteDifferenceStateEstimate(double inTimeStep)
{
    m_DMB->FiniteDifferenceStateEstimate(inTimeStep);
    ComputingZeroMomentumPoint();
}

void HumanoidDynamicMultiBody::SaveCurrentStateAsPastState()
{
    m_DMB->SaveCurrentStateAsPastState();
}

void HumanoidDynamicMultiBody::staticState(const vectorN& inConfiguration)
{
    m_DMB->staticState(inConfiguration);
}

bool HumanoidDynamicMultiBody::computeCenterOfMassDynamics()
{
  return m_DMB->computeCenterOfMassDynamics();
}

const MAL_S3_VECTOR(,double) & HumanoidDynamicMultiBody::positionCenterOfMass()
{
  return m_DMB->positionCenterOfMass();
}

const MAL_S3_VECTOR(,double) & HumanoidDynamicMultiBody::velocityCenterOfMass()
{
  return m_DMB->velocityCenterOfMass();
}

const MAL_S3_VECTOR(,double) & HumanoidDynamicMultiBody::accelerationCenterOfMass()
{
  return m_DMB->accelerationCenterOfMass();
}

const MAL_S3_VECTOR(,double) & HumanoidDynamicMultiBody::linearMomentumRobot()
{
  return m_DMB->linearMomentumRobot();
  
}

const MAL_S3_VECTOR(,double) & HumanoidDynamicMultiBody::derivativeLinearMomentum()
{
  return m_DMB->derivativeLinearMomentum();
}

const MAL_S3_VECTOR(,double) & HumanoidDynamicMultiBody::angularMomentumRobot()
{
  return m_DMB->angularMomentumRobot();
}

const MAL_S3_VECTOR(,double) & HumanoidDynamicMultiBody::derivativeAngularMomentum()
{
  return m_DMB->derivativeAngularMomentum();
}

void HumanoidDynamicMultiBody::computeJacobianCenterOfMass()
{
  return m_DMB->computeJacobianCenterOfMass();
}

const MAL_MATRIX(,double) & HumanoidDynamicMultiBody::jacobianCenterOfMass() const
{
  return m_DMB->jacobianCenterOfMass();
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
  double lWidth, lHeight, lDepth;
  m_HS->GetFootSize(1, lDepth, lWidth, lHeight);
  
  return lHeight;
}


double HumanoidDynamicMultiBody::upperBoundDof(unsigned int inRankInConfiguration)
{
    return m_DMB->upperBoundDof(inRankInConfiguration);
}

double HumanoidDynamicMultiBody::lowerBoundDof(unsigned int inRankInConfiguration)
{
    return m_DMB->lowerBoundDof(inRankInConfiguration);
}

double HumanoidDynamicMultiBody::upperBoundDof(unsigned int inRankInConfiguration, const vectorN& inConfig)
{
    return m_DMB->upperBoundDof(inRankInConfiguration, inConfig);
}

double HumanoidDynamicMultiBody::lowerBoundDof(unsigned int inRankInConfiguration, const vectorN& inConfig)
{
    return m_DMB->lowerBoundDof(inRankInConfiguration, inConfig);
}

void HumanoidDynamicMultiBody::waist(CjrlJoint * inWaist)
{
  // This method is ineffective regarding the internals.
}

CjrlJoint* HumanoidDynamicMultiBody::waist()
{
  return m_WaistJoint;
}

double HumanoidDynamicMultiBody::mass() const
{
    return m_DMB->mass();
}

double HumanoidDynamicMultiBody::getHandClench(CjrlHand* inHand)
{
    
    //Designed for the parallel mechanism of HRP2 two-jaw hands
    unsigned int dof = inHand->associatedWrist()->childJoint(0).rankInConfiguration();
    double upperLimit = m_DMB->upperBoundDof(dof);
    double lowerLimit = m_DMB->lowerBoundDof(dof);
    double curVal = m_DMB->currentConfiguration()(dof);
    
    return (upperLimit - curVal)/(upperLimit-lowerLimit);
}

bool HumanoidDynamicMultiBody::setHandClench(CjrlHand* inHand, double inClenchingValue)
{
    //Designed for the parallel mechanism of HRP2 two-jaw hands
    if (inClenchingValue > 1 || inClenchingValue < 0)
    {
        std::cout << "HumanoidDynamicMultiBody::setHandClench(): argument 2 should be between 0 and 1\n";
        return false;
    }
    
    unsigned int dof = inHand->associatedWrist()->childJoint(0).rankInConfiguration();
    double upperLimit = m_DMB->upperBoundDof(dof);
    double lowerLimit = m_DMB->lowerBoundDof(dof);
    double wantedVal =  upperLimit - inClenchingValue*(upperLimit-lowerLimit);
    
    vectorN config = m_DMB->currentConfiguration();
    
    config(dof) = wantedVal;
    
    unsigned int parallelDofStart;
    if (inHand = m_rightHand)
        parallelDofStart = 36;
    else
        parallelDofStart = 41;
    
    int coef = -1;
    for (unsigned int i = parallelDofStart ;i<parallelDofStart+5; i++)
    {
        config(i) = coef * wantedVal;
        coef *= -1;
    }
    
    m_DMB->currentConfiguration( config );
    return true;
}

/***************************************************/
/* End of the implementation                       */
/***************************************************/
