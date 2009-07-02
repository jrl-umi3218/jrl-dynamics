/*
 *   Copyright (c) 2009 CNRS-AIST
 *
 *   Research carried out within the scope of the Associated
 *   International Laboratory: Joint Japanese-French Robotics
 *   Laboratory (JRL)
 *
 *   Author: Olivier Stasse, Florent Lamiraux
 *
 */

#include "dynamicsJRLJapan/HumanoidDynamicMultiBody.h"
#include "../private/HumDynMultiBodyPrivate.h"

using namespace dynamicsJRLJapan;


HumanoidDynamicMultiBody::HumanoidDynamicMultiBody() :
  DynamicMultiBody(false)
{
  HumDynMultiBodyPrivate* obj = new HumDynMultiBodyPrivate();
  m_privateObj = 
    boost::shared_ptr<HumDynMultiBodyPrivate>(obj);

  /*
    Store the same pointer to private part in base class
  */
  DynamicMultiBody::m_privateObj = m_privateObj;
}

HumanoidDynamicMultiBody::
HumanoidDynamicMultiBody(const HumanoidDynamicMultiBody& inHumanoid) :
  DynamicMultiBody(false)
{
  HumDynMultiBodyPrivate* obj = 
    new HumDynMultiBodyPrivate(*inHumanoid.m_privateObj);
  m_privateObj = boost::shared_ptr<HumDynMultiBodyPrivate>(obj);
  /*
    Store the same pointer to private part in base class
  */
  DynamicMultiBody::m_privateObj = m_privateObj;
}

/**
   \brief Destructor
*/
HumanoidDynamicMultiBody::~HumanoidDynamicMultiBody()
{
}

/**
   \brief Set the pointer to the waist.
*/
void HumanoidDynamicMultiBody::waist(CjrlJoint* inWaist)
{
  m_privateObj->waist(inWaist);
}

/**
   \brief Get a pointer to the waist.
*/
CjrlJoint* HumanoidDynamicMultiBody::waist()
{
  return m_privateObj->waist();

}

/**
   \brief Set the pointer to the chest.
     
   \note for some humanoid robots, the waist and the chest are the same joints.
*/
void HumanoidDynamicMultiBody::chest(CjrlJoint* inChest)
{
  m_privateObj->chest(inChest);
}

/**
   \brief Get a pointer to the chest.
     
   \note for some humanoid robots, the waist and the chest are the same joints.
*/
CjrlJoint* HumanoidDynamicMultiBody::chest()
{
  return m_privateObj->chest();
}

/**
   \brief Set the pointer to the left wrist joint.
*/
void HumanoidDynamicMultiBody::leftWrist(CjrlJoint* inLeftWrist)
{
  m_privateObj->leftWrist(inLeftWrist);
}

/**
   \brief Get a pointer to the left wrist.
*/
CjrlJoint* HumanoidDynamicMultiBody::leftWrist() 
{
  return m_privateObj->leftWrist();
}

/**
   \brief Set the pointer to the right wrist joint.
*/
void HumanoidDynamicMultiBody::rightWrist(CjrlJoint* inRightWrist)
{
  m_privateObj->rightWrist(inRightWrist);
}
  
/**
   \brief Get a pointer to the right wrist.
*/
CjrlJoint* HumanoidDynamicMultiBody::rightWrist()
{
  return m_privateObj->rightWrist();
}

/**
   \brief Set the pointer to the right hand
*/
void HumanoidDynamicMultiBody::rightHand(CjrlHand* inRightHand)
{
  m_privateObj->rightHand(inRightHand);
}

/**
   \brief Get a pointer to the right hand
*/
CjrlHand* HumanoidDynamicMultiBody::rightHand()
{
  return m_privateObj->rightHand();
}
  
/**
   \brief Set the pointer to the left hand
*/
void HumanoidDynamicMultiBody::leftHand(CjrlHand* inLeftHand)
{
  m_privateObj->leftHand(inLeftHand);
}
  
/**
   \brief Get a pointer to the left hand
*/
CjrlHand* HumanoidDynamicMultiBody::leftHand()
{
  return m_privateObj->leftHand();

}
  
/**
   \brief Get the hand clench value. 
   This is a scalar value ranging between 0 and 1 which 
   describes the hand clench (0 for open and 1 for closed hand)
*/
double HumanoidDynamicMultiBody::getHandClench(CjrlHand* inHand)
{
  return m_privateObj->getHandClench(inHand);
}
  
/**
   \brief Set the hand clench value. This is a scalar value 
   ranging between 0 and 1 which describes the hand clench 
   (0 for open and 1 for closed hand)
   \return false if parameter 2 is out of range
*/
bool HumanoidDynamicMultiBody::setHandClench(CjrlHand* inHand, double inClenchingValue)
{
  return m_privateObj->setHandClench(inHand,inClenchingValue);
}
  
/**
   \brief Set the pointer to the left foot joint.
*/
void HumanoidDynamicMultiBody::leftFoot(CjrlFoot* inLeftFoot)
{
  return m_privateObj->leftFoot(inLeftFoot);
}
  
/**
   \brief Get a pointer to the left foot.
*/
CjrlFoot* HumanoidDynamicMultiBody::leftFoot()
{
  return m_privateObj->leftFoot();    
}

/**
   \brief Set the pointer to the right foot joint.
*/
void HumanoidDynamicMultiBody::rightFoot(CjrlFoot* inRightFoot)
{
  return m_privateObj->rightFoot(inRightFoot);    
}

/**
   \brief Get a pointer to the right foot.
*/
CjrlFoot* HumanoidDynamicMultiBody::rightFoot()
{
  return m_privateObj->rightFoot();
}

/**
   \brief Set gaze joint
        
   \note  For most humanoid robots, the gaze joint is the head.
*/
void HumanoidDynamicMultiBody::gazeJoint(CjrlJoint* inGazeJoint)
{
  m_privateObj->gazeJoint(inGazeJoint);
}

/**
   \brief Get gaze joint
*/
CjrlJoint* HumanoidDynamicMultiBody::gazeJoint()
{
  return m_privateObj->gazeJoint();
}

/**
   \brief Set the gaze orientation and position in the local frame of the gaze joint.
   \return inOrigin a point on the gaze straight line,
   \return inDirection the direction of the gaze joint.
*/
void HumanoidDynamicMultiBody::gaze(const vector3d& inDirection, 
				    const vector3d& inOrigin)
{
  m_privateObj->gaze(inDirection,inOrigin);

}

/**
   \brief Get a point on the gaze straight line
*/
const vector3d& HumanoidDynamicMultiBody::gazeOrigin() const
{
  return m_privateObj->gazeOrigin();
}

/**
   \brief Get the direction of gaze
*/
const vector3d& HumanoidDynamicMultiBody::gazeDirection() const
{
  return m_privateObj->gazeOrigin();
}

/**
   \@}
*/

/**
   \name Zero momentum point
*/

/**
   \brief return the coordinates of the Zero Momentum Point.
*/
const vector3d& HumanoidDynamicMultiBody::zeroMomentumPoint() const
{
  return m_privateObj->zeroMomentumPoint();
}


/**
   \brief Return the distance between the sole of a foot and its joint center
     
   \deprecated This piece of information has been moved in class CjrlFoot
*/
double HumanoidDynamicMultiBody::footHeight() const
{
  return 0;
}

    
