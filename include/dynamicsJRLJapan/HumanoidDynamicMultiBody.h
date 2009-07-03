/* Computation of the dynamic aspect for a humanoid robot.
  
   Copyright (c) 2005-2009, 
   @author Francois Keith, Olivier Stasse. 
   
   JRL-Japan, CNRS/AIST
   
   All rights reserved.
   
   Please see License.txt for more informations on the license related to this software.
   
*/

#ifndef _HUMANOID_DYNAMIC_MULTIBODY_H_
#define _HUMANOID_DYNAMIC_MULTIBODY_H_
#include <vector>

#include "boost/shared_ptr.hpp"
#include "MatrixAbstractLayer/MatrixAbstractLayer.h"
#include "robotDynamics/jrlHumanoidDynamicRobot.h"
#include "dynamicsJRLJapan/DynamicMultiBody.h"

#include "dynamicsJRLJapan/dll.h"


namespace dynamicsJRLJapan
{
  /*
    Forward declaration
  */
  class HumDynMultiBodyPrivate;

class HumanoidDynamicMultiBody : public  virtual CjrlHumanoidDynamicRobot, 
				   public virtual DynamicMultiBody
{
public:

  boost::shared_ptr<HumDynMultiBodyPrivate> m_privateObj;

  HumanoidDynamicMultiBody();

  HumanoidDynamicMultiBody(const HumanoidDynamicMultiBody& inHumanoid);

  /**
     \brief Destructor
  */
  virtual ~HumanoidDynamicMultiBody();

  /**
     \name Joints specific to humanoid robots
  */
  
  /**
     \brief Set the pointer to the waist.
  */
  virtual void waist(CjrlJoint* inWaist);

  /**
     \brief Get a pointer to the waist.
  */
  virtual CjrlJoint* waist();

  /**
     \brief Set the pointer to the chest.
     
     \note for some humanoid robots, the waist and the chest are the same joints.
  */
  virtual void chest(CjrlJoint* inChest);

  /**
     \brief Get a pointer to the chest.
     
     \note for some humanoid robots, the waist and the chest are the same joints.
  */
  virtual CjrlJoint* chest();

  /**
     \brief Set the pointer to the left wrist joint.
  */
  virtual void leftWrist(CjrlJoint* inLeftWrist);

  /**
     \brief Get a pointer to the left wrist.
  */
  virtual CjrlJoint* leftWrist();

  /**
     \brief Set the pointer to the right wrist joint.
  */
  virtual void rightWrist(CjrlJoint* inRightWrist);
  
  /**
     \brief Get a pointer to the right wrist.
  */
  virtual CjrlJoint* rightWrist();

  /**
     \brief Set the pointer to the right hand
  */
  virtual void rightHand(CjrlHand* inRightHand);

  /**
     \brief Get a pointer to the right hand
  */
  virtual CjrlHand* rightHand();
  
  /**
     \brief Set the pointer to the left hand
  */
  virtual void leftHand(CjrlHand* inLeftHand);
  
  /**
     \brief Get a pointer to the left hand
  */
  virtual CjrlHand* leftHand();
  
  /**
     \brief Get the hand clench value. 
     This is a scalar value ranging between 0 and 1 which 
     describes the hand clench (0 for open and 1 for closed hand)
  */
  virtual double getHandClench(CjrlHand* inHand);
  
  /**
     \brief Set the hand clench value. This is a scalar value 
     ranging between 0 and 1 which describes the hand clench 
     (0 for open and 1 for closed hand)
     \return false if parameter 2 is out of range
  */
  virtual bool setHandClench(CjrlHand* inHand, double inClenchingValue);
  
  /**
     \brief Set the pointer to the left foot joint.
  */
  virtual void leftFoot(CjrlFoot* inLeftFoot);

  /**
     \brief Get a pointer to the left foot.
  */
  virtual CjrlFoot* leftFoot();

  /**
     \brief Set the pointer to the right foot joint.
  */
  virtual void rightFoot(CjrlFoot* inRightFoot);

  /**
     \brief Get a pointer to the right foot.
  */
  virtual CjrlFoot* rightFoot();

  /**
     \brief Set gaze joint
        
     \note  For most humanoid robots, the gaze joint is the head.
  */
  virtual void gazeJoint(CjrlJoint* inGazeJoint);

  /**
     \brief Get gaze joint
  */
  virtual CjrlJoint* gazeJoint();
  /**
     \brief Set the gaze orientation and position in the local frame of the gaze joint.
     \return inOrigin a point on the gaze straight line,
     \return inDirection the direction of the gaze joint.
  */
  virtual void gaze(const vector3d& inDirection, const vector3d& inOrigin);

  /**
     \brief Get a point on the gaze straight line
  */
  virtual const vector3d& gazeOrigin() const;

  /**
     \brief Get the direction of gaze
  */
  virtual const vector3d& gazeDirection() const;

  /**
     \@}
  */

  /**
     \name Zero momentum point
  */

  /**
     \brief return the coordinates of the Zero Momentum Point.
  */
  virtual const vector3d& zeroMomentumPoint() const;

  /**
     \brief Return the distance between the sole of a foot and its joint center
     
     \deprecated This piece of information has been moved in class CjrlFoot
    */
  virtual double footHeight() const  __attribute__ ((deprecated));
    
    /*! @} */

  /**
     @}
  */
    
};

};
#endif
