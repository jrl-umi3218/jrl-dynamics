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

#include "dynamicsJRLJapan/DynamicMultiBody.h"
#include "../private/DynMultiBodyPrivate.h"

using namespace dynamicsJRLJapan;

DynamicMultiBody::DynamicMultiBody()
{
  DynMultiBodyPrivate* obj = new DynMultiBodyPrivate();
  m_privateObj = boost::shared_ptr<DynMultiBodyPrivate>(obj);
}

DynamicMultiBody::DynamicMultiBody(const DynamicMultiBody& inDynamicRobot)
{
  DynMultiBodyPrivate* obj = 
    new DynMultiBodyPrivate(*inDynamicRobot.m_privateObj);
  m_privateObj = boost::shared_ptr<DynMultiBodyPrivate>(obj);
}

DynamicMultiBody::DynamicMultiBody(bool inAllocatePrivate) :
  m_privateObj()
{
  if (inAllocatePrivate) {
    DynMultiBodyPrivate* obj = new DynMultiBodyPrivate();
    m_privateObj = boost::shared_ptr<DynMultiBodyPrivate>(obj);
  }
}

/**
   \brief Initialize data-structure necessary to dynamic computations
   This function should be called after building the tree of joints.
*/
bool DynamicMultiBody::initialize()
{
  return m_privateObj->initialize();
}

/**
   \brief Destructor
*/
DynamicMultiBody::~DynamicMultiBody() 
{
};

/**
   @}
*/
/**
   \name Kinematic chain
   @{
*/

/**
   \brief Set the root joint of the robot.
*/
void DynamicMultiBody::rootJoint(CjrlJoint& inJoint)
{
  m_privateObj->rootJoint(inJoint);
}

/**
   \brief Get the root joint of the robot.
*/
CjrlJoint* DynamicMultiBody::rootJoint() const
{
  return m_privateObj->rootJoint();
}

/**
   \brief Get a vector containing all the joints.
*/
std::vector< CjrlJoint* > DynamicMultiBody::jointVector()
{
  return m_privateObj->jointVector();
}

/**
   \brief Get the chain of joints influencing the relative kinematics between 
   \param inStartJoint and \param inEndJoint.
*/
std::vector<CjrlJoint*> 
DynamicMultiBody::jointsBetween(const CjrlJoint& inStartJoint, 
				const CjrlJoint& inEndJoint) const
{
  return m_privateObj->jointsBetween(inStartJoint,inEndJoint);
}

/**
   \brief Get the upper bound for ith dof.
*/
double DynamicMultiBody::upperBoundDof(unsigned int inRankInConfiguration)
{
  
  return m_privateObj->upperBoundDof(inRankInConfiguration);
}
/**
   \brief Get the lower bound for ith dof.
*/
double DynamicMultiBody::lowerBoundDof(unsigned int inRankInConfiguration)
{
  return m_privateObj->lowerBoundDof(inRankInConfiguration);
}

/**
   \brief Compute the upper bound for ith dof using other configuration values if possible.
*/
double DynamicMultiBody::upperBoundDof(unsigned int inRankInConfiguration,
				       const vectorN& inConfig) 
{
  return m_privateObj->upperBoundDof(inRankInConfiguration,inConfig);
}
/**
   \brief Compute the lower bound for ith dof using other configuration values if possible.
*/
double DynamicMultiBody::lowerBoundDof(unsigned int inRankInConfiguration,
				       const vectorN& inConfig)
{
  return m_privateObj->lowerBoundDof(inRankInConfiguration,inConfig);
}

/**
   \brief Get the number of degrees of freedom of the robot.
*/
unsigned int DynamicMultiBody::numberDof() const
{
  return m_privateObj->numberDof();
}

/**
   \brief Set the joint ordering in the configuration vector
   
   \param inJointVector Vector of the robot joints
   
   Specifies the order of the joints in the configuration vector. 
   The vector should contain all the joints of the current robot.
*/
void 
DynamicMultiBody::setJointOrderInConfig(std::vector<CjrlJoint*> inJointVector)
{
  return m_privateObj->setJointOrderInConfig(inJointVector);
}

/**
   @}
*/

/**
   \name Configuration, velocity and acceleration
*/

/**
   \brief Set the current configuration of the robot.  
   
   \param inConfig the configuration vector \f${\bf q}\f$.
   
   \return true if success, false if failure (the dimension of the
   input vector does not fit the number of degrees of freedom of the
   robot).
*/
bool DynamicMultiBody::currentConfiguration(const vectorN& inConfig)
{
  return m_privateObj->currentConfiguration(inConfig);
}

/**
   \brief Get the current configuration of the robot.
   
   \return the configuration vector \f${\bf q}\f$.
*/
const vectorN& DynamicMultiBody::currentConfiguration() const
{
  return m_privateObj->currentConfiguration();
}

/**
   \brief Set the current velocity of the robot.  
   
   \param inVelocity the velocity vector \f${\bf \dot{q}}\f$.
   
   \return true if success, false if failure (the dimension of the
   input vector does not fit the number of degrees of freedom of the
   robot).
*/
bool DynamicMultiBody::currentVelocity(const vectorN& inVelocity)
{
  return m_privateObj->currentVelocity(inVelocity);
}

/**
   \brief Get the current velocity of the robot.
   
   \return the velocity vector \f${\bf \dot{q}}\f$.
*/
const vectorN& DynamicMultiBody::currentVelocity() const
{
  return m_privateObj->currentVelocity();
}
/**
   \brief Set the current acceleration of the robot.  
   
   \param inAcceleration the acceleration vector \f${\bf \ddot{q}}\f$.
   
   \return true if success, false if failure (the dimension of the
   input vector does not fit the number of degrees of freedom of the
   robot).
*/
bool DynamicMultiBody::currentAcceleration(const vectorN& inAcceleration)
{
  return m_privateObj->currentAcceleration(inAcceleration);
}

/**
   \brief Get the current acceleration of the robot.
   
   \return the acceleration vector \f${\bf \ddot{q}}\f$.
*/
const vectorN& DynamicMultiBody::currentAcceleration() const 
{
  return m_privateObj->currentAcceleration();
}

/**
   \brief Get the current forces of the robot.
   
   \return the force vector \f${\bf f}\f$.
*/
const matrixNxP& DynamicMultiBody::currentForces() const
{
  return m_privateObj->currentForces();
}

/**
   \brief Get the current torques of the robot.
   
   \return the torque vector \f${\bf \tau }\f$.
*/
const matrixNxP& DynamicMultiBody::currentTorques() const 
{
  return m_privateObj->currentTorques();
}


/**
   @}
*/

/**
   \name Forward kinematics and dynamics
*/


/**
   \brief Compute forward kinematics.
   
   Update the position, velocity and accelerations of each
   joint wrt \f${\bf {q}}\f$, \f${\bf \dot{q}}\f$, \f${\bf \ddot{q}}\f$.
   
*/
bool DynamicMultiBody::computeForwardKinematics()
{
  return m_privateObj->computeForwardKinematics();
}


/**
   \brief Compute the dynamics of the center of mass.
   
   Compute the linear and  angular momentum and their time derivatives, at the center of mass.
*/
bool DynamicMultiBody::computeCenterOfMassDynamics() 
{
  return m_privateObj->computeCenterOfMassDynamics();
}

/**
   \brief Get the position of the center of mass.
*/
const vector3d& DynamicMultiBody::positionCenterOfMass() const 
{
  return m_privateObj->positionCenterOfMass();
}

/**
   \brief Get the velocity of the center of mass.
*/
const vector3d& DynamicMultiBody::velocityCenterOfMass()
{
  return m_privateObj->velocityCenterOfMass();
}

/**
   \brief Get the acceleration of the center of mass.
*/
const vector3d& DynamicMultiBody::accelerationCenterOfMass()
{
  return m_privateObj->accelerationCenterOfMass();
}

/**
   \brief Get the linear momentum of the robot.
*/
const vector3d& DynamicMultiBody::linearMomentumRobot()
{
  return m_privateObj->linearMomentumRobot();
}

/**
   \brief Get the time-derivative of the linear momentum.
*/
const vector3d& DynamicMultiBody::derivativeLinearMomentum()
{
  return m_privateObj->derivativeLinearMomentum();
}

/**
   \brief Get the angular momentum of the robot at the center of mass.
*/
const vector3d& DynamicMultiBody::angularMomentumRobot()
{
  return m_privateObj->angularMomentumRobot();
}

/**
   \brief Get the time-derivative of the angular momentum at the center of mass.
*/
const vector3d& DynamicMultiBody::derivativeAngularMomentum()
{
  return m_privateObj->derivativeAngularMomentum();
}

/**
   \brief Get the total mass of the robot
*/
double DynamicMultiBody::mass() const
{
  return m_privateObj->mass();

}
/**
   @}
*/

/**
   \name Jacobian fonctions
*/

/**
   \brief Compute the Jacobian matrix of the center of mass wrt \f${\bf q}\f$.
*/
void DynamicMultiBody::computeJacobianCenterOfMass()
{
  m_privateObj->computeJacobianCenterOfMass();
}

/**
   \brief Get the Jacobian matrix of the center of mass wrt \f${\bf q}\f$.
*/
const matrixNxP& DynamicMultiBody::jacobianCenterOfMass() const 
{
  return m_privateObj->jacobianCenterOfMass();
}

/**
   @}
*/

/**
   \name Control of the implementation
   @{
*/

/**
   \brief Whether the specified property in implemented.
*/
bool DynamicMultiBody::isSupported(const std::string &inProperty) 
{
  return m_privateObj->isSupported(inProperty);
}

/**
   \brief Get property corresponding to command name.

   \param inProperty name of the property.
   \retval outValue value of the property if implemented.

   \note The returned string needs to be cast into the right type (double, int,...).
*/
bool DynamicMultiBody::getProperty(const std::string &inProperty, 
				   std::string& outValue) 
{
  return m_privateObj->getProperty(inProperty,
				   outValue);
}

/**
   \brief Set property corresponding to command name.

   \param inProperty name of the property.
   \param inValue value of the property.

   \note The value string is obtained by writing the 
   corresponding value in a string (operator<<).
*/
bool DynamicMultiBody::setProperty(std::string &inProperty, 
				   const std::string& inValue) 
{
  return m_privateObj->setProperty(inProperty,
				   inValue);

} 

/**
   @}
*/
  
  
/**
   Compute and get position and orientation jacobian
   \param inStartJoint the start of the chain of joints influencing the jacobian.
   \param inEndJoint the joint where the control frame is located.
   \param inFrameLocalPoint the position of the control frame in inEndJoint's local frame.
   \param outjacobian computed jacobian matrix.
   \param offset is the rank of the column of 
   \param outjacobian where writing of jacobian begins.
   \param inIncludeStartFreeFlyer is an option to include the 
   contribution of a fictive freeflyer superposed with \param inStartJoint
  
   \return false if matrix has inadequate size. Number of columns 
   in matrix \param outJacobian must be at least numberDof() if inIncludeStartFreeFlyer = true. 
   It must be at least numberDof()-6 otherwise.
*/
bool DynamicMultiBody::getJacobian(const CjrlJoint& inStartJoint, 
				   const CjrlJoint& inEndJoint, 
				   const vector3d& inFrameLocalPosition, 
				   matrixNxP& outjacobian, 
				   unsigned int offset, 
				   bool inIncludeStartFreeFlyer) 
{
  return m_privateObj->getJacobian(inStartJoint,
				   inEndJoint,
				   inFrameLocalPosition,
				   outjacobian,
				   offset,
				   inIncludeStartFreeFlyer);
}
  
bool 
DynamicMultiBody::getPositionJacobian(const CjrlJoint& inStartJoint, 
				      const CjrlJoint& inEndJoint, 
				      const vector3d& inFrameLocalPosition, 
				      matrixNxP& outjacobian, 
				      unsigned int offset, 
				      bool inIncludeStartFreeFlyer)
{
  return m_privateObj->getPositionJacobian(inStartJoint,
					   inEndJoint,
					   inFrameLocalPosition,
					   outjacobian,
					   offset,
					   inIncludeStartFreeFlyer);
}
  
bool 
DynamicMultiBody::getOrientationJacobian(const CjrlJoint& inStartJoint, 
					 const CjrlJoint& inEndJoint, 
					 matrixNxP& outjacobian, 
					 unsigned int offset, 
					 bool inIncludeStartFreeFlyer) 
{
  return m_privateObj->getOrientationJacobian(inStartJoint, 
					      inEndJoint, 
					      outjacobian, 
					      offset, 
					      inIncludeStartFreeFlyer);
}


bool 
DynamicMultiBody::getJacobianCenterOfMass(const CjrlJoint& inStartJoint, 
					  matrixNxP& outjacobian, 
					  unsigned int offset, 
					  bool inIncludeStartFreeFlyer)
{
  return m_privateObj->getJacobianCenterOfMass(inStartJoint,
					       outjacobian,
					       offset,
					       inIncludeStartFreeFlyer);
}

/*! \name Inertia matrix related methods 
  @{ */
/*! \brief Compute the inertia matrix of the robot according wrt \f${\bf q}\f$.
 */
void DynamicMultiBody::computeInertiaMatrix() 
{
  return m_privateObj->computeInertiaMatrix();
}

/*! \brief Get the inertia matrix of the robot according wrt \f${\bf q}\f$.
 */
const matrixNxP& DynamicMultiBody::inertiaMatrix() const
{
  return m_privateObj->inertiaMatrix();
}
/*! @} */

/*! \name Actuated joints related methods.  
  @{
*/

/** 
    \brief Returns the list of actuated joints. 
*/
const std::vector<CjrlJoint*>& DynamicMultiBody::getActuatedJoints() const 
{
  return m_privateObj->getActuatedJoints();
}

/**
   \brief Specifies the list of actuated joints. 
*/
void 
DynamicMultiBody::setActuatedJoints(std::vector<CjrlJoint*>& lActuatedJoints)
{
  return m_privateObj->setActuatedJoints(lActuatedJoints);
}

/*! 
  @} 
*/


