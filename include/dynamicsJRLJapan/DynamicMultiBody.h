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

#ifndef DYNAMICSJRLJAPAN_DYNAMICMULTIBODY_H
#define DYNAMICSJRLJAPAN_DYNAMICMULTIBODY_H

#include "boost/shared_ptr.hpp"
#include "MatrixAbstractLayer/MatrixAbstractLayer.h"
#include "robotDynamics/jrlJoint.h"
#include "robotDynamics/jrlDynamicRobot.h"

namespace dynamicsJRLJapan {

  /*
    Forward declaration
  */
  class DynMultiBodyPrivate;

  /**
     \brief Implementation of a dynamic model of robot 

     This implementation follows the standard defined by abstract interface
     CjrlDynamicRobot.
   
  */
  class DynamicMultiBody : public virtual CjrlDynamicRobot
  {
  private:
    boost::shared_ptr<DynMultiBodyPrivate> m_privateObj;

  protected:
    DynamicMultiBody(boost::shared_ptr<DynMultiBodyPrivate> & inDynamicRobot);

  public:


    /**
       \name Initialization
       @{
    */
    DynamicMultiBody();

    DynamicMultiBody(const DynamicMultiBody & inDynamicRobot);
    /**
       \brief Initialize data-structure necessary to dynamic computations
       This function should be called after building the tree of joints.
    */
    virtual bool initialize();

    /**
       \brief Destructor
    */
    virtual ~DynamicMultiBody();
  
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
    virtual void rootJoint(CjrlJoint& inJoint);

    /**
       \brief Get the root joint of the robot.
    */
    virtual CjrlJoint* rootJoint() const;

    /**
       \brief Get a vector containing all the joints.
    */
    virtual std::vector< CjrlJoint* > jointVector();
  
    /**
       \brief Get the chain of joints influencing the relative kinematics between 
       \param inStartJoint and \param inEndJoint.
    */
    virtual std::vector<CjrlJoint*> 
      jointsBetween(const CjrlJoint& inStartJoint, 
		    const CjrlJoint& inEndJoint) const;
    
    /**
       \brief Get the upper bound for ith dof.
    */
    virtual double upperBoundDof(unsigned int inRankInConfiguration);

    /**
       \brief Get the lower bound for ith dof.
    */
    virtual double lowerBoundDof(unsigned int inRankInConfiguration);

    /**
       \brief Compute the upper bound for ith dof using other configuration values if possible.
    */
    virtual double upperBoundDof(unsigned int inRankInConfiguration,
				 const vectorN& inConfig);

    /**
       \brief Compute the lower bound for ith dof using other configuration values if possible.
    */
    virtual double lowerBoundDof(unsigned int inRankInConfiguration,
				 const vectorN& inConfig);

    /**
       \brief Get the number of degrees of freedom of the robot.
    */
    virtual unsigned int numberDof() const;

    /**
       \brief Set the joint ordering in the configuration vector
     
       \param inJointVector Vector of the robot joints

       Specifies the order of the joints in the configuration vector. 
       The vector should contain all the joints of the current robot.
    */
    virtual void setJointOrderInConfig(std::vector<CjrlJoint*> inJointVector);

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
    virtual bool currentConfiguration(const vectorN& inConfig);

    /**
       \brief Get the current configuration of the robot.

       \return the configuration vector \f${\bf q}\f$.
    */
    virtual const vectorN& currentConfiguration() const;

    /**
       \brief Set the current velocity of the robot.  

       \param inVelocity the velocity vector \f${\bf \dot{q}}\f$.

       \return true if success, false if failure (the dimension of the
       input vector does not fit the number of degrees of freedom of the
       robot).
    */
    virtual bool currentVelocity(const vectorN& inVelocity);

    /**
       \brief Get the current velocity of the robot.

       \return the velocity vector \f${\bf \dot{q}}\f$.
    */
    virtual const vectorN& currentVelocity() const;
    /**
       \brief Set the current acceleration of the robot.  

       \param inAcceleration the acceleration vector \f${\bf \ddot{q}}\f$.

       \return true if success, false if failure (the dimension of the
       input vector does not fit the number of degrees of freedom of the
       robot).
    */
    virtual bool currentAcceleration(const vectorN& inAcceleration);

    /**
       \brief Get the current acceleration of the robot.

       \return the acceleration vector \f${\bf \ddot{q}}\f$.
    */
    virtual const vectorN& currentAcceleration() const;

    /**
       \brief Get the current forces of the robot.

       \return the force vector \f${\bf f}\f$.
    */
    virtual const matrixNxP& currentForces() const;

    /**
       \brief Get the current torques of the robot.

       \return the torque vector \f${\bf \tau }\f$.
    */
    virtual const matrixNxP& currentTorques() const;


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
    virtual bool computeForwardKinematics();
  

    /**
       \brief Compute the dynamics of the center of mass.

       Compute the linear and  angular momentum and their time derivatives, at the center of mass.
    */
    virtual bool computeCenterOfMassDynamics() ;

    /**
       \brief Get the position of the center of mass.
    */
    virtual const vector3d& positionCenterOfMass() const;

    /**
       \brief Get the velocity of the center of mass.
    */
    virtual const vector3d& velocityCenterOfMass();

    /**
       \brief Get the acceleration of the center of mass.
    */
    virtual const vector3d& accelerationCenterOfMass();

    /**
       \brief Get the linear momentum of the robot.
    */
    virtual const vector3d& linearMomentumRobot();

    /**
       \brief Get the time-derivative of the linear momentum.
    */
    virtual const vector3d& derivativeLinearMomentum();

    /**
       \brief Get the angular momentum of the robot at the center of mass.
    */
    virtual const vector3d& angularMomentumRobot();

    /**
       \brief Get the time-derivative of the angular momentum at the center of mass.
    */
    virtual const vector3d& derivativeAngularMomentum();

    /**
       \brief Get the total mass of the robot
    */
    virtual double mass() const;

    /**
       @}
    */

    /**
       \name Jacobian fonctions
    */

    /**
       \brief Compute the Jacobian matrix of the center of mass wrt \f${\bf q}\f$.
    */
    virtual void computeJacobianCenterOfMass();
    
    /**
       \brief Get the Jacobian matrix of the center of mass wrt \f${\bf q}\f$.
    */
    virtual const matrixNxP& jacobianCenterOfMass() const;
    
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
    virtual bool isSupported(const std::string &inProperty);

    /**
       \brief Get property corresponding to command name.

       \param inProperty name of the property.
       \retval outValue value of the property if implemented.

       \note The returned string needs to be cast into the right type (double, int,...).
    */
    virtual bool getProperty(const std::string &inProperty, 
			     std::string& outValue);
    
    /**
       \brief Set property corresponding to command name.

       \param inProperty name of the property.
       \param inValue value of the property.

       \note The value string is obtained by writing the 
       corresponding value in a string (operator<<).
    */
    virtual bool setProperty(std::string &inProperty, 
			     const std::string& inValue);


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
    virtual bool getJacobian(const CjrlJoint& inStartJoint, 
			     const CjrlJoint& inEndJoint, 
			     const vector3d& inFrameLocalPosition, 
			     matrixNxP& outjacobian, 
			     unsigned int offset = 0, 
			     bool inIncludeStartFreeFlyer = true);
  
    virtual bool getPositionJacobian(const CjrlJoint& inStartJoint, 
				     const CjrlJoint& inEndJoint, 
				     const vector3d& inFrameLocalPosition, 
				     matrixNxP& outjacobian, 
				     unsigned int offset = 0, 
				     bool inIncludeStartFreeFlyer = true);
  
    virtual bool getOrientationJacobian(const CjrlJoint& inStartJoint, 
					const CjrlJoint& inEndJoint, 
					matrixNxP& outjacobian, 
					unsigned int offset = 0, 
					bool inIncludeStartFreeFlyer = true);

    virtual bool getJacobianCenterOfMass(const CjrlJoint& inStartJoint, 
					 matrixNxP& outjacobian, 
					 unsigned int offset = 0, 
					 bool inIncludeStartFreeFlyer = true);

    /*! \name Inertia matrix related methods 
      @{ */
    /*! \brief Compute the inertia matrix of the robot according wrt \f${\bf q}\f$.
     */
    virtual void computeInertiaMatrix() ;

    /*! \brief Get the inertia matrix of the robot according wrt \f${\bf q}\f$.
     */
    virtual const matrixNxP& inertiaMatrix() const;

    /*! @} */

    /*! \name Actuated joints related methods.  
      @{
    */

    /** 
	\brief Returns the list of actuated joints. 
    */
    virtual const std::vector<CjrlJoint*>& getActuatedJoints() const;

    /**
       \brief Specifies the list of actuated joints. 
    */
    virtual void setActuatedJoints(std::vector<CjrlJoint*>& lActuatedJoints);

    /*! 
      @} 
    */

  protected:
    /**
       \brief Constructor optionally allocating the private part
    */
    DynamicMultiBody(bool inAllocatePrivate);
 };
};

#endif /* DYNAMICSJRLJAPAN_DYNAMICMULTIBODY_H_ */
