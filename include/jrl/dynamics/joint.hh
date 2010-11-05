/*
 * Copyright 2006, 2007, 2008, 2009, 2010, 
 *
 * Oussama Kanoun
 * Fumio Kanehiro
 * Francois Keith
 * Florent Lamiraux
 * Olivier Stasse
 *
 * JRL/LAAS, CNRS/AIST
 *
 * This file is part of dynamicsJRLJapan.
 * dynamicsJRLJapan is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * dynamicsJRLJapan is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Lesser Public License for more details.
 * You should have received a copy of the GNU Lesser General Public License
 * along with dynamicsJRLJapan.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef DYNAMICSJRLJAPAN_JOINT_H
#define DYNAMICSJRLJAPAN_JOINT_H

#include "boost/shared_ptr.hpp"
#include "abstract-robot-dynamics/jrljoint.hh"
#include "jrl/dynamics/dll.hh"

/*
  Forward declaration
*/
class CjrlBody;

namespace dynamicsJRLJapan {


  /** \ingroup userclasses
      \brief This class represents a robot joint.
      
      Implements abstract interface CjrlJoint.
  */
  class DYN_JRL_JAPAN_EXPORT Joint : public virtual CjrlJoint
  {
  public:

    boost::shared_ptr<CjrlJoint> m_privateObj;

    /**
       \name Constructor and destructor
     */
    virtual ~Joint() {};

    Joint(const Joint& inJoint);

  protected:
    Joint();

  public:

    /**
       @}
    */
    /**
       \name Joint hierarchy
       @{
    */

    /**
       \brief Get a pointer to the parent joint (if any).
    */
    virtual CjrlJoint* parentJoint() const;

    /**
       \brief Add a child joint.
    */
    virtual bool addChildJoint (CjrlJoint& inJoint);

    /**
       \brief Get the number of children.
    */
    virtual unsigned int countChildJoints() const;

    /**
       \brief  	Returns the child joint at the given rank.
    */
    virtual CjrlJoint* childJoint(unsigned int inJointRank) const;

    /**
       \brief Get a vector containing references of the joints between the rootJoint and this joint. The root Joint and this Joint are included in the vector.
    */
    virtual std::vector<CjrlJoint*> jointsFromRootToThis() const;

    /**
    \brief Get the rank of this joint in the robot configuration vector.
    If the Joint has several degrees of freedom, it is the rank of the first degree of freedom.
     */
    virtual unsigned int rankInConfiguration() const;

    /**
       @}
    */

    /**
       \name Joint kinematics
       @{
    */

    /**
       \brief Get the initial position of the joint. 

       The initial position of the joint is the position of the local frame of
       the joint.
    */
    virtual const matrix4d& initialPosition();

    /**
    \brief Update this joint's transformation according to degree of freedom value from argument robot configuration. This does not update the transformations of child joints.
    \param inDofVector is a robot configuration vector.
    \return false if argument vector's size is not equal to the robot's number of degrees of freedom
     */
    virtual bool updateTransformation(const vectorN& inDofVector);

    /**
       \brief Get the current transformation of the joint.
       
       The current transformation of the joint is the transformation
       moving the joint from the position in initial configuration to
       the current position. 
       
       The current transformation is determined by the configuration \f${\bf q}\f$ of the robot.
    */
    virtual const matrix4d &currentTransformation() const;

    /**
       \brief Get the velocity \f$({\bf v}, {\bf \omega})\f$ of the joint.

       The velocity is determined by the configuration of the robot and its time derivative: \f$({\bf q},{\bf \dot{q}})\f$.
       
       \return the linear velocity \f${\bf v}\f$ of the origin of the joint frame
       and the angular velocity \f${\bf \omega}\f$ of the joint frame.
    */
    virtual CjrlRigidVelocity jointVelocity();

    /**
       \brief Get the acceleration of the joint.

       The acceleratoin is determined by the configuration of the robot and its first and second time derivative: \f$({\bf q},{\bf \dot{q}}, {\bf \ddot{q}})\f$.
    */
    virtual CjrlRigidAcceleration jointAcceleration();

    /**
       \brief Get the number of degrees of freedom of the joint.
    */
    virtual unsigned int numberDof() const;

    /**
       @}
    */

    /**
       \name Bounds of the degrees of freedom
       @{
    */
    /**
       \brief Get the lower bound of a given degree of freedom of the joint.

       \param inDofRank Id of the dof in the joint
    */
    virtual double lowerBound(unsigned int inDofRank) const;

    /**
       \brief Get the upper bound of a given degree of freedom of the joint.

       \param inDofRank Id of the dof in the joint
    */
    virtual double upperBound(unsigned int inDofRank) const;

    /**
       \brief Set the lower bound of a given degree of freedom of the joint.

       \param inDofRank Id of the dof in the joint
       \param inLowerBound lower bound
    */
    virtual void lowerBound(unsigned int inDofRank, double inLowerBound);

    /**
       \brief Set the upper bound of a given degree of freedom of the joint.

       \param inDofRank Id of the dof in the joint
       \param inUpperBound Upper bound.
    */
    virtual void upperBound(unsigned int inDofRank, double inUpperBound);

    /**
       \brief Get the lower velocity bound of a given degree of freedom of the joint.

       \param inDofRank Id of the dof in the joint
    */
    virtual double lowerVelocityBound(unsigned int inDofRank) const;

    /**
       \brief Get the upper veocity bound of a given degree of freedom of the joint.

       \param inDofRank Id of the dof in the joint
    */
    virtual double upperVelocityBound(unsigned int inDofRank) const;

    /**
       \brief Set the lower velocity bound of a given degree of freedom of the joint.

       \param inDofRank Id of the dof in the joint
       \param inLowerBound lower bound
    */
    virtual void lowerVelocityBound(unsigned int inDofRank, double inLowerBound);

    /**
       \brief Set the upper velocity bound of a given degree of freedom of the joint.

       \param inDofRank Id of the dof in the joint
       \param inUpperBound Upper bound.
    */
    virtual void upperVelocityBound(unsigned int inDofRank, double inUpperBound);

    /**
       @}
    */

    /**
       \name Jacobian functions wrt configuration.
       @{
    */

    /**
       \brief Get the Jacobian matrix of the joint position and orientation wrt the robot configuration.
       Kinematical constraints from interaction with the environment are not taken into account for this computation.

       The corresponding computation can be done by the robot for each of its joints or by the joint.
       
       \return a matrix \f$J \in {\bf R}^{6\times n_{dof}}\f$ defined by 
       \f[
       J = \left(\begin{array}{llll}
       {\bf v_1} & {\bf v_2} & \cdots & {\bf v_{n_{dof}}} \\
       {\bf \omega_1} & {\bf \omega_2} & \cdots & {\bf \omega_{n_{dof}}}
       \end{array}\right)
       \f]
       where \f${\bf v_i}\f$ and \f${\bf \omega_i}\f$ are respectively the linear and angular velocities of the joint 
       implied by the variation of degree of freedom \f$q_i\f$. The velocity of the joint returned by 
       CjrlJoint::jointVelocity can thus be obtained through the following formula:
       \f[
       \left(\begin{array}{l} {\bf v} \\ {\bf \omega}\end{array}\right) = J {\bf \dot{q}}
       \f]
    */
    virtual const matrixNxP& jacobianJointWrtConfig() const;

    /**
        \brief Compute the joint's jacobian wrt the robot configuration.
     */
    virtual void computeJacobianJointWrtConfig();

    /**
        \brief Get the jacobian of the point specified in local frame by inPointJointFrame.
    The output matrix outjacobian is automatically resized if necessary

     */
    virtual void getJacobianPointWrtConfig(const vector3d& inPointJointFrame, matrixNxP& outjacobian) const;

    /**
       @}
    */

    /**
       \name Body linked to the joint
       @{
    */

    /**
       \brief Get a pointer to the linked body (if any).
    */
    virtual CjrlBody* linkedBody() const;

    /**
       \brief Link a body to the joint.
    */
    virtual void setLinkedBody (CjrlBody& inBody);

    /**
       @}
    */

  };

  /**
     \brief Free flyer joint
  */
  class DYN_JRL_JAPAN_EXPORT JointFreeflyer : public Joint 
  {
  public:
    JointFreeflyer(const matrix4d& inInitialPosition);
  };

  /**
     \brief Rotation joint
  */
  class DYN_JRL_JAPAN_EXPORT JointRotation : public Joint 
  {
  public:
    JointRotation(const matrix4d& inInitialPosition);
  };

  /**
     \brief Translation joint
  */
  class DYN_JRL_JAPAN_EXPORT JointTranslation : public Joint 
  {
  public:
    JointTranslation(const matrix4d& inInitialPosition);
  };

  /**
     \brief Anchor joint
  */
  class DYN_JRL_JAPAN_EXPORT JointAnchor : public Joint 
  {
  public:
    JointAnchor(const matrix4d& inInitialPosition);
  };

};


#endif /* DYNAMICSJRLJAPAN_JOINT_H */
