/* Computation of the dynamic aspect for a humanoid robot.
  
Copyright (c) 2005-2006, 
@author Olivier Stasse.
   
JRL-Japan, CNRS/AIST

All rights reserved.
   
Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:
   
* Redistributions of source code must retain the above copyright notice, 
this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice, 
this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
* Neither the name of the CNRS and AIST nor the names of its contributors 
may be used to endorse or promote products derived from this software without specific prior written permission.
   
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS 
OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY 
AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER 
OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, 
OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS 
OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef _HUMANOID_DYNAMIC_MULTIBODY_H_
#define _HUMANOID_DYNAMIC_MULTIBODY_H_
#include <vector>

#include "HumanoidSpecificities.h"

#include "MatrixAbstractLayer/MatrixAbstractLayer.h"
#include <robotDynamics/jrlJoint.h>
#include <robotDynamics/jrlDynamicRobot.h>
#include <robotDynamics/jrlHumanoidDynamicRobot.h>


namespace dynamicsJRLJapan
{

  struct HumanoidDMBComparison
  {
    bool operator()(int a,int b) const
    {
      return a < b;
    }
  };

  /** \brief This class implements the functionnalities specific to a dynamic model for a humanoid.
      
  This includes a direct access to the joints reprensenting the hands, the foot, and the gaze.
  This specific class is the specialization of the generic class CjrlHumanoidDynamicRobot.

  */
  class HumanoidDynamicMultiBody: public CjrlHumanoidDynamicRobot
    {
    private:
      
      /** \brief Store the implementation of Dynamic Multi Body */
      CjrlDynamicRobot *m_DMB;

      /** \brief Store the Left Hand Joint */
      CjrlJoint *m_LeftHandJoint;
      
      /** \brief Store the Right Hand Joint */
      CjrlJoint *m_RightHandJoint;
      
      /** \brief Store the Left Foot Joint */
      CjrlJoint *m_LeftFootJoint;
      
      /** \brief Store the Right Foot Joint */
      CjrlJoint *m_RightFootJoint;
      
      /** \brief Store the Gaze Joint */
      CjrlJoint *m_GazeJoint;

      /** \brief Store the Waist Joint */
      CjrlJoint *m_WaistJoint;

      /** \name Gaze related store */

      /** \brief Set the direction of the line. */
      vector3d m_LineVector;

      /** \brief Set the point through which the line is going. */
      vector3d m_LinePoint;
      
      /** @} */
      
      /** \brief Vector of fixed joints. */
      std::vector<CjrlJoint*> m_VectorOfFixedJoints;

      /*! Object to store the specificities to an instance of a humanoid robot. */
      HumanoidSpecificities *m_HS;

      /*! Distance between the ankle and the soil. */
      double m_AnkleSoilDistance;

      /*! Distance between some axis of the hip. */
      vector3d m_Dt;

      /*! \name Parameters related to the distance
	between the CoM and the hip. 
	@{
      */
      
      /*! Static translation from the CoM to the left
       hip. */
      vector3d m_StaticToTheLeftHip,
	m_StaticToTheRightHip;

      /*! Static translation from the CoM to the right
       hip. */
      vector3d m_TranslationToTheRightHip,
	m_TranslationToTheLeftHip;

      vector3d m_ZeroMomentumPoint;
      /*! @} */

    public:

      /** \name Constructors and destructors 
	  @{
      */
      /*! Constructor */
      
      /*! Default constructor: assume that a file name HumanoidSpecificities.xml 
	 is present in the current directory, and creates a Dynamic Multi Body 
	 object instead of a more generic multibody.
	 Good for use by default.
      */
      HumanoidDynamicMultiBody(void);

      /*! Advanced constructor: */
      HumanoidDynamicMultiBody(CjrlDynamicRobot * aDMB,
			       string aFileNameForHumanoidSpecificities);
      
      /*! Destructor */
      virtual ~HumanoidDynamicMultiBody();

      /** @} */
      
      /**! This method creates the link between the Humanoid Specificities
	 object and the Dynamic MultiBody fields. */
      void LinkBetweenJointsAndEndEffectorSemantic();

      /**! This method is a proxy for a Dynamic Multi Body object 
	 It gives an index transformation from VRMLID to Configuration.
       */
      void GetJointIDInConfigurationFromVRMLID(std::vector<int>  & aVector);

      /*! Set Humanoid Specificties file. */
      void SetHumanoidSpecificitiesFile(string &aFileName);

      /*! Get pointer on the information specific to the humanoid */
      inline HumanoidSpecificities * getHumanoidSpecificities() const
	{return m_HS;};

      /*! Get pointer on the CjrlDynamicRobot used as a proxy. */
      inline CjrlDynamicRobot *getDynamicMultiBody() const
	{return m_DMB;}

      /** \name jrlHumanoidDynamicRobot Interface */
      
      /**
	 \name Joints specific to humanoid robots
      */
      
      /**
	 \brief Set the pointer to the left hand joint.
      */
      inline void leftHand(CjrlJoint *inLeftHand)
	{ m_LeftHandJoint = (Joint *)inLeftHand;};
      
      /** 
	  \brief Get a pointer to the left hand.
      */
      inline CjrlJoint *leftHand() 
	{ return m_LeftHandJoint;}
      
      /**
	 \brief Set the pointer to the right hand joint.
      */
      inline void rightHand(CjrlJoint *inRightHand)
	{ m_RightHandJoint = (Joint *)inRightHand;}
      
      /** 
	  \brief Get a pointer to the right hand.
      */
      inline CjrlJoint *rightHand()
	{ return m_RightHandJoint;}
      
      /**
	 \brief Set the pointer to the left foot joint.
      */
      inline void leftFoot(CjrlJoint *inLeftFoot)
	{ m_LeftFootJoint = (Joint *)inLeftFoot;}
      
      /** 
	  \brief Get a pointer to the left foot.
      */
      inline CjrlJoint *leftFoot() 
	{ return m_LeftFootJoint;};
      
      /**
	 \brief Set the pointer to the right foot joint.
      */
      inline void rightFoot(CjrlJoint *inRightFoot)
	{m_RightFootJoint = (Joint *) inRightFoot;}
      
      /** 
	  \brief Get a pointer to the right foot.
      */
      inline CjrlJoint *rightFoot()
	{return m_RightFootJoint;}
      
      /** 
	  \brief Set gaze joint
	  
	  \note  For most humanoid robots, the gaze joint is the head.
      */
      inline void gazeJoint(CjrlJoint *inGazeJoint)
	{ m_GazeJoint = (Joint *)inGazeJoint; }
      
      /**
	 \brief Get gaze joint
      */
      CjrlJoint *gazeJoint()
	{ return m_GazeJoint; }
      
      /**
	 \brief Set the gaze in the local frame of the gaze joint.
	 
	 \note The gaze is defined as a straight line linked to the gaze joint.
	 @param inVector: A 3D vector which define the direction of the gaze.
	 @param inPoint: A 3D point by which the line of direction \a inVector
	 goes through.

	 Those two paramaters defines a line in the head reference frame defining
	 a gaze direction.

      */
      inline void gaze( vector3d& inVector, vector3d & inPoint)
	{ m_LineVector = inVector; m_LinePoint = inPoint;};

      inline void gaze( const vector3d& inVector)
	{ m_LineVector = inVector;};
      
      /** 
	  \brief Get the gaze orientation in the local frame of the gaze joint.
	  
	  \note Returns the line along which the head is oriented.
	  @return outVector: The direction of the line.
	  @return outPoint: The point by which the line is going through in the head reference frame.

      */
      inline void gaze(vector3d & outVector, vector3d & outPoint ) const
	{ outVector = m_LineVector; outPoint = m_LinePoint;};
	
      /**
      \brief Get a point on the gaze straight line
       */
       const vector3d & gazeOrigin() const {return m_LinePoint;}
  
  /**
      \brief Get the direction of gaze
   */
      const vector3d & gazeDirection() const {return m_LineVector;}
      
      /**
	 \@}
      */

      /** \name Methods related to fixed joints.
	  @{
      */
      /** 
	  \brief Add a joint to the vector of fixed joints.
	  This Declares a joint as fixed in the world.
      */
      void addFixedJoint(CjrlJoint *inFixedJoint) ;
      
      /** 
	  \brief Count joints that are fixed in the world.
      */
      unsigned int countFixedJoints() const;
      
      /** 
	  \brief Remove a joint from the vector of fixed joints.
	  The input joint will no longer be considered fixed in the world.
      */
      void removeFixedJoint(CjrlJoint *inFixedJoint);
      
      /** 
            \brief Clear the list of fixed joints
        */
       void clearFixedJoints();
       
       /** 
	   \brief Return the fixed joint at rank inRank 
       */
       CjrlJoint& fixedJoint(unsigned int inJointRank);

      
      /**
	 \brief Get the jacobian of a joint wrt to internal configuration variables assuming a joint is fixed.
	 
	 Fixed joint is first fixed joint in vector.
	 \return true if there is at least one fixed joint, false otherwise.  
      */
      bool jacobianJointWrtFixedJoint(CjrlJoint *inJoint, 
				      matrixNxP & outJacobian);
      
      /** 
	  \brief Return the distance between the sole of a foot and its joint center
      */
      double footHeight() const;


      /** @} */
      /**
	 \name Zero momentum point
      */
      
      /**
	 \brief Compute the coordinates of the Zero Momentum Point.
      */
      const vector3d & zeroMomentumPoint() const;
      
      /** 
	  \brief Trigger the computation of the Zero Momentum Point.
      */
      void ComputingZeroMomentumPoint();

      /**
	 @}
      */
      
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
      void rootJoint(CjrlJoint& inJoint);
     
      
      /**
	 \brief Get the root joint of the robot.
      */
      CjrlJoint* rootJoint() const ;
      
      /**
	 \brief Get a vector containing all the joints.
      */
      std::vector<CjrlJoint*> jointVector() ;
      
      /**
	 \brief Get the number of degrees of freedom of the robot.
      */
      unsigned int numberDof() const ;

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
      bool currentConfiguration(const vectorN& inConfig) ;

      /**
	 \brief Get the current configuration of the robot.

	 \return the configuration vector \f${\bf q}\f$.
      */
      const vectorN& currentConfiguration() const ;

      /**
	 \brief Set the current velocity of the robot.  

	 \param inVelocity the velocity vector \f${\bf \dot{q}}\f$.

	 \return true if success, false if failure (the dimension of the
	 input vector does not fit the number of degrees of freedom of the
	 robot).
      */
      bool currentVelocity(const vectorN& inVelocity) ;

      /**
	 \brief Get the current velocity of the robot.

	 \return the velocity vector \f${\bf \dot{q}}\f$.
      */
      const vectorN& currentVelocity() const ;
      /**
	 \brief Set the current acceleration of the robot.  

	 \param inAcceleration the acceleration vector \f${\bf \ddot{q}}\f$.

	 \return true if success, false if failure (the dimension of the
	 input vector does not fit the number of degrees of freedom of the
	 robot).
      */
      bool currentAcceleration(const vectorN& inAcceleration) ;

      /**
	 \brief Get the current acceleration of the robot.

	 \return the acceleration vector \f${\bf \ddot{q}}\f$.
      */
      const vectorN& currentAcceleration() const ;

      /**
	 @}
      */

      /** 
	  \name Forward kinematics and dynamics
      */
  
   /**
      \brief Apply a configuration

    Based on the entered configuration, this method computes:
    for every joint:
      the new transformation
      the new position of the center of mass in world frame
      for the robot
      position of the center of mass in world frame
        
      \return true if success, false if failure (the dimension of the
      input vector does not fit the number of degrees of freedom of the
      robot).
    */
      bool applyConfiguration(const vectorN& inConfiguration);
    /**
      \brief Compute kinematics and dynamics following a finite difference scheme.

    Based on previously stored values, this method computes:
    for every joint:
      linear velocity and acceleration
      angular velocity and acceleration
      linear momentum
      angular momentum
    for the robot
      linear momentum
      angular momentum
      ZMP

     */
      void FiniteDifferenceStateUpdate(double inTimeStep);
      
      /**
      \brief Set the robot in the static state described by the given configuration vector.
       */
      void staticState(const vectorN& inConfiguration);
      
      /**
	 \brief Compute forward kinematics.

	 Update the position, velocity and accelerations of each
	 joint wrt \f${\bf {q}}\f$, \f${\bf \dot{q}}\f$, \f${\bf \ddot{q}}\f$.

      */
      bool computeForwardKinematics() ;

      /**
	 \brief Compute the dynamics of the center of mass.

	 Compute the linear and  angular momentum and their time derivatives, at the center of mass.
      */
      bool computeCenterOfMassDynamics() ;

      /**
	 \brief Get the position of the center of mass.
      */
      const vector3d& positionCenterOfMass() ;

      /**
	 \brief Get the velocity of the center of mass.
      */
      const vector3d& velocityCenterOfMass() ;

      /**
	 \brief Get the acceleration of the center of mass.
      */
      const vector3d& accelerationCenterOfMass() ;

      /**
	 \brief Get the linear momentum of the robot.
      */
      const vector3d& linearMomentumRobot() ;
  
      /**
	 \brief Get the time-derivative of the linear momentum.
      */
      const vector3d& derivativeLinearMomentum() ;

      /**
	 \brief Get the angular momentum of the robot at the center of mass.
      */
      const vector3d& angularMomentumRobot() ;
  
      /**
	 \brief Get the time-derivative of the angular momentum at the center of mass.
      */
      const vector3d& derivativeAngularMomentum() ;

      /**
	 @}
      */

      /** 
	  \name Jacobian fonctions
      */

      /**
	 \brief Compute the Jacobian matrix of the center of mass wrt \f${\bf q}\f$.
      */
      void computeJacobianCenterOfMass() ;

      /**
	 \brief Get the Jacobian matrix of the center of mass wrt \f${\bf q}\f$.
      */
      const matrixNxP& jacobianCenterOfMass() const ;

      void waist(CjrlJoint* inWaist);

      double mass() const;
      
      CjrlJoint* waist();

	
      /**
	 @}
      */

    };
};
#endif
