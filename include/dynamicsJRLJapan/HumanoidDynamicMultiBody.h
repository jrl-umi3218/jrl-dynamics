/* Computation of the dynamic aspect for a humanoid robot.
  
Copyright (c) 2005-2006, 
@author Francois Keith, Olivier Stasse. 
   
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
#include "robotDynamics/jrlJoint.h"
#include "robotDynamics/jrlDynamicRobot.h"
#include "robotDynamics/jrlHumanoidDynamicRobot.h"

#include "dynamics-config.h"

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
  class DYN_JRL_JAPAN_EXPORT HumanoidDynamicMultiBody: public CjrlHumanoidDynamicRobot, public virtual DynamicMultiBody
    {
    private:
      
      /** \brief Store the Left Wrist Joint */
      CjrlJoint *m_LeftWristJoint;
      
      /** \brief Store the Right Wrist Joint */
      CjrlJoint *m_RightWristJoint;
      
      /** \brief Store the Left Foot Joint */
      CjrlJoint *m_LeftFootJoint;
      
      /** \brief Store the Right Foot Joint */
      CjrlJoint *m_RightFootJoint;
      
      /** \brief Store the Left hand */
      CjrlHand*  m_leftHand;
      
      /** \brief Store the Right hand */
      CjrlHand*  m_rightHand;
      
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
      HumanoidDynamicMultiBody(const DynamicMultiBody& inDynamicMultiBody,
			       string aFileNameForHumanoidSpecificities);
      
      /*! Destructor */
      virtual ~HumanoidDynamicMultiBody();

      /** @} */
      
      /*! This method creates the link between the Humanoid Specificities
	 object and the Dynamic MultiBody fields. */
      void LinkBetweenJointsAndEndEffectorSemantic();

      /*! Set Humanoid Specificties file. */
      void SetHumanoidSpecificitiesFile(string &aFileName);

      /*! Get pointer on the information specific to the humanoid */
      inline HumanoidSpecificities * getHumanoidSpecificities() const
	{return m_HS;};

#if 0
      /*! Get pointer on the CjrlDynamicRobot used as a proxy. */
      inline CjrlDynamicRobot *getDynamicMultiBody() const
	{return this;}
#endif

      /** \name jrlHumanoidDynamicRobot Interface */
      
      /**
	 \name Joints specific to humanoid robots
      */
      
      /**
	\brief Set the pointer to the Waist joint
      */
      void waist(CjrlJoint* inWaist);

      /**
	 \brief Get a pointer to the Waist
      */
      CjrlJoint* waist();

      /**
      \brief Set the pointer to the left Wrist joint.
      */
      inline void leftWrist(CjrlJoint *inLeftWrist)
      { m_LeftWristJoint = inLeftWrist;};
      
      /** 
      \brief Get a pointer to the left Wrist.
      */
      inline CjrlJoint *leftWrist() 
      { return m_LeftWristJoint;}
      
      /**
      \brief Set the pointer to the right Wrist joint.
      */
      inline void rightWrist(CjrlJoint *inRightWrist)
      { m_RightWristJoint = inRightWrist;}
      
      /** 
      \brief Get a pointer to the right Wrist.
      */
      inline CjrlJoint *rightWrist()
      { return m_RightWristJoint;}
      
      /**
        \brief Set the pointer to the right hand
      */
      virtual void rightHand(CjrlHand* inRightHand)
      { m_rightHand = inRightHand;}
  
      /**
        \brief Get a pointer to the right hand
      */
      virtual CjrlHand* rightHand()
      { return m_rightHand;}
  
      /**
        \brief Set the pointer to the left hand
      */
      virtual void leftHand(CjrlHand* inLeftHand)
      { m_leftHand = inLeftHand;}
  
      /**
        \brief Get a pointer to the left hand
      */
      virtual CjrlHand* leftHand()
      { return m_leftHand;}
      
        /**
      \brief Get the hand clench value. This is a scalar value ranging between 0 and 1 which describes the hand clench (0 for open and 1 for closed hand)
      This method is customized for HRP2 with two-jaw hands (parallel mechanism)
         */
      virtual double getHandClench(CjrlHand* inHand);

    /**
      \brief Set the hand clench value. This is a scalar value ranging between 0 and 1 which describes the hand clench (0 for open and 1 for closed hand).
      This method is customized for HRP2 with two-jaw hands (parallel mechanism)
   */
      virtual bool  setHandClench(CjrlHand* inHand, double inClenchingValue);
      
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
      inline void gaze( const vector3d& inVector, const vector3d & inPoint)
	{ m_LineVector = inVector; m_LinePoint = inPoint;};
	
      /**
      \brief Get a point on the gaze straight line
      */
      const vector3d & gazeOrigin() const {return m_LinePoint;}
  
      /**
	 \brief Get the direction of gaze
      */
      const vector3d & gazeDirection() const {return m_LineVector;}
      
      /** 
	  \brief Return the distance between the sole of a foot and its joint center
      */
      double footHeight() const;


      /**
	 \@}
      */

      /** \name Methods related to fixed joints.
	  @{
      */
      
      /**
	 \@}
      */

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
	  \name Forward kinematics and dynamics
      */
  
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
      void FiniteDifferenceStateEstimate(double inTimeStep);
      /**
      \brief Compute kinematics and dynamics following a finite difference scheme and update past values
      */
      void FiniteDifferenceStateUpdate(double inTimeStep);
      
      /**
      \brief Set the robot in the static state described by the given configuration vector.
      */
      void staticState( const vectorN& inConfiguration );
      
      /**
	 \brief Compute forward kinematics.

	 Update the position, velocity and accelerations of each
	 joint wrt \f${\bf {q}}\f$, \f${\bf \dot{q}}\f$, \f${\bf \ddot{q}}\f$.

	 Computes the ZMP position.

      */
      bool computeForwardKinematics() ;

      /**
	 @}
      */

      /** 
	  \name Jacobian fonctions
      */

      /**
	 \brief Get the jacobian of a joint wrt to internal configuration variables assuming a joint is fixed.
	 
	 Fixed joint is first fixed joint in vector.
	 \return true if there is at least one fixed joint, false otherwise.  
      */
      bool jacobianJointWrtFixedJoint(CjrlJoint *inJoint, 
				      matrixNxP & outJacobian);
      
      /**
	 @}
      */

    };

};
#endif
