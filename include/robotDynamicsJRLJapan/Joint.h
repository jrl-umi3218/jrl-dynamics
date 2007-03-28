/* Class to implement a Joint object.

   Copyright (c) 2007, 
   @author Olivier Stasse
   
   JRL-Japan, CNRS/AIST

   All rights reserved.
   
   Redistribution and use in source and binary forms, with or without modification, 
   are permitted provided that the following conditions are met:
   
   * Redistributions of source code must retain the above copyright notice, 
   this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright notice, 
   this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
   * Neither the name of the <ORGANIZATION> nor the names of its contributors 
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

#ifndef _JRLCIRDYNAMICS_JOINT_JRL_JAPAN_H_
#define _JRLCIRDYNAMICS_JOINT_JRL_JAPAN_H_

#include <vector>

#include "MatrixAbstractLayer/MatrixAbstractLayer.h"
#include "robotDynamics/jrlJoint.h"


using namespace std;

namespace dynamicsJRLJapan
{  

  /** @ingroup forwardynamics
       Define a transformation from a body to another
      Supported type:
      - Rotation around an axis with a quantity (type = ROTATION)
      - Translation of a vector : quantite*axe	(type = TRANSLATION)
      - Rotation through a homogeneous matrix : *rotation (type = FREE_LIBRE)
  */
  class Joint: public CjrlJoint
  {

  private:
    /*!  Type of the transformation */ 
    int m_type;
    
    /*! Axis of the transformation,
      for the link with one DoF. */
    vector3d m_axe;	
    
    /*! Quantity of the rotation . */
    float m_quantity;

    /*! 4x4 matrix for pose */
    matrix4d m_pose;

    /*! Father joint */
    Joint * m_FatherJoint;

    /*! Vector of childs */
    std::vector< CjrlJoint*> m_Children;

    /*! Vector of joints from the root to this joint. */
    std::vector< CjrlJoint*> m_FromRootToThis;

    /*! Pointer towards the body. */
    CjrlBody * m_Body;

    /*! Name */
    string m_Name;

    /*! Identifier in the VRML file. */
    int m_IDinVRML;

    /*! Rigid Velocity */
    CjrlRigidVelocity m_RigidVelocity;
    
    /*! Jacobian with respect to the current configuration */
    matrixNxP m_J;

    /*! First entry into the state vector */
    unsigned int m_StateVectorPosition;

    /*! Limits of the joint. */
    double * m_LowerLimits;
    double * m_UpperLimits;
    
    /*! Create the arrays (when the type is known). */
    void CreateLimitsArray();

  public: 
    
    /*! \brief Static constant to define the kind of joints
      available */
    static const int FREE_JOINT=-1;
    static const int FIX_JOINT=0;
    static const int REVOLUTE_JOINT=1;
    static const int PRISMATIC_JOINT=2;
    
    /*! \brief Constructor with full initialization. */
    Joint(int ltype, vector3d&  laxe, 
	  float lquantite, matrix4d & apose);

    Joint(int ltype, vector3d& laxe, 
	  float lquantite, vector3d &translationStatic);
    
    Joint(int ltype, vector3d& laxe, 
	  float lquantite);

	
    /*! \brief Constructor by copy. */
    Joint(const Joint &r); 
    
    /*! \brief Default constructor. */ 
    Joint();

    /*! \brief default destructor */
    ~Joint();
    
    /*! \brief Affectation operator */
    Joint & operator=(const Joint &r);

    /*! \brief Operator to get one element of the rotation 
      
    \f$
    R= \left[ 
    \begin{matrix}
    r_{0} & r_{1} & r_{2} & r_{3} \\
    r_{4} & r_{5} & r_{6} & r_{7} \\
    r_{8} & r_{9} & r_{10} & r_{11} \\
    r_{12} & r_{13} & r_{14} & r_{15} \\
    \end{matrix}
    \right]
    \f$
    where \f$pose(i) = r_{i}\f$.
     */
    float pose(unsigned r) ;

    /*! Returns the matrix corresponding to the rigid motion */
    inline const matrix4d & pose() const
      {return m_pose;};

    /*! Sets the matrix corresponding to the rigid motion */
    inline void pose(const matrix4d & pose ) 
      {m_pose=pose;};
    
    /*! Operator to access the rotation matrix */
    inline double & operator()(unsigned int i) 
      {return MAL_S4x4_MATRIX_ACCESS_I_J(m_pose,i/4,i%4);}

    /*! Operator to access the rotation matrix */
    inline double & operator()(unsigned int i,
			       unsigned int j) 
      {return MAL_S4x4_MATRIX_ACCESS_I_J(m_pose,i,j);}
    
    
    /*! \name Getter and setter for the parameter 
      @{
     */    
    
    /*! Returns the axe of the rotation. */
    inline const vector3d& axe() const
      { return m_axe; };

    /*! Set the axe of the rotation */
    inline void axe(const vector3d &anaxe)
      { m_axe = anaxe; };

    /*! Quantity of the rotation */
    inline const float & quantity() const
      { return m_quantity; } 

    /*! Set the rotation of the joint */
    inline void quantity(const float & aquantity)
      { m_quantity = aquantity; }

    /*! Returns the type of the joint */
    inline const int & type() const
      { return m_type; } 

    /*! Set the type of the joint */
    inline void type(const int atype) 
      { m_type = atype; } 

    /*! Set the name */
    inline void setName(string & aname)
      { m_Name = aname; }

    /*! Get the name */
    inline const string & getName() const
      { return m_Name;}

    /*! Set the Identifier of the joint inside
      the VRML file. */
    inline void setIDinVRML(int IDinVRML)
      { m_IDinVRML = IDinVRML;}

    /*! Get the Identifier of the joint insidie
      the VRML file. */
    inline const int & getIDinVRML() const
      { return m_IDinVRML;}

    /*! Set Joint Upper Limit for i-th DOF. */
    inline void setJointULimit(double ulimit, int i)
      { m_UpperLimits[i] = ulimit; }

    /*! Set Joint Lower Limit for i-th DOF. */
    inline void setJointLLimit(double llimit, int i)
      { m_LowerLimits[i] = llimit; }
    
    /*! Get Joint Upper Limit for i-th DOF. */
    inline double getJointULimit(int i) const
      { return m_UpperLimits[i];}
    
    /*! Get Joint Lower Limit for i-th DOF. */
    inline double getJointLLimit(int i) const
      { return m_LowerLimits[i]; }
    
    /*! Get the static translation. */
    inline void getStaticTranslation(vector3d & staticTranslation) 
      { staticTranslation(0) = MAL_S4x4_MATRIX_ACCESS_I_J(m_pose,0,3);
	staticTranslation(1) = MAL_S4x4_MATRIX_ACCESS_I_J(m_pose,1,3);
	staticTranslation(2) = MAL_S4x4_MATRIX_ACCESS_I_J(m_pose,2,3); }

    /*! Get the static translation. */
    inline void setStaticTranslation(vector3d & staticTranslation) 
      { MAL_S4x4_MATRIX_ACCESS_I_J(m_pose,0,3) =staticTranslation(0);
	MAL_S4x4_MATRIX_ACCESS_I_J(m_pose,1,3) = staticTranslation(1);
	MAL_S4x4_MATRIX_ACCESS_I_J(m_pose,2,3) = staticTranslation(2) ; }

    /*! Compute pose from a vector x,y,z, Theta, Psi, Phi. */
    void UpdatePoseFrom6DOFsVector(vectorN a6DVector);

    /*! Compute velocity from two vectors (dx,dy,dz) (dTheta, dPsi, dPhi) */
    void UpdateVelocityFrom2x3DOFsVector(vector3d& alinearVelocity,
					 vector3d& anAngularVelocity);
    
    /* @} */

    /*! \name Implements the virtual function inherited from CjrlJoint
      @{
     */
    /*! \name Joint hierarchy 
      @{
     */
    /*! \brief parent Joint */
    CjrlJoint&  parentJoint() const ;

    /*! \brief Add a child Joint */
    bool addChildJoint(const CjrlJoint&);
    
    /*! \brief Count the number of child joints */
    unsigned int countChildJoints() const;
    
    /*! \brief Returns the child joint at the given rank */
    const CjrlJoint& childJoint(unsigned int givenRank) const;

    /*! \brief Joints from root to this joint */
    std::vector< CjrlJoint* > jointsFromRootToThis() const ;
    /*! @} */
    
    /*! \name Joint Kinematics 
      @{
     */
    
    /** \brief Get the initial position of the joint.
	The initial position of the joint is the position
	of the local frame of the joint.
     */
    const matrix4d & initialPosition();
    /**
       \brief Get the current transformation of the joint.
       
       The current transformation of the joint is the transformation
       moving the joint from the position in initial configuration to
       the current position. 
       
       The current transformation is determined by the configuration \f${\bf q}\f$ of the robot.
    */
    const matrix4d &currentTransformation() const;
    
     /**
    \brief Update this joint's transformation according to the given vector of DoF values, and the parent joint's transformation if this is not a free flyer joint.
    \return false if the required number of dof values is not met in given paramter
      */
    virtual bool updateTransformation(const vectorN& inRobotConfigVector);
    
    /**
       \brief Get the velocity \f$({\bf v}, {\bf \omega})\f$ of the joint.
       
       The velocity is determined by the configuration of the robot and its time derivative: \f$({\bf q},{\bf \dot{q}})\f$.
       
       \return the linear velocity \f${\bf v}\f$ of the origin of the joint frame
       and the angular velocity \f${\bf \omega}\f$ of the joint frame.
    */
    CjrlRigidVelocity jointVelocity();
    
    /**
       \brief Get the acceleration of the joint.
       
       The acceleratoin is determined by the configuration of the robot 
       and its first and second time derivative: \f$({\bf q},{\bf \dot{q}}, {\bf \ddot{q}})\f$.
    */
    CjrlRigidAcceleration jointAcceleration();
    
    /**
       \brief Get the number of degrees of freedom of the joint.
    */
    unsigned int numberDof() const;

    /**
       \brief Returns the rank of the Joint in the state vector.
       If the Joint has several dimensions, it is the rank of the first dimension.
    */
    inline unsigned int rankInConfiguration() const
      { return m_StateVectorPosition; }
    
    /*! @} */
    
    /**
       \name Jacobian functions wrt configuration.
       @{
    */

    /**
       \brief Get the Jacobian matrix of the joint position wrt the robot configuration.

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
    const matrixNxP & jacobianJointWrtConfig() const;
    
    /**
       \brief Compute the joint's jacobian wrt the robot configuration.
    */
    void computeJacobianJointWrtConfig();
    
    /**
       \brief Get the jacobian of the point specified in local frame by inPointJointFrame.
       
    */
    matrixNxP jacobianPointWrtConfig(const vector3d & inPointJointFrame) const ;

    /** 
	\brief resize the Jacobian with the number of DOFs.
    */
    void resizeJacobianJointWrtConfig(int lNbDofs);
      
    /**
       @}
    */

    /**
    \brief compute the rotation matrix correponding to axis of rotation inAxis and angle inAngle.
    */
    void RodriguesRotation(vector3d& inAxis, double inAngle, matrix3d& outRotation);
    
    /**
    \brief Convenient variables to avoid online matrix/vector allocation
     */
    matrix3d localR;
    vector3d wn3d, vek;
    vectorN dof6D;
    
    /**
       \name Body linked to the joint
       @{
    */

    /**
       \brief Get a pointer to the linked body (if any).
    */
    CjrlBody* linkedBody() const;
 	
    /**
       \brief Link a body to the joint.
    */
    void setLinkedBody (CjrlBody& inBody);
  
    /**
       @}
    */
    
    /*! @} */

    /*! Specify the joint father. */
    void SetFatherJoint(Joint *aFather);

    /*! Set the state vector position. */
    inline const unsigned int & stateVectorPosition() const
      { return m_StateVectorPosition; }
    
    
    inline void stateVectorPosition(unsigned aStateVectorPosition)
      { m_StateVectorPosition = aStateVectorPosition;}

  };

  class JointFreeflyer : public Joint
  {
    JointFreeflyer(const matrix4d &inInitialPosition);
    bool updateTransformation(const vectorN& inRobotConfigVector);
  };

  class JointRotation : public Joint
  {
    JointRotation(const matrix4d &inInitialPosition);
    bool updateTransformation(const vectorN& inRobotConfigVector);
  };

  class JointTranslation : public Joint
  {
    JointTranslation(const matrix4d &inInitialPosition);
    bool updateTransformation(const vectorN& inRobotConfigVector);
  };

};
#endif
