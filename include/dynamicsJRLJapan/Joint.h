/* Class to implement a Joint object.

   Copyright (c) 2007, 
   @author Olivier Stasse, Florent Lamiraux
   
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

      \note Two ways of constructing a kinematic chain are supported.
        \li through VRML parser VRMLReader::ParseVRMLFile
        \li through abstract robot dynamics interfaces. When using this solution, 
	the joints should be inserted in the kinematic tree with increasing depth. For instance, in chain J1 -> J2 -> J3, J2 should be inserted as J1 child before J3 is inserted as J2 child.
  */
  class Joint: public CjrlJoint
  {
  protected:
    /** 
	\brief Whether pose was specified in global frame or not 
	If true, the position of the joint has been defined in the global frame. By convention, the axis of the joint is X-axis in joint frame.
    */
    bool m_inGlobalFrame;

    /**
       \brief Position of the joint in the global frame at construction (joint value is equal to 0).
    */
    matrix4d m_globalPoseAtConstruction;

  private:
    /*!  Type of the transformation */ 
    int m_type;
    
    /*! Axis of the transformation,
      for the link with one DoF. */
    vector3d m_axe;	
    
    /*! Quantity of the rotation . */
    float m_quantity;

    /*! 
      \brief 4x4 matrix for pose 
      This homogeneous matrix represents the position of this joint frame in the frame of the parent joint.
    */
    matrix4d m_poseInParentFrame;

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

    /*! Limits of the joint. */
    vector<double> m_LowerLimits;
    vector<double> m_UpperLimits;

    /*! Limits of the joint velocity. */
    vector<double> m_LowerVelocityLimits;
    vector<double> m_UpperVelocityLimits;

    double m_EquivalentInertia;

    /*! Rigid Velocity */
    CjrlRigidVelocity m_RigidVelocity;
    
    /*! Jacobian with respect to the current configuration */
    matrixNxP m_J;

    /*! First entry into the state vector */
    unsigned int m_StateVectorPosition;
    
    /*! Create the arrays (when the type is known). */
    void CreateLimitsArray();

    /**
       \brief Compute the pose of the joint in the global frame or in local frame of parent
       The relation between poses in the global frame and in local frame of parent joint is the following:
       \f[
       R^{global}_{joint} = R^{global}_{parent} R^{parent}_{joint}
       \f]
       where
       \li \f$R^{global}_{joint}\f$ is the pose of the joint in global frame,
       \li \f$R^{global}_{parent}\f$ is the pose of the parent joint in global frame,
       \li \f$R^{parent}_{joint}\f$ is the pose of the joint in parent joint local frame.

       If the pose of the joint has been defined in global frame at construction, the local pose in parent frame is computed. 
       If the pose of the joint has been defined in local frame of parent joint at construction, the global pose is computed.
    */
    void computeLocalAndGlobalPose();

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
    virtual ~Joint();
    
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
      {return m_poseInParentFrame;};

    /*! Sets the matrix corresponding to the rigid motion */
    inline void pose(const matrix4d & pose ) 
      {m_poseInParentFrame=pose;};
    
    /*! Operator to access the rotation matrix */
    inline double & operator()(unsigned int i) 
      {return MAL_S4x4_MATRIX_ACCESS_I_J(m_poseInParentFrame,i/4,i%4);}

    /*! Operator to access the rotation matrix */
    inline double & operator()(unsigned int i,
			       unsigned int j) 
      {return MAL_S4x4_MATRIX_ACCESS_I_J(m_poseInParentFrame,i,j);}
    
    
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

    /*! Get the static translation. */
    inline void getStaticTranslation(vector3d & staticTranslation) 
      { staticTranslation(0) = MAL_S4x4_MATRIX_ACCESS_I_J(m_poseInParentFrame,0,3);
	staticTranslation(1) = MAL_S4x4_MATRIX_ACCESS_I_J(m_poseInParentFrame,1,3);
	staticTranslation(2) = MAL_S4x4_MATRIX_ACCESS_I_J(m_poseInParentFrame,2,3); }

    /*! Set the static translation. */
    inline void setStaticTranslation(vector3d & staticTranslation) 
      { MAL_S4x4_MATRIX_ACCESS_I_J(m_poseInParentFrame,0,3) =staticTranslation(0);
	MAL_S4x4_MATRIX_ACCESS_I_J(m_poseInParentFrame,1,3) = staticTranslation(1);
	MAL_S4x4_MATRIX_ACCESS_I_J(m_poseInParentFrame,2,3) = staticTranslation(2) ; }

    /*! Get the static rotation. */
    inline void getStaticRotation(matrix3d & staticRotation) 
      { for (int i=0; i<3; i++){
	  for (int j=0; j<3; j++){
	    MAL_S3x3_MATRIX_ACCESS_I_J(staticRotation,i,j) 
	      = MAL_S4x4_MATRIX_ACCESS_I_J(m_poseInParentFrame, i,j);
	  }
	}
      }

    /*! Set the static rotation. */
    inline void setStaticRotation(matrix3d & staticRotation) 
      { for (int i=0; i<3; i++){
	  for (int j=0; j<3; j++){
	    MAL_S4x4_MATRIX_ACCESS_I_J(m_poseInParentFrame,i,j) 
	      = MAL_S3x3_MATRIX_ACCESS_I_J(staticRotation, i,j);
	  }
	}
      }

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
    CjrlJoint*  parentJoint() const ;

    /*! \brief Add a child Joint */
    bool addChildJoint(CjrlJoint&);
    
    /*! \brief Count the number of child joints */
    unsigned int countChildJoints() const;
    
    /*! \brief Returns the child joint at the given rank */
    CjrlJoint* childJoint(unsigned int givenRank) const;

    /**
    ! \brief Get a vector containing references of the joints 
    between the rootJoint and this joint. 
    The root Joint and this Joint are included in the vector.
     */
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
    inline double lowerBound(unsigned int inDofRank) const
    {
      return m_LowerLimits[inDofRank];
    };

    /**
       \brief Get the upper bound of a given degree of freedom of the joint.

       \param inDofRank Id of the dof in the joint
    */
    inline double upperBound(unsigned int inDofRank) const
    {
      return m_UpperLimits[inDofRank];
    };

    /**
       \brief Set the lower bound of a given degree of freedom of the joint.

       \param inDofRank Id of the dof in the joint
       \param inLowerBound lower bound
    */
    inline void lowerBound(unsigned int inDofRank, double inLowerBound) 
    {
      m_LowerLimits[inDofRank] = inLowerBound;
    };

    /**
       \brief Set the upper bound of a given degree of freedom of the joint.

       \param inDofRank Id of the dof in the joint
       \param inUpperBound Upper bound.
    */
    inline void upperBound(unsigned int inDofRank, double inUpperBound)
    {
      m_UpperLimits[inDofRank] = inUpperBound;
    };

    /**
       \brief Set the upper velocity bound of a given degree of freedom of the joint.

       \param inDofRank Id of the dof in the joint
       \param inUpperVelocityBound Upper bound.
    */
    inline void upperVelocityBound(unsigned int inDofRank, double inUpperVelocityBound)
    {
      m_UpperVelocityLimits[inDofRank] = inUpperVelocityBound;
    };

    /**
       \brief Get the lower velocity bound of a given degree of freedom of the joint.

       \param inDofRank Id of the dof in the joint
    */
    inline double lowerVelocityBound(unsigned int inDofRank) const
    {
      return m_LowerVelocityLimits[inDofRank];
    };

    /** \brief Returns the equivalent inertia */
    inline double equivalentInertia() const
      {
	return m_EquivalentInertia;
      }

    /** \brief Set the equivalent inertia */
    inline void equivalentInertia(double &lequivalentInertia) 
      {
	m_EquivalentInertia = lequivalentInertia;
      }

    /**
       \brief Get the upper velocity bound of a given degree of freedom of the joint.

       \param inDofRank Id of the dof in the joint
    */
    inline double upperVelocityBound(unsigned int inDofRank) const
    {
      return m_UpperVelocityLimits[inDofRank];
    };

    /**
       \brief Set the lower velocity bound of a given degree of freedom of the joint.

       \param inDofRank Id of the dof in the joint
       \param inLowerVelocityBound lower bound
    */
    inline void lowerVelocityBound(unsigned int inDofRank, double inLowerVelocityBound) 
    {
      m_LowerVelocityLimits[inDofRank] = inLowerVelocityBound;
    };

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
    The output matrix outjacobian is automatically resized if necessary

     */
    void getJacobianPointWrtConfig(const vector3d& inPointJointFrame, matrixNxP& outjacobian) const;

    /**
        \brief Get the jacobian of the point specified in world frame by inPointWorldFrame.
	The output matrix outjacobian must have appropriate size.

     */
    void getJacobianWorldPointWrtConfig(const vector3d& inPointWorldFrame, 
					matrixNxP& outjacobian) const;

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
  public:
    JointFreeflyer(const matrix4d &inInitialPosition);
    virtual ~JointFreeflyer();
  };

  class JointRotation : public Joint
  {
  public:
    JointRotation(const matrix4d &inInitialPosition);
    virtual ~JointRotation();
  };

  class JointTranslation : public Joint
  {
  public:
    JointTranslation(const matrix4d &inInitialPosition);
    virtual ~JointTranslation();
  };

  class JointAnchor : public Joint
  {
  public:
    JointAnchor(const matrix4d &inInitialPosition);
    virtual ~JointAnchor();
  };
};
#endif
