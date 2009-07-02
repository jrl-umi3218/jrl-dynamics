/* Class to implement a Joint object.

   Copyright (c) 2007, 
   @author Olivier Stasse, Florent Lamiraux, Francois Keith
   
   JRL-Japan, CNRS/AIST

   All rights reserved.

   Please refers to file License.txt for details on the license.   
*/

#ifndef JOINTPRIVATE_H
#define JOINTPRIVATE_H

#include <vector>

#include "MatrixAbstractLayer/MatrixAbstractLayer.h"
#include "robotDynamics/jrlJoint.h"
#include "dynamics-config.h"

using namespace std;

namespace dynamicsJRLJapan
{  
    class DynamicBodyPrivate;
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
  class DYN_JRL_JAPAN_EXPORT JointPrivate: public CjrlJoint
  {
  protected:
    /** 
	\brief Whether pose was specified in global frame or not 
	If true, the position of the joint has been defined in the global frame. 
	By convention, the axis of the joint is X-axis in joint frame.
    */
    bool m_inGlobalFrame;

    /**
       \brief Position of the joint in the global frame at construction (joint value is equal to 0).
    */
    matrix4d m_globalPoseAtConstruction;
    

    

  private:
      
      /** */
      double attSTcoef;
      /** */
      unsigned int attId, attNumberLifted;
      /** */
      vector3d attSTmcom;
      
      
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
    JointPrivate * m_FatherJoint;

    /*! Vector of childs */
    std::vector< JointPrivate*> m_Children;

    /*! Vector of joints from the root to this joint. */
    std::vector< CjrlJoint*> m_FromRootToThis;
    std::vector<JointPrivate*> m_FromRootToThisJoint;

    /*! Pointer towards the body. */
    CjrlBody * m_Body;
    DynamicBodyPrivate * m_dynBody;

    /*! Name */
    string m_Name;

    /*! Identifier in the VRML file. */
    int m_IDinActuated;

    /*! Limits of the joint. */
    std::vector<double> m_LowerLimits;
    std::vector<double> m_UpperLimits;

    /*! Limits of the joint velocity. */
    std::vector<double> m_LowerVelocityLimits;
    std::vector<double> m_UpperVelocityLimits;

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
      
      /** */
      double subTreeCoef();
      /** */
      void subTreeCoef(double inReplacement);
      /** */
      void computeSubTreeMCom();
      /** */
      void computeSubTreeMComExceptChild(const CjrlJoint* inJoint);
      /** */
      const vector3d& subTreeMCom() const;
      /** */
      void subTreeMCom(const vector3d& inReplacement);
    
    /*! \brief Static constant to define the kind of joints
      available */
    static const int FREE_JOINT=-1;
    static const int FIX_JOINT=0;
    static const int REVOLUTE_JOINT=1;
    static const int PRISMATIC_JOINT=2;
    
    /*! \brief Constructor with full initialization. */
    JointPrivate(int ltype, vector3d&  laxe, 
	  float lquantite, matrix4d & apose);

    JointPrivate(int ltype, vector3d& laxe, 
	  float lquantite, vector3d &translationStatic);
    
    JointPrivate(int ltype, vector3d& laxe, 
	  float lquantite);

	
    /*! \brief Constructor by copy. */
    JointPrivate(const JointPrivate &r); 
    
    /*! \brief Default constructor. */ 
    JointPrivate();

    /*! \brief default destructor */
    virtual ~JointPrivate();
    
    /*! \brief Affectation operator */
    JointPrivate & operator=(const JointPrivate &r);

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
    inline void setIDinActuated(int IDinActuated)
      { m_IDinActuated = IDinActuated;}

    /*! Get the Identifier of the joint insidie
      the VRML file. */
    inline const int & getIDinActuated() const
      { return m_IDinActuated;}

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
    /*! \name JointPrivate hierarchy 
      @{
     */
    /*! \brief parent JointPrivate */
    CjrlJoint*  parentJoint() const ;

    /*! \brief Add a child JointPrivate */
    bool addChildJoint(CjrlJoint&);
    
    /*! \brief Count the number of child joints */
    unsigned int countChildJoints() const;
    
    /*! \brief Returns the child joint at the given rank */
    CjrlJoint* childJoint(unsigned int givenRank) const;
    JointPrivate* child_JointPrivate(unsigned int givenRank) const;

    /**
    ! \brief Get a vector containing references of the joints 
    between the rootJoint and this joint. 
    The root JointPrivate and this JointPrivate are included in the vector.
     */
    std::vector< CjrlJoint* > jointsFromRootToThis() const ;
    
    std::vector< JointPrivate* > jointsFromRootToThisJoint() const ;
    /*! @} */
    
    /*! \name JointPrivate Kinematics 
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
       \brief Returns the rank of the JointPrivate in the state vector.
       If the JointPrivate has several dimensions, it is the rank of the first dimension.
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
    DynamicBodyPrivate* linkedDBody() const;
 	
    /**
       \brief Link a body to the joint.
    */
    void setLinkedBody (CjrlBody& inBody);
  
    /**
       @}
    */
    
    /*! @} */

    /*! Specify the joint father. */
    void SetFatherJoint(JointPrivate *aFather);

    /*! Set the state vector position. */
    inline const unsigned int & stateVectorPosition() const
      { return m_StateVectorPosition; }
    
    
    inline void stateVectorPosition(unsigned aStateVectorPosition)
      { m_StateVectorPosition = aStateVectorPosition;}

  };

  class JointFreeflyerPrivate : public JointPrivate
  {
  public:
    JointFreeflyerPrivate(const matrix4d &inInitialPosition);
    virtual ~JointFreeflyerPrivate();
  };

  class JointRotationPrivate : public JointPrivate
  {
  public:
    JointRotationPrivate(const matrix4d &inInitialPosition);
    virtual ~JointRotationPrivate();
  };

  class JointTranslationPrivate : public JointPrivate
  {
  public:
    JointTranslationPrivate(const matrix4d &inInitialPosition);
    virtual ~JointTranslationPrivate();
  };

  class  JointAnchorPrivate : public JointPrivate
  {
  public:
    JointAnchorPrivate(const matrix4d &inInitialPosition);
    virtual ~JointAnchorPrivate();
  };
};
#endif /* JOINTPRIVATE_H */
