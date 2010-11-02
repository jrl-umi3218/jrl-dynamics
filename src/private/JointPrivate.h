/*
 * Copyright 2010,
 *
 * Florent Lamiraux
 * Olivier Stasse
 *
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
 *  Research carried out within the scope of the Associated
 *  International Laboratory: Joint Japanese-French Robotics
 *  Laboratory (JRL)
 *
 */

/* Class to implement a Joint object.
*/

#ifndef JOINTPRIVATE_H
#define JOINTPRIVATE_H

#include <vector>
#include <Debug.h>

#include "jrl/mal/matrixabstractlayer.hh"
#include "abstract-robot-dynamics/joint.hh"

#include "Spatial.h"

using namespace std;

namespace dynamicsJRLJapan
{
  class DynamicBodyPrivate;
  /** @ingroup forwardynamics
      \class JointPrivate
  */
  class JointPrivate: public CjrlJoint
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

    /**
       \brief Normalized position of the joint in the global frame at construction (joint value is equal to 0).
    */
    matrix4d m_globalPoseAtConstructionNormalized;


    /*! Vector of childs */
    std::vector< JointPrivate*> m_Children;

    /*! Nb of dofs for this joint. */
    unsigned int m_nbDofs;

    /*! Create the arrays (when the type is known). */
    void CreateLimitsArray();

    /*! \brief True if the Joint is initially in global frame, false it is a local reference frame*/
    bool getinGlobalFrame() const;

  private:

    /** Coefficient of the mass related to the subtree of this joint. */
    double m_STcoef;
    /** CoM of the subtree reaching this joint. */
    vector3d m_STmcom;
    /*! Local CoM oriented in the world reference frame . */
    vector3d m_wlc;

    /*!  Type of the transformation */
    int m_type;

    /*! Axis of the transformation,
      for the link with one DoF. */
    vector3d m_axis;

    /*! Quantity of the rotation . */
    double m_quantity;

    /*!
      \brief 4x4 matrix for pose
      This homogeneous matrix represents the position of this joint frame in the frame of the parent joint.
    */
    matrix4d m_poseInParentFrame;

    /*! Father joint */
    JointPrivate * m_FatherJoint;

    /*! Vector of joints from the root to this joint. */
    std::vector< CjrlJoint*> m_FromRootToThis;

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

    /*! Compute global data from initialization using Local information*/
    void computeLocalAndGlobalPoseFromGlobalFrame();

    /*! Compute local data from initialization using global information*/
    void computeLocalAndGlobalPoseFromLocalFrame();

    /*! Compute normalized rotation matrix (to create a rotation around x-axis)
      from axis. */
    void NormalizeRotationFromAxis(vector4d &Axis, matrix3d &NormalizedRotation);

  public:

    /** */
    double subTreeCoef();
    /** */
    void subTreeCoef(double inReplacement);
    /** \brief Compute the CoM of the joint subtree.
     */
    void computeSubTreeMCom();
    /**
     */
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
    JointPrivate(int ltype, vector3d&  laxis,
		 double lquantite, matrix4d & apose);

    JointPrivate(int ltype, vector3d& laxis,
		 double lquantite, vector3d &translationStatic);

    JointPrivate(int ltype, vector3d& laxis,
		 double lquantite);


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
    double pose(unsigned r) ;

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

    /*! Returns the axis of the rotation. */
    inline const vector3d& axis() const
    { return m_axis; };

    /*! Set the axis of the rotation */
    inline void axis(const vector3d &anaxis)
    { m_axis = anaxis; };

    /*! Quantity of the rotation */
    inline const double & quantity() const
    { return m_quantity; }

    /*! Set the rotation of the joint */
    inline void quantity(const double & aquantity)
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
    { MAL_S4x4_MATRIX_ACCESS_I_J(m_poseInParentFrame,0,3) = staticTranslation(0);
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
    const matrix4d & initialPosition() const;
    /**
       \brief Get the current transformation of the joint.

       The current transformation of the joint is the transformation
       moving the joint from the position in initial configuration to
       the current position.

       The current transformation is determined by the configuration \f${\bf q}\f$ of the robot.
    */
    const matrix4d &currentTransformation() const;

    /*! \name Methods related to inverse dynamics computation.
      @{ */
    /**
       \brief Update this joint and body transformation according to the given vector of DoF values,
       and the parent joint's transformation if this is not a free flyer joint.
       \return false if the required number of dof values is not met.
    */
    virtual bool updateTransformation(const vectorN& inDofVector);

    /**
       \brief Update the joint and body velocity according to the given vector of DoF values,
       and the parent joint's transformation if this is not a free flyer joint.
       \return false if the required number of dof values is not met.
    */
    virtual bool updateVelocity(const vectorN& inRobotConfigVector,
				const vectorN& inRobotSpeedVector);

    /**
       \brief Update the joint and body acceleration according to the given vector of DoF values,
       and the parent joint's transformation if this is not a free flyer joint.
       \return false if the required number of dof values is not met.
    */
    virtual bool updateAcceleration(const vectorN& inRobotConfigVector,
				    const vectorN& inRobotSpeedVector,
				    const vectorN& inRobotAccelerationVector)=0;

    /*! @} */

    /**
       \brief Update the world position of the CoM.
    */
    virtual void updateWorldCoMPosition();

    /**
       \brief Update the world position of the CoM.
    */
    virtual void updateAccelerationCoM();

    /**
       \brief Update the momentum according the current transformation
       and speed.
    */
    virtual void updateMomentum();

    /**
       \brief Update the torque and forces of the associated body
       according to the previously computed body.
    */
    virtual void updateTorqueAndForce();


    /**
       \brief Get the velocity \f$({\bf v}, {\bf \omega})\f$ of the joint.

       The velocity is determined by the configuration of the robot and its time derivative: \f$({\bf q},{\bf \dot{q}})\f$.

       \return the linear velocity \f${\bf v}\f$ of the origin of the joint frame
       and the angular velocity \f${\bf \omega}\f$ of the joint frame.
    */
    CjrlRigidVelocity jointVelocity() const;

    /**
       \brief Get the acceleration of the joint.

       The acceleratoin is determined by the configuration of the robot
       and its first and second time derivative: \f$({\bf q},{\bf \dot{q}}, {\bf \ddot{q}})\f$.
    */
    CjrlRigidAcceleration jointAcceleration() const;
	
	/*added function for possible avoidance of the error: pure virtual function call*/
	//void ClearALL();
    
    /**
       \brief Get the number of degrees of freedom of the joint.
	   The number of Dof is updated in the constructor of each derivative type of private joint accordingly.
    */

	virtual unsigned int numberDof() const 
	 { return m_nbDofs;}
   

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
      if (inDofRank<m_LowerLimits.size())
	return m_LowerLimits[inDofRank];
      return 0.0;
    };

    /**
       \brief Get the upper bound of a given degree of freedom of the joint.

       \param inDofRank Id of the dof in the joint
    */
    inline double upperBound(unsigned int inDofRank) const
    {
      if (inDofRank<m_UpperLimits.size())
	return m_UpperLimits[inDofRank];
      return 0.0;
    };

    /**
       \brief Set the lower bound of a given degree of freedom of the joint.

       \param inDofRank Id of the dof in the joint
       \param inLowerBound lower bound
    */
    inline void lowerBound(unsigned int inDofRank, double inLowerBound)
    {
      if (inDofRank<m_LowerLimits.size())
	m_LowerLimits[inDofRank] = inLowerBound;
    };

    /**
       \brief Set the upper bound of a given degree of freedom of the joint.

       \param inDofRank Id of the dof in the joint
       \param inUpperBound Upper bound.
    */
    inline void upperBound(unsigned int inDofRank, double inUpperBound)
    {
      if (inDofRank<m_UpperLimits.size())
	m_UpperLimits[inDofRank] = inUpperBound;
    };

    /**
       \brief Set the upper velocity bound of a given degree of freedom of the joint.

       \param inDofRank Id of the dof in the joint
       \param inUpperVelocityBound Upper bound.
    */
    inline void upperVelocityBound(unsigned int inDofRank, double inUpperVelocityBound)
    {
      if (inDofRank<m_UpperVelocityLimits.size())
	m_UpperVelocityLimits[inDofRank] = inUpperVelocityBound;
    };

    /**
       \brief Get the lower velocity bound of a given degree of freedom of the joint.

       \param inDofRank Id of the dof in the joint
    */
    inline double lowerVelocityBound(unsigned int inDofRank) const
    {
      if (inDofRank<m_LowerVelocityLimits.size())
	return m_LowerVelocityLimits[inDofRank];
      return 0.0;
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
      if (inDofRank<m_UpperVelocityLimits.size())
	return m_UpperVelocityLimits[inDofRank];
      return 0.0;
    };

    /**
       \brief Set the lower velocity bound of a given degree of freedom of the joint.

       \param inDofRank Id of the dof in the joint
       \param inLowerVelocityBound lower bound
    */
    inline void lowerVelocityBound(unsigned int inDofRank, double inLowerVelocityBound)
    {
      if (inDofRank<m_LowerVelocityLimits.size())
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
    void setLinkedDBody(DynamicBodyPrivate* aDBP);
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



      /*! \name Methods related to Spatial notations.
	@{ */
      /*! \brief Returns the transformation of the joint
	following a Plucker transformation according to table 1.5 of the HoR */
      virtual Spatial::PluckerTransform xjcalc(const vectorN & qi);
      /*! \brief Returns the position of the joint in the link reference frame
	following a Plucker transformation according to table 1.5 of the HoR */
      const Spatial::PluckerTransform & XL();

      /*! \brief Returns the position of the joint in the world reference frame
	following a Plucker transformation according to table 1.5 of the HoR */
      const Spatial::PluckerTransform & X0();

      /*! \brief Returns the free modes of the  joint.
	Currently this will return an empty matrix.
	This is a pure virtual function that will be implemented for each type of joint in each corresponding derivative code.
      */
      const virtual matrixNxP & pcalc(const vectorN & qi) = 0;

      /*! \brief Returns the derivative of the free modes of the  joint.
	Currently this will return an empty matrix.
	This is a pure virtual function that will be implemented for each type of joint in each corresponding derivative code
	But this function is not computing the derivative only initialyzing it.
      */
      const virtual matrixNxP & pdcalc(const vectorN & qi) = 0;

      /*! \brief Returns the spatial velocity. */
      const Spatial::Velocity & sv();

      /*! \brief Returns the spatial accelaration. */
      const Spatial::Acceleration & sa();

      friend ostream & operator<<(ostream & os, const JointPrivate &a);

	  // Functions added by L.S to be conform to Featherstone's code RNEA using spatial vectors

	  /*Rotation matrix for a free body (precisely here the FF) knowing its euler angles by L.S */
	  void eulerXYZ(MAL_VECTOR(,double) & qi_ang, matrix3d & localRot);

	  /*Rotation matrix for a revolute joint knowing the angle of rotation of its signle DOF by L.S */
	  void rotx(const vectorN & qi_ang, matrix3d & localRot);

	  /*the 3d skew matrix by L.S */
	  const MAL_S3x3_MATRIX(,double) & skew(MAL_S3_VECTOR(,double) & qi_pos);

	  /*External force vector including only gravity terms by L.S */
	  MAL_VECTOR(,double) & ComputeExtForce();

	  /*Compute the parent joint's spatial transformation matrix Xparent(i) to joint i, 
	  and the absolute joint's spatial transformation matrix X0 to joint i for all types of joints,
	  and the homogeneous transformation matrix of each joint*/
	  bool SupdateTransformation(const vectorN& inRobotConfigVector);

	  /*Update the spatial body velocity according to the given vector of DoF values*/
	  bool SupdateVelocity(const vectorN& inRobotConfigVector,
				  const vectorN& inRobotSpeedVector);

	  /*Update the world Com Position and the spatial body acceleration according to the given vector of DoF values and the spatial body force*/
	  bool SupdateAcceleration(const vectorN& inRobotConfigVector,
				      const vectorN& inRobotSpeedVector,
				      const vectorN& inRobotAccelerationVector);

	  /*Update the torque vector of the associated body and spatial parent body force according to the previously computed body*/
	  void SupdateTorqueAndForce();
	  
  private:

      /*! \brief Position of the joint in the link
	reference frame using Plucker coordinate.
	@{ */
      /*! \brief Initialize the position
	using other parameters */
      void initXL();

      /*! \brief Store the position of the joint in the body reference frame. */
      Spatial::PluckerTransform m_XL;

      /*! \brief Store the position of the joint in the father joint reference frame. */
      Spatial::PluckerTransform m_iXpi;

      /*! \brief Store the position of the joint in the world
	reference frame.  */
      Spatial::PluckerTransform m_X0;

      /*! \brief Store the velocity. */
      Spatial::Velocity m_sv;

      /*! \brief Store the spatial acceleration. */
      Spatial::Acceleration m_sa;

      /*! \brief Store the spatial force. */
      Spatial::Force m_sf;

      /*! \brief Store the spatial inertia of the related body. */
      Spatial::Inertia m_sI;

  protected:
      /*! \brief Store the constraints on motion
	also known as \f$[\phi \f$] or called the modes
      */
      matrixNxP m_phi;

      /*! \brief Store the derivative of the constraints on motion
	also known as \f$[\phi \f$]
      */
      matrixNxP m_dotphi;

  private:
      /*! \brief Store Zeta the momentum */
      vectorN m_Zeta;

	  /*gravity constant by L.S*/
	  //const double gravity_cst;
	  MAL_S3x3_MATRIX(,double) sk;

	  /* The spatial transformation matrix from the joint frame to the current body frame by L.S */
	  Spatial::PluckerTransform Xl_i;

	  /* The spatial transformation matrix from the mother body frame to the joint frame by L.S */
	  Spatial::PluckerTransform Xj_i;

	  /* The spatial transformation matrix from the world reference frame to the mother body frame by L.S*/
	  Spatial::PluckerTransform X0i_j;

	  /* The spatial external force due to gravity by L.S*/
	  Spatial::Force f_ext;

	  /* The spatial plucker transform (XF)^{-1}=X^{T} by L.S*/
	  Spatial::PluckerTransformTranspose XpiiT;

  };

  ostream & operator<<(ostream & os, const JointPrivate &a);
}
#endif /* JOINTPRIVATE_H */
