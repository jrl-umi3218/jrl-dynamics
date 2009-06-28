/* Computation of the dynamic aspect for a robot.
   
   This class will load the description of a robot from a VRML file
   following the OpenHRP syntax. Using ForwardVelocity it is then
   possible specifying the angular velocity and the angular value 
   to get the absolute position, and absolute velocity of each 
   body separetly. Heavy rewriting from the original source
   of Adrien and Jean-Remy. 

   This implantation is an updated based on a mixture between 
   the code provided by Jean-Remy and Adrien.
  
   Copyright (c) 2005-2006, 
   @author Olivier Stasse, Ramzi Sellouati, Jean-Remy Chardonnet, Adrien Escande, Francois Keith, Abderrahmane Kheddar
   Copyright (c) 2007-2009
   @author Olivier Stasse, Oussama Kannoun, Fumio Kanehiro, Florent Lamiraux.
   
   JRL-Japan, CNRS/AIST

   All rights reserved.

   Please see License.txt for more informations on the license related to this software.
*/

#ifndef _DYNAMIC_MULTI_BODY_H_
#define _DYNAMIC_MULTI_BODY_H_
#include <vector>
#include "MatrixAbstractLayer/MatrixAbstractLayer.h"
#include "robotDynamics/jrlDynamicRobot.h"
#include "dynamics-config.h"
#include "Joint.h"
#include "MultiBody.h"
#include "DynamicBody.h"


using namespace::std;
namespace dynamicsJRLJapan
{
  typedef struct 
  { char LinkName[1028];
    unsigned int RankInConfiguration;
  } NameAndRank_t;

  /** @ingroup forwardynamics
      This class simulates the dynamic of a multibody robot.

      \note Two ways of constructing a kinematic chain are supported.
        \li through VRML parser VRMLReader::ParseVRMLFile
        \li through abstract robot dynamics interfaces. When using this solution, 
	the joints should be inserted in the kinematic tree with increasing depth. For instance, in chain J1 -> J2 -> J3, J2 should be inserted as J1 child before J3 is inserted as J2 child.
  */
  class DYN_JRL_JAPAN_EXPORT DynamicMultiBody : public virtual CjrlDynamicRobot,
    public MultiBody 
    
  {
  private:
     /* **/ 
      vector3d tempLV,tempP,tempDP;
      
    /**  Label of the root. */
    int labelTheRoot;
    
    /**  List of bodies with dynamical properties */
    std::vector<DynamicBody *> m_listOfBodies;
    
    /** Array to convert Joint Id from VRL file to Body array index. */
    std::vector<int> ConvertIDInActuatedToBodyID;
    
    /** Update body parameters from the bodyinfo list of 
      joints and the internal list of bodies. */
    void UpdateBodyParametersFromJoint(int cID, int lD, int LiaisonForFatherJoint);
    
    /** The skew matrix related to the CoM position. */
    matrix3d SkewCoM;

    /** Weighted CoM position. */
    vector3d positionCoMPondere;
    
    /** Splitted inertial matrices. */
    matrixNxP m_MHStarB;
    matrixNxP m_MHStarLeftFoot; 
    matrixNxP m_MHStarRightFoot;
    matrixNxP m_MHFree;
    
    /** Inversed Jacobian for the left and right foot. */
    matrixNxP m_ILeftJacobian;
    matrixNxP m_IRightJacobian;
    matrixNxP m_ERBFI_Left;
    matrixNxP m_ERBFI_Right;

    /*! \name Members related to the momentum 
      @{
    */
    /** \brief Linear momentum vector */
    vector3d m_P;
    
    /** \brief Derivative of the linear momentum */
    vector3d m_dP;

    /** \brief Angular momentum vector. */
    vector3d m_L;

    /** \brief Derivative of the angular momentum */
    vector3d m_dL;
    
    /** \brief Previous Linear momentum vector */
    vector3d m_Prev_P;

    /** \brief Previous Angular momentum vector. */
    vector3d m_Prev_L;
        
    
    /** @} */
    /** \brief Starting the computation. */
    bool m_FirstTime;

    /** \brief Store the root of the Joints tree. */
    Joint * m_RootOfTheJointsTree;

    /** \brief Velocity of the center of Mass */
    vector3d m_VelocityCenterOfMass;

    /** \brief Acceleration of the center of Mass */
    vector3d m_AccelerationCenterOfMass;

    /** \brief Vector to store the global current configuration 

    \f$ q \f$ 
      The current format is :
      \f[ q = \\
      \left( 
      \begin{matrix}
      {\bf c} \\
      {\bf o} \\
      q_0 \\
      ... \\
      q_{n-1}\\ 
      \end{matrix}
      \right)
      \f]
      
      where \f$ {\bf c} \f$ is the position of the free flying body,
      \f$ {\bf  o} \f$ its orientation using the \f$ xyz \f$ convention.
      \f$q_0, ..., q_{n-1}\f$ is the value for each of the \f$ n\f$ joint.
    */
    vectorN m_Configuration;

    /** \brief Vector to store the global current velocity 

    \f$ q \f$ 
      The current format is :
      \f[ \dot{q} = \\
      \left( 
      \begin{matrix}
      {\bf v} \\
      {\bf w} \\
      \dot{q}_0 \\
      ... \\
      \dot{q}_{n-1}\\ 
      \end{matrix}
      \right)
      \f]
      
      where \f$ {\bf c} \f$ is the position of the free flying body,
      \f$ {\bf  o} \f$ its orientation using the \f$ xyz \f$ convention.
      \f$q_0, ..., q_{n-1}\f$ is the value for each of the \f$ n \f$ joint.
    */
    vectorN m_Velocity;

    /** \brief Vector to store the global current velocity 

    \f$ q \f$ 
      The current format is :
      \f[ \dot{q} = \\
      \left( 
      \begin{matrix}
      {\bf V} \\
      {\bf W} \\
      \ddot{q}_0 \\
      ... \\
      \ddot{q}_{n-1}\\ 
      \end{matrix}
      \right)
      \f]
      
      where \f$ {\bf c} \f$ is the position of the free flying body,
      \f$ {\bf  o} \f$ its orientation using the \f$ xyz \f$ convention.
      \f$q_0, ..., q_{n-1}\f$ is the value for each of the \f$ n \f$ joint.
    */
    vectorN m_Acceleration;

    /**
    This is m_Velocity a sampling period ago (used by FiniteDifferenceStateUpdate)
    */
    vectorN m_pastVelocity;
    
    /**
    This is m_Configuration a sampling period ago (used by FiniteDifferenceStateUpdate)
     */
    vectorN m_pastConfiguration;
    

    /** Vector of forces. */
    matrixNxP m_Forces;

    /** Vector of torques. */
    matrixNxP m_Torques;

    /** Inertia Matrix. */
    matrixNxP m_InertiaMatrix;
	
    /** Time step used to compute momentum derivative. */
    double m_TimeStep;

    /** Store the current ZMP value . */
    vector3d m_ZMP;

    /** Vector of pointers towards the joints. */
    std::vector<CjrlJoint*> m_JointVector;

    /*! \name Internal computation of Joints */
    /*! Method to compute the number of joints
      and btw the size of the configuration.
    */
    void ComputeNumberOfJoints();

    /*! Member to store the number of Dofs. */
    unsigned int m_NbDofs;
    
    /** @} */

    /*! \name Methods related to the building of proxy values */
    /*! @{ */
    /*! Interface between the state vector and the joints. */
    std::vector<int> m_StateVectorToJoint; 

    /*! Interface between the state vector and the DOFs
      of each joints. */
    std::vector<int> m_StateVectorToDOFs; 

    /*! Interface between the VRML ID and the row of the
      configuration state vector */
    std::vector<int> m_ActuatedIDToConfiguration;

    /*! Number of VRML IDs, they are supposed to be 
     positive or null, and continuous. */
    int m_NbOfVRMLIDs;

    /*! The method to build the two previous vectors. */
    void BuildStateVectorToJointAndDOFs();

    /*! The method to allocate and reallocate the
      Jacobian of the joints. */
    void UpdateTheSizeOfJointsJacobian();

    /*! @} */

    /*! Iteration number */
    unsigned int m_IterationNumber;

    /*! Jacobian of the CoM. */
    matrixNxP m_JacobianOfTheCoM;
    
    /*! Computation matrix */
    matrixNxP m_attCalcJointJacobian;

    /*! \brief Link between joint and state vector position */
    std::vector<NameAndRank_t> m_LinksBetweenJointNamesAndRank;
    
    /*! \brief The ith element of this vector is the joint corrsponding 
      to the ith element of the configuration of the robot (dofs)*/
    std::vector<Joint*> m_ConfigurationToJoints;

    /*! \brief Number of links */
    int m_LinksBetweenJointNamesAndRankNb;
    
    /**
    \brief Recursive emthod to update the kinematic tree transformations 
    starting from the given joint. This method update p, R and w_c for every Body
     */
    void forwardTransformation(Joint* inJoint, const vectorN& inConfiguration);

    /**
    \brief a temporary vector used in forwardTransformation, to avoid dynamic allocation in a recursive method
    */
    vector3d m_vek;
    
    /*! \brief Boolean control variables. 
      @{
     */

    /*! Computing velocity for each joint. */
    bool m_ComputeVelocity;

    /*! Computing acceleration for each joint. */
    bool m_ComputeAcceleration;

    /*! Computing CoM according to the position of each joint. */
    bool m_ComputeCoM;
    
    /*! Computing SkewCoM matrix */
    bool m_ComputeSkewCoM;

    /*! Computing momentum using the velocity, i.e. 
      setting the m_ComputeMomentum to true will set m_ComputeVelocity to true. */
    bool m_ComputeMomentum;

    /*! Compute the acceleration of the CoM. */
    bool m_ComputeAccCoM;

    /*! Compute backward dynamics, according to the Newton-Euler algorithm. */
    bool m_ComputeBackwardDynamics;

    /*! Compute ZMP. */
    bool m_ComputeZMP;

    /*! String specifying the map between the joints and their rank. */
    std::string m_FileLinkJointRank;

    /* @} */

    /*! Actuated joints related fields 
      @{
     */

    /* ! Boolean to know if the synchronization between the actuated vector 
     and the joints have been realized. 
     Set to false each time a new vector of actuated joints 
     is set.
     Set to true each time the method BuildLinkBetweenActuatedVectorAndJoints()
     is called.
    */
    bool m_SynchronizationBetweenActuatedVectorAndJoints;

    /*! Vector of actuated Joints. */
    std::vector<CjrlJoint *> m_ActuatedJoints;

    /*! @} */
  protected:

    /*! Internal synchronization */
    int BuildLinkBetweenActuatedVectorAndJoints();

    
  public:
    
    /** \brief Default constructor. */
    DynamicMultiBody(void);
    
    /** \brief Destructor */
    virtual  ~DynamicMultiBody();
    
  
    //-----------------------------
    // Forward model computation
    //-----------------------------
    
    /** \brief Parse a vrml file which describes the robot. The format
	should be compatible with the one specified by OpenHRP. 
	\param path directory where following files are located
	\param nom filename of VRML file
	\param option filename of XML file which contains information on associations between joint names and ranks
    */
    virtual void parserVRML(string path, string nom, 
			    const char *option);
    virtual void parserVRML(string nom, 
			    const char *option);

    /**
       \brief Initialization of the kinematic chain
       \return true if success, false otherwise
    */
    virtual bool initialize();

    /** \brief Initialize the complete structure from a tree description
	based on Joints.  The link between joints and their rank in
	the state vector should be specify using setLinksBetweenJointNamesAndRank. */
    virtual void InitializeFromJointsTree();
	
    /** \brief Creates the tree structure from the undirected graph
	structure build upon the VRML file, or the Joints tree structure. 
	It is very important to have either correctly set the
	links between joint names and rank in the state vector,
	or to provide a file name which specify those links.

	\param option: Read from a file the relationship between Joint Name and rank
	in the state vector.
    */
    virtual void CreatesTreeStructure(const char * option);

    /**
       \brief Set the joint ordering in the configuration vector
       
       \param inJointVector Vector of the robot joints
       
       Specifies the order of the joints in the configuration vector. 
       The vector should contain all the joints of the current robot.
    */
    virtual void setJointOrderInConfig(std::vector<CjrlJoint*> inJointVector);

    /** \name Dynamic parameters computation related methods 
     */


    /** \brief Computation of the dynamics according to the Newton-Euler 
	algorithm. Can be controlled by the methods
	setComputeVelocity, setComputeAcceleration, setComputeCoM,
	setcomputeBackwardDynamics, according to the needs.
	It is assume that the value of the joints (position, velocity, acceleration) has been
	correctly set. */
    void NewtonEulerAlgorithm(vector3d &PosForRoot, 
			      matrix3d &OrientationForRoot, 
			      vector3d &v0ForRoot,
			      vector3d &wForRoot,
			      vector3d &dvForRoot,
			      vector3d &dwForRoot);


    /** \brief Call the previous function, kept for backward compatibility,
     this method is doom to be deleted soon. */
    void ForwardVelocity(vector3d &PosForRoot, 
			 matrix3d &OrientationForRoot, 
			 vector3d &v0ForRoot,
			 vector3d &wForRoot,
			 vector3d &dvForRoot,
			 vector3d &dwForRoot);
    
        
    /** \brief Compute the backward part of the dynamics
	to get the force and the torques of the bodies.
     */
    void BackwardDynamics(DynamicBody & CurrentBody);

    /** \brief Calculate ZMP. */
    void CalculateZMP(double &px, 
		      double &py,
		      vector3d dP, 
		      vector3d dL, 
		      double zmpz);

    
    /** \brief Compute the D operator (Kajita IROS 2003 p. 1647) */
    matrix3d D(vector3d &r);

    /** @}
     */ 
    
    /** \name Jacobian computation related methods 
	@{
     */

    /** \brief Finding a path between two bodies   
      (this is the version of "trouverCheminEntre" and has been set in english) 
    */
    std::vector<int> FindPathBetween(int body1, int body2);

    /** @} */

    /** \name Getter and setter for dynamic bodies  */

    /** @{ */

    /** Get back the joint values for the joint JointID in the VRML numbering system. */
    double Getq(int JointID) const;
    
    /** Specifies the joint values for the joint JointID in the VRML numbering system. */
    void Setq(int JointID, double q);

    /** Get the joint speed values for the joint JointID in the VRML numbering system. */
    double Getdq(int JointID) const;
    
    /** Specifies the joint speed values for the joint JointID in the VRML numbering system. */
    void Setdq(int JointID, double dq);
        
    /** Get the linear velocity for the joint JointID in the VRML numbering system*/
    vector3d Getv(int JointID);
    
    /** Get the linear velocity for the body JointID in the VRML numbering system. */
    vector3d GetvBody(int BodyID);
    
    /**  Set the orientation for the body JointID in the VRML numbering system. */
    void SetRBody(int BodyID, matrix3d R);
    
    /** Set the linear velocity for the body JointID in the VRML numbering system. */
    void Setv(int JointID, vector3d v0);
    
    /** Set the angular velocity. */
    void Setw(int JointID, vector3d w);
    
    /** Get the angular velocity for the body JointID in the VRML numbering system. */
    vector3d Getw(int JointID);
    
    /** Get the angular velocity for the body JointID in the VRML numbering system . */
    vector3d GetwBody(int BodyID);
    
    /** Get the position for the body JointID in the VRML numbering system */
    vector3d Getp(int JointID);
    
    /** Get the Angular Momentum for the body JointID in the VRML numbering system */
    vector3d GetL(int JointID);
    
    /** Get the Linear Momentum for the body JointID in the VRML numbering system */
    vector3d GetP(int JointID);

    /*! \brief Read the specificities for the link between joint names and rank. */
    void ReadSpecificities(string aFileName);
    
    /** Set the time step to compute the Momentum derivative */
    inline void SetTimeStep(double inTimeStep)
      {m_TimeStep = inTimeStep;};
    
    /** Get the time step used to compute the Momentum derivative */
    inline double GetTimeStep() const
      {return m_TimeStep ;};
    
    /** Set the position, to be used with the body JointID in the VRML numbering system. */
    void Setp(int JointID, vector3d apos);

    /** Gives the two momentums vector. */
    void GetPandL(vector3d &aP, 
		  vector3d &aL);

    /** Get the position of the center of Mass. */
    vector3d getPositionCoM(void);
        
    /** Returns the rank of the joint from the name. */
    int JointRankFromName(Joint *aJoint);    

    /** Returns the joint according to the rank. */
    Joint * JointFromRank(int aRank);

    /** Returns the name of the body JointID in the VRML numbering system. */
    string GetName(int JointID);

    /** Returns a CjrlJoint corresponding to body JointID in the VRML numbering system . */
    CjrlJoint* GetJointFromActuatedID(int JointID);

    /** Returns a vector to transfer from VRML ID to configuration ID . */
    void GetJointIDInConfigurationFromActuatedID(std::vector<int> & VectorFromVRMLIDToConfigurationID);

    /** Returns the ZMP value */
    inline const vector3d getZMP() const
      {return m_ZMP;};

    /** @} */

    /** \name Methods related to the construction of a tree, by specifying a root among the vertices of the undirected graph.
	@{ 
    */

    /** Specify the root of the tree and recompute it. */
    void SpecifyTheRootLabel(int ID);
    
    /** Relabel the current body \a corpsCourant according to the 
	link (specified by \a liaisonDeProvenance ) by which this body is explored.
     */
    void ReLabelling(int corpsCourant, int liaisonDeProvenance);
    
    /** @} */

    /** Print all the informations. */
    void PrintAll();
    
    /*! \name Methods related to the generic JRL-interface 
      @{
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
    CjrlJoint* rootJoint() const;
    
    /**
       \brief Get a vector containing all the joints.
    */
    std::vector<CjrlJoint*> jointVector();
    
    
    /**
       \brief Get the number of degrees of freedom of the robot.
    */
    unsigned int numberDof() const;
    
    /** 
	\name Configuration, velocity and acceleration
    */
    
    /**
       \brief Set the current configuration of the robot.  
       
       It is assumed that this vector includes the free joint
       corresponding to the root composed of the position and 
       the orientation in this case.
       The linear and angular velocity should be specified with
       the method currentVelocity().
       
       Regarding the orientation the convention is the \f$ x y z \f$ convention
       (pitch-roll-yaw), \f$ \theta \f$ is pitch, \f$ \psi \f$ is row,
       \f$ \phi \f$ is yaw.
       The corresponding orientation is then:


       \f$ {\bf D} \equiv \left[ 
       \begin{matrix}
       cos \; \phi & sin \; \phi & 0 \\
       -sin \; \phi & cos \; \phi & 0 \\
       0 & 0 & 1 \\
       \end{matrix}
       \right]
       \f$


       \f$ {\bf C} \equiv \left[ 
       \begin{matrix}
       cos \; \theta & 0  & -sin \; \theta \\
       0 & 1 & 0  \\
       sin \; \theta & 0 & cos \; \theta \\
       \end{matrix}
       \right]
       \f$


       \f$ {\bf B} \equiv \left[ 
       \begin{matrix}
       1 & 0 & 0 \\
       0 & cos \; \psi & sin \; \psi  \\
       0 & -sin \; \psi & cos \; \psi  \\
       \end{matrix}
       \right]
       \f$


       the final matrix is \f$ {\bf A} = {\bf B} {\bf C} {\bf D} \f$
       

       \param inConfig the configuration vector \f${\bf q}\f$.
       
       \return true if success, false if failure (the dimension of the
       input vector does not fit the number of degrees of freedom of the
       robot).
    */
    bool currentConfiguration(const vectorN& inConfig);
    
    /**
       \brief Get the current configuration of the robot.
       
       \return the configuration vector \f${\bf q}\f$.
    */
    const vectorN& currentConfiguration() const;
    
    /**
       \brief Set the current velocity of the robot.  
       
       \param inVelocity the velocity vector \f${\bf \dot{q}}\f$.
       
       \return true if success, false if failure (the dimension of the
       input vector does not fit the number of degrees of freedom of the
       robot).
    */
    bool currentVelocity(const vectorN& inVelocity);
    
    /**
       \brief Get the current velocity of the robot.
       
       \return the velocity vector \f${\bf \dot{q}}\f$.
    */
    const vectorN& currentVelocity() const;
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

    
    /** ! \brief Creates the path between inStartJoint and inEndJoint. */
    std::vector<CjrlJoint*> jointsBetween(const CjrlJoint& inStartJoint, 
					  const CjrlJoint& inEndJoint) const;

    /** ! \brief Get the jacobian from inStartJoint to inEndJoint in the frame
     inFrameLocalPosition in outJacobian. */
    bool getJacobian(const CjrlJoint& inStartJoint, 
		     const CjrlJoint& inEndJoint, 
		     const vector3d& inFrameLocalPosition, 
       matrixNxP& outjacobian, unsigned int offset = 0, bool inIncludeStartFreeFlyer = true);

    /** ! \brief Get the jacobian for linear velocity in the frame
     inFrameLocalPosition inside outJacobian. */
    bool getPositionJacobian(const CjrlJoint& inStartJoint, 
			     const CjrlJoint& inEndJoint, 
			     const vector3d& inFrameLocalPosition, 
        matrixNxP& outjacobian, unsigned int offset = 0, bool inIncludeStartFreeFlyer = true);

    /** ! \brief Get the jacobian for angular velocity in the frame
     inFrameLocalPosition inside outJacobian. */
    bool getOrientationJacobian(const CjrlJoint& inStartJoint, 
				const CjrlJoint& inEndJoint, 
    matrixNxP& outjacobian, unsigned int offset = 0, bool inIncludeStartFreeFlyer = true);

    /** ! \brief Get the jacobian for Center of Mass in the frame
     inFrameLocalPosition inside outJacobian. */
    bool getJacobianCenterOfMass(const CjrlJoint& inStartJoint, 
                                 matrixNxP& outjacobian, unsigned int offset = 0, bool inIncludeStartFreeFlyer = true);

    /** ! \brief Get the jacobian of the linear momentum with respect to the
	center of Mass. */
    void getJacobianLinearMomentumWrtCoM(matrixNxP& outjacobian);

    /** ! \brief Get the jacobian of the linear momentum with respect to the
	center of Mass. */
    void getJacobianAngularMomentumWrtCoM(matrixNxP& outjacobian);

    /** ! \brief Get the angular momentum with respect to the
	center of Mass. 
	\param[out] angularmomentum: the 3d vector in which 
	the angular momentum with respect to the CoM is provided.
    */
    void angularMomentumWrtCoM(vector3d & angularmomentum);

    /** ! \brief Get the angular momentum with respect to the
	center of Mass. 
	\param[in] point: The point with respect to which the angular momentum is computed.
	\param[out] angularmomentum: The angularmomentum computed with respect to apoint.
    */
    void angularMomentumWrtToPt(vector3d & apoint, vector3d &angularmomentum);
    
    /** 
	\name Forward kinematics and dynamics
    */

    /**
        \brief Get the upper bound for ith dof.
    */
    double upperBoundDof(unsigned int inRankInConfiguration);
    /**
        \brief Get the lower bound for ith dof.
    */
    double lowerBoundDof(unsigned int inRankInConfiguration);
    
    /**
        \brief Get the upper bound for ith dof.
    */
    double upperBoundDof(unsigned int inRankInConfiguration, 
			 const vectorN& inConfig);
    /**
        \brief Get the lower bound for ith dof.
    */
    double lowerBoundDof(unsigned int inRankInConfiguration,
			 const vectorN& inConfig);
    
    /**
       \brief Compute forward kinematics.
       
       Update the position, velocity and accelerations of each
       joint wrt \f${\bf {q}}\f$, \f${\bf \dot{q}}\f$, \f${\bf \ddot{q}}\f$.
       
    */
    virtual bool computeForwardKinematics() ;
    
    /**
       \brief Compute the dynamics of the center of mass.
       
       Compute the linear and  angular momentum and their time derivatives, at the center of mass.
    */
    bool computeCenterOfMassDynamics() ;
    
    /**
       \brief Get the position of the center of mass.
    */
    const vector3d& positionCenterOfMass() const;
    
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
    
    /*! \brief Compute the inertia matrix. 
     */
    void computeInertiaMatrix();

    /*! \brief returns the inertia matrix. */
    const matrixNxP& inertiaMatrix() const;

    /*! \brief returns the inertia matrix. */
    matrixNxP& getInertiaMatrix();
    
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
     const matrixNxP & jacobianCenterOfMass() const ;

    /**
       \brief Get the Jacobian matrix of the center of mass wrt \f${\bf q}\f$.
    */
     matrixNxP & getJacobianOfTheCoM() ;

    /**
       \brief Get the total mass
    */
     double mass() const
     {
          return masse;
     }
    /**
       @}
    */
     
     
    /*! @}*/

     /*! \name Method related to the iteration number. 
       @{
      */
     /*! Reset the iterationNumber. */
     bool ResetIterationNumber()
     { m_IterationNumber = 0; return true;} ;

     /*! GetIndex */
     const unsigned int & GetIterationNumber() const
       { return m_IterationNumber;} 

          /*! \brief Setting the velocity. */
     void setComputeVelocity(const bool & abool)
     { m_ComputeVelocity = abool; }
     
     /*! \brief Getting the computation status for the velocity. */
     bool getComputeVelocity()
     { return m_ComputeVelocity; }
     
     /*! \brief Setting the acceleration. */
     void setComputeAcceleration(const bool & abool)
     { if (abool) m_ComputeVelocity=true;
       m_ComputeAcceleration = abool; }
     
     /*! \brief Getting the computation status for the acceleration. */
     bool getComputeAcceleration()
     { return m_ComputeAcceleration; }

     /*! \brief Compute the CoM. */
     void setComputeCoM(const bool & abool)
     { m_ComputeCoM = abool; }
     
     /*! \brief Compute the SkewCoM matrix. */
     void setComputeSkewCoM(const bool & abool)
     { m_ComputeSkewCoM = abool; }
     
     /*! \brief Getting the computation status for the CoM. */
     bool getComputeCoM()
     { return m_ComputeCoM; }

     /*! \brief Compute the Momentum. */
     void setComputeMomentum(const bool & abool)
     { 
       if (abool)
	 {
	   m_ComputeVelocity = true;
	 }
       m_ComputeMomentum = abool; 
     }
     
     /*! \brief Getting the computation status for the Momentum. */
     bool getComputeMomentum()
     { return m_ComputeMomentum; }

     /*! \brief Compute the CoM acceleration. */
     void setComputeAccelerationCoM(const bool & abool)
     { m_ComputeAccCoM = abool; }
     
     /*! \brief Getting the computation status for the CoM. */
     bool getComputeAccelerationCoM()
     { return m_ComputeAccCoM; }
     
      /*! \brief Compute the backward dynamics. */
     void setComputeBackwardDynamics(const bool & abool)
     { if (abool)
	 setComputeAcceleration(true);
       m_ComputeBackwardDynamics = abool; }
     
     /*! \brief Getting the computation status for the CoM. */
     bool getComputeBackwardDynamics()
     { return m_ComputeBackwardDynamics; }

      /*! \brief Compute the ZMP. */
     void setComputeZMP(const bool & abool)
     {
       if (abool)
	 {
	   m_ComputeVelocity = true;
	   m_ComputeCoM = true;
	   m_ComputeMomentum = true;
	 }
       m_ComputeZMP = abool; 
     }
     
     /*! \brief Getting the computation status for the ZMP. */
     bool getComputeZMP()
     { return m_ComputeZMP; }

     /* \name Methods related to the links between the joints and their
       rank in the state vector. 
       @{
     */
     /** \brief Set the relation between the joints and their rank in the state vector. */
     int setLinksBetweenJointNamesAndRank(std::vector<NameAndRank_t> &aLinks);

     /** \brief Get the relation between the joints and their rank in the state vector. */
     int getLinksBetweenJointNamesAndRank(std::vector<NameAndRank_t> &aLinks);

     /* @} */
   
     /*! \name Implementation of properties. 
       @{
      */
     /*! \brief Returns true if the property is supported.
       \param inProperty The valid value are:
       \li ComputeVelocity : Compute velocity for the bodies.
       \li ComputeCoM: Compute global CoM.
       \li ComputeAccelerationCoM: Compute the CoM acceleration.
       \li ComputeMomentum: Compute the global momentum and the momentum for each body.
       \li ComputeZMP: Compute ZMP.
       \li ComputeBackwardDynamics: Compute the force and the torque of each joint.
       \retval 
      */
     bool isSupported(const std::string &inProperty );
       
     /*! \brief Get property corresponding to command name.

     \param inProperty name of the property (cf isSupported) .
     \retval outValue value of the property.

     \note The returned string needs to be cast into boolean .*/ 
     bool getProperty(const std::string &inProperty,std::string &outValue);

     /**
	\brief Set property corresponding to command name.
	
	\param inProperty name of the property.
	\param inValue value of the property.
	
	\note The value string should contain either true or false.
       */
     bool setProperty(std::string &inProperty, const std::string& inValue); 

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

  };
};
#endif
