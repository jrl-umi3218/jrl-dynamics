/* Fundamental object used to compute :
   
   - Center Of Mass,
   - Zero Momentum Point.
   OS: Updated the names of the contributors, the documentation
   and added a sample file for WalkPlugin
   OS (21/12/2006): removed any reference to non-homogeneous matrix
   library.


   Copyright (c) 2005-2009, 
   @author : 
   Adrien Escande, 
   Francois Keith,
   Abderrahmane Kheddar,  
   Olivier Stasse, 
   Ramzi Sellouati
   
   JRL-Japan, CNRS/AIST

   Please refers to file License.txt for details on the license.
*/
#ifndef _JRLCIRDYNAMICS_BODY_JRL_JAPAN_H_
#define _JRLCIRDYNAMICS_BODY_JRL_JAPAN_H_

#include <iostream>
#include <string>
//#include "linalg.h"
#include "MatrixAbstractLayer/MatrixAbstractLayer.h"
#include "robotDynamics/jrlBody.h"
#include "JointPrivate.h"

using namespace::std;

namespace dynamicsJRLJapan
{

  /** @ingroup forwardynamics
      This object is used to compute the Center Of Mass,
      and store some basic information of each body of the robot. 
  */    
  class Body : public CjrlBody
  {
  protected:
    
    /*! \name Physical parameters of the body. */
    //@{
    /*! \brief Label */
    int label;
    /*! \brief Mass */
    double m_mass;
    /*! \brief Position of the Center of Mass. */
    vector3d posCoM;

    /*! \brief Inertia Matrix. */
    matrix3d inertie;
    //@}
  
    /*! \brief Name of the body */
    std::string Name;
    
    /*! \brief Label of the mother of the body in a tree structure. */
    int labelMother;
  
    /*! \brief Says if the body has been explored. */
    int m_Explored;

    /*! \brief Says if the body has been initialized. */
    bool m_Initialized;
    
    /*! \brief JointPrivate to which the body is attached inside a tree
     structure. */
    JointPrivate * m_AttachedJoint;
    
    double attCoefMass;

  public:

    /*! \name Constructors and destructor 
      @{
     */
    
    /*! \brief Basic constructor. */
    Body(void);
    
    /*! \brief Constructor while parsing.*/
    Body(double mass);

    /*! \brief Constructor while parsing. */
    Body(double mass, 
	 vector3d positionCoM);

    /*! \brief Constructor while parsing. */
    Body(double mass, 
	 vector3d positionCoM, 
	 matrix3d inertiaMatrix);
     
    /*! \brief Destructor. */
    virtual ~Body(void);
    
    /* @} */

    /*! \name Getter and setter for basic parameters */
    /* @{ */

    /* Assignment operator. */
    Body & operator=( Body const & r);
    
    /*! \brief Get the label of the body. */
    int getLabel(void) const;
    
    /*! \brief Set the label of the body. */
    void setLabel(int i);
    
    /*! \brief Get the mass of the body. */
    double getMass(void) const;
    
    /*! \brief Set the mass of the body. */
    void setMass(double);
    
    /*! \brief Get the inertia matrix of the body. */
    inline const matrix3d & getInertie(void) const
      { return inertie;}
    
    /*! \brief Set the inertia matrix of the body. */
    void setInertie(double mi[9]);
    
    /*! \brief Returns the label of the mother. */
    int getLabelMother() const;
    
    /*! \brief  Set the label of the mother. */
    void setLabelMother(int);
    
    /*! \brief  Get the number of geometric objects. */
    int getNbObjets() const;
    
    /*! \brief  Set the number of geometric objects. */
    void setNbObjets(int n);

    /** 
	\brief Set pointer to the joint the body is attached to.
	
	This joint is defined once a tree for a MultiBody object has been specified.
    */
    void joint(JointPrivate * ajoint );
    
    /*! \brief  Returns if the object has been explored or not. */
    int getExplored() const;

    /*! \brief  Set the object as explored. */
    void setExplored(int anEx);

    /*! \brief  Returns if the object has been initialized or not. 
      This is used to handle the case of virtual body,
      to handle complex joints. For instance, they appear in the case of
      compound of several straigh simple joints.
     */
    bool getInitialized() const;

    /*! \brief  Set the object as explored. */
    void setInitialized(bool anEx);
  
    /*! \brief  Returns the name of the object. */
    std::string getName() const;

    /*! \brief  Specify the name of the object. */
    void setName(char *);

    /** @} */
    
    /*! \name Methods to display informations */
    /** @{ */
    /*! \brief  Display on stdout all the information of the body. */
    void Display();
    
    /** @} */

    
    /** Get the mass of body divided by mass of robot*/
    double massCoef() const {return attCoefMass;}
    /** Set mass of body divided by mass of robot*/
    void massCoef(double inCoef){attCoefMass = inCoef;}
    
    
    /*! \name Interface from jrlBody 
      @{
    */
    /**
       \brief Get position of center of mass in joint local reference frame.
    */
    const vector3d& localCenterOfMass() const ;
    /**
       \brief Set position of center of mass in joint reference frame.
    */
    void localCenterOfMass(const vector3d& inlocalCenterOfMass);
    
    /**
       \brief Get Intertia matrix expressed in joint local reference frame.
    */
    const matrix3d& inertiaMatrix() const;
    
    /**
       \brief Set inertia matrix.
    */
    void inertiaMatrix(const matrix3d& inInertiaMatrix) ;
    
    /**
       \brief Get const pointer to the joint the body is attached to.
    */
    const CjrlJoint* joint() const;

        
    /**
    \brief Get mass.
     */
    inline double mass() const
      {return m_mass;};

    /**
    \brief Set mass.
     */
    inline void mass(double inMass)
    { m_mass=inMass;};


    /*! @} */
    
  };


};
#endif /* Body_H_*/
