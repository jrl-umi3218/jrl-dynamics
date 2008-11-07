/* Fundamental object used to compute :
   
   - Center Of Mass,
   - Zero Momentum Point.
   OS: Updated the names of the contributors, the documentation
   and added a sample file for WalkPlugin
   OS (21/12/2006): removed any reference to non-homogeneous matrix
   library.


   Copyright (c) 2005-2006, 
   @author : 
   Adrien Escande, 
   Abderrahmane Kheddar,  
   Olivier Stasse, 
   Ramzi Sellouati
   
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
#ifndef _JRLCIRDYNAMICS_BODY_JRL_JAPAN_H_
#define _JRLCIRDYNAMICS_BODY_JRL_JAPAN_H_

#include <iostream>
#include <string>
//#include "linalg.h"
#include "MatrixAbstractLayer/MatrixAbstractLayer.h"
#include "robotDynamics/jrlBody.h"
#include "dynamicJRLJapan_API.h"
#include "dynamicsJRLJapan/Joint.h"

using namespace::std;

namespace dynamicsJRLJapan
{

  /** @ingroup forwardynamics
      This object is used to compute the Center Of Masse,
      and store some basic information of each body of the robot. 
  */    
  class DYN_JRL_JAPAN_EXPORT Body : public CjrlBody
  {
  protected:
    
    /*! \name Physical parameters of the body. */
    //@{
    /*! \brief Label */
    int label;
    /*! \brief Mass */
    double masse;
    /*! \brief Position of the Center of Mass. */
    vector3d posCoM;

    /*! \brief Inertia Matrix. */
    matrix3d inertie;
    //@}
  
    /*! \brief Number of objects. */
    int nombreObjets;
    /*! \brief Name of the body */
    std::string Name;
    
    /*! \brief Label of the mother of the body in a tree structure. */
    int labelMother;
  
    /*! \brief Says if the body has been explored. */
    int m_Explored;
    
    /*! \brief Joint to which the body is attached inside a tree
     structure. */
    Joint * m_AttachedJoint;

  public:

    /*! \name Constructors and destructor 
      @{
     */
    
    /*! \brief Basic constructor. */
    Body(void);
    
    /*! \brief Constructor while parsing.*/
    Body(double masse);

    /*! \brief Constructor while parsing. */
    Body(double masse, 
	 vector3d positionCoM);

    /*! \brief Constructor while parsing. */
    Body(double masse, 
	 vector3d positionCoM, 
	 matrix3d matriceInertie);
     
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
    double getMasse(void) const;
    
    /*! \brief Set the mass of the body. */
    void setMasse(double);
    
    /*! \brief Get the inertia matrix of the body. */
    inline const matrix3d & getInertie(void) const
      { return inertie;}
    
    /*! \brief Set the inertia matrix of the body. */
    void setInertie(double mi[9]);
    
    /*! \brief Get the position of the center of Mass. */
    vector3d getPositionCoM(void) const;
    
    /*! \brief Set the position of the center of Mass. */
    void setPositionCoM(double cm[3]);
    
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
    void joint(Joint * ajoint );
    
    /*! \brief  Returns if the object has been explored or not. */
    int getExplored() const;

    /*! \brief  Set the object as explored. */
    void setExplored(int anEx);
  
    /*! \brief  Returns the name of the object. */
    std::string getName() const;

    /*! \brief  Specify the name of the object. */
    void setName(char *);

    /** @} */
    
    /*! \name Methods to display informations */
    /** @{ */
    /*! \brief  Display on stdout all the information of the body. */
    void Display();
    
    /*! \brief  Display the number of geometric objects (stdout). */
    void afficherNombreObjets(void);

    /** @} */

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
      {return masse;};

    /**
    \brief Set mass.
     */
    inline void mass(double inMass)
      { masse=inMass;};


    /*! @} */
    
  };


};
#endif /* Body_H_*/
