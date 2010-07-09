/* @doc Fundamental object used to compute :
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
   
   Please refers to file License.txt for details on the license.

*/

#include "Body.h"

using namespace dynamicsJRLJapan;

/**************************************************/
/* Implementation of constructors and destructors */
/**************************************************/

Body::Body(void)
{
  m_AttachedJoint =0;
  m_mass	= 0;
  attCoefMass = 0;
  posCoM[0] = 0;
  posCoM[1] = 0;
  posCoM[2] = 0;
  MAL_S3x3_MATRIX_SET_IDENTITY(inertie);
  label=-1;
  labelMother=-1;
  m_Explored =0 ;
  m_Initialized = false;
}

Body::Body(double lmass) 
{
  m_AttachedJoint =0;
  this->m_mass		= lmass;
  m_Explored =0 ;
  m_Initialized = false;
}

Body::Body(double lmass, 
	   MAL_S3_VECTOR(positionCoM,double)) 
{
  m_AttachedJoint =0;
  this->m_mass		= lmass;
  m_Explored =0 ;  
  m_Initialized = false;
  this->posCoM	= positionCoM;
}
Body::Body(double lmass, 
	   MAL_S3_VECTOR(positionCoM,double), 
	   MAL_S3x3_MATRIX(matriceInertie,double)) 
{
  m_AttachedJoint =0;
  this->posCoM	= positionCoM;
  this->m_mass		= lmass;
  m_Explored =0 ;  
  m_Initialized = false;
  this->inertie	= matriceInertie;
}


Body::~Body(void)
{
}


/***********************************************/
/* Implementation of setters and getters       */
/***********************************************/

int Body::getLabel() const
{
  return label;
}

void Body::setLabel(int i)
{
  label = i;
}

double Body::getMass(void) const
{
  return m_mass;
}

string Body::getName() const
{
  return Name;
}

void Body::setName(char *aname)
{
  Name = aname;
}

Body & Body::operator=( Body const & r)
{
  label = r.getLabel();
  m_mass = r.getMass();
  posCoM =r.localCenterOfMass();
  inertie = r.getInertie();
  Name= r.getName();
  labelMother=r.getLabelMother();
  m_Explored = r.getExplored();
  return *this;
}

void Body::setInertie(double mi[9])
{
  for(unsigned int i=0;i<3;i++)
    for(unsigned int j=0;j<3;j++)
      inertie(i,j) = mi[i*3+j];
}

void Body::setMass(double lmass)
{
  m_mass =lmass;
}


void Body::setLabelMother(int al)
{
  labelMother = al;
}

int Body::getLabelMother() const
{
  return labelMother;
}

int Body::getExplored() const
{
  return m_Explored;
}

void Body::setExplored(int anEx)
{
  m_Explored = anEx;

}

bool Body::getInitialized() const
{
  return m_Initialized;
}

void Body::setInitialized(bool anInitialized)
{
  m_Initialized = anInitialized;
}

/***********************************************/
/* Implementation of the methods for display   */
/***********************************************/

void Body::Display()
{
  cout << "Name  :" << Name << endl;
  cout << "Mass :" << m_mass << endl;
  cout << "Center of Mass    : " << posCoM[0] << " " 
       << posCoM[1] << " " <<posCoM[2]<<endl;
  cout << "Inertia Matrix : " << endl;
  for(int i=0;i<3;i++)
    {
      for(int j=0;j<3;j++)
	cout << inertie(i,j) << " ";
      cout << endl;
    }
  cout << "Mother: " << labelMother<< endl;
}


/***********************************************/
/* Implementation of the generic JRL interface */
/***********************************************/

const MAL_S3_VECTOR(,double) & Body::localCenterOfMass() const
{
  return posCoM;
}

void Body::localCenterOfMass(const MAL_S3_VECTOR(,double) &inlocalCenterOfMass)
{
  posCoM = inlocalCenterOfMass;
}

/*! Returns inertia matrix in the local reference frame */
const MAL_S3x3_MATRIX(,double) & Body::inertiaMatrix() const
{
  return inertie;
}


void Body::inertiaMatrix(const MAL_S3x3_MATRIX(,double) &inInertiaMatrix) 
{
  inertie = inInertiaMatrix;
}

const CjrlJoint* Body::joint() const
{
  return m_AttachedJoint;
}

JointPrivate* Body::getJointPrivate()
{
  return m_AttachedJoint;
}

void Body::joint(JointPrivate * ajoint)
{
  m_AttachedJoint = ajoint;
  if (m_AttachedJoint!=0)
    m_AttachedJoint->setLinkedBody(*this);
}



