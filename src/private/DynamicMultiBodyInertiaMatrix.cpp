/* @doc Computation of the dynamic aspect for a robot.
   This class will load the description of a robot from a VRML file
   following the OpenHRP syntax. Using ForwardVelocity it is then
   possible specifying the angular velocity and the angular value 
   to get the absolute position, and absolute velocity of each 
   body separetly. Heavy rewriting from the original source
   of Adrien and Jean-Remy. 
 
   This implantation is an updated based on a mixture between 
   the code provided by Jean-Remy and Adrien.
 
   Copyright (c) 2005-2006, 
   @author Olivier Stasse, Ramzi Sellouati, Jean-Remy Chardonnet, Adrien Escande, Abderrahmane Kheddar
   Copyright (c) 2007-2009
   @author Olivier Stasse, Oussama Kannoun, Fumio Kanehiro.
   JRL-Japan, CNRS/AIST
 
   All rights reserved.

   Please refers to file License.txt for details on the license.

*/

/*! System includes */
#include <iostream>
#include <sstream>
#include <fstream>
#include <string.h>

#include "Debug.h"

/*! Local library includes. */
#include "MatrixAbstractLayer/MatrixAbstractLayer.h"
#include "dynamicsJRLJapan/DynamicBody.h"
#include "DynMultiBodyPrivate.h"
#include "robotDynamics/jrlBody.h"

#include "fileReader.h"

using namespace dynamicsJRLJapan;
/*! Implements Inertia Matrix methods of DynMultiBodyPrivate. */

void DynMultiBodyPrivate::computeInertiaMatrix()
{
  if ((MAL_MATRIX_NB_ROWS(m_InertiaMatrix) != numberDof()) || 
      (MAL_MATRIX_NB_COLS(m_InertiaMatrix) != numberDof()))
    MAL_MATRIX_RESIZE(m_InertiaMatrix,numberDof(),numberDof());

  MAL_MATRIX_FILL(m_InertiaMatrix,0);

  unsigned int rank;
  JointPrivate* aJoint;
  DynamicBodyPrivate* aBody;
    
  for(unsigned int i=1;i<m_listOfBodies.size();i++)
    {
      aBody=  m_listOfBodies[i];
      aJoint=(JointPrivate *)aBody->joint();
      
      rank = aJoint->rankInConfiguration();
      
      matrixNxP pJacobian;
      MAL_MATRIX_RESIZE(pJacobian, 6, numberDof());
      vector3d aCoM = aBody->localCenterOfMass(); 
      getJacobian(*rootJoint(),*aJoint,aCoM,pJacobian);
      
      matrixNxP pLinearJacobian;
      MAL_MATRIX_RESIZE(pLinearJacobian,3,MAL_MATRIX_NB_COLS(pJacobian));
      MAL_MATRIX_C_eq_EXTRACT_A(pLinearJacobian,pJacobian,double,0,0,3,
				MAL_MATRIX_NB_COLS(pJacobian));

      matrixNxP pAngularJacobian; 
      MAL_MATRIX_RESIZE(pAngularJacobian,3,MAL_MATRIX_NB_COLS(pJacobian));
      MAL_MATRIX_C_eq_EXTRACT_A(pAngularJacobian,pJacobian,double,3,0,3,
				MAL_MATRIX_NB_COLS(pJacobian));

      // Used to compute the symmetric matrix.
      double lmasse = aBody->getMasse();
      matrixNxP leftoperand;

      MAL_C_eq_A_by_B(leftoperand,MAL_RET_TRANSPOSE(pLinearJacobian),pLinearJacobian);
      m_InertiaMatrix = m_InertiaMatrix + lmasse * leftoperand;
      ODEBUG("masse* leftoperand: " << lmasse*leftoperand);
      matrixNxP rightoperand;
      matrix3d tmp2_3d,tmp2_3d2;
      matrixNxP tmp2,tmp3;
      MAL_MATRIX_RESIZE(tmp2,3,3);
      MAL_S3x3_C_eq_A_by_B(tmp2_3d,aBody->getInertie(),MAL_S3x3_RET_TRANSPOSE(aBody->R)); 
      MAL_S3x3_C_eq_A_by_B(tmp2_3d2,aBody->R,tmp2_3d); 
      
      for(unsigned int k=0;k<3;++k)
	for(unsigned int l=0;l<3;++l)
	  tmp2(k,l) = tmp2_3d2(k,l);
      
      MAL_C_eq_A_by_B(tmp3,tmp2,pAngularJacobian);
      MAL_C_eq_A_by_B(rightoperand,MAL_RET_TRANSPOSE(pAngularJacobian),tmp3);
	  
      ODEBUG("rightoperand: " << rightoperand);
      m_InertiaMatrix = m_InertiaMatrix + rightoperand;
    }
  
}

const matrixNxP & DynMultiBodyPrivate::inertiaMatrix() const
{
  return m_InertiaMatrix;
}

matrixNxP & DynMultiBodyPrivate::getInertiaMatrix() 
{
  return m_InertiaMatrix;
}
