/*
 * Copyright 2009, 2010, 
 *
 * Florent Lamiraux
 * Olivier Stasse
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

/*! System includes */
#include <iostream>
#include <sstream>
#include <fstream>
#include <string.h>

#include "Debug.h"

/*! Local library includes. */
#include "jrl/mal/matrixabstractlayer.hh"
#include "jrl/dynamics/dynamicbody.hh"
#include "DynMultiBodyPrivate.h"
#include "abstract-robot-dynamics/body.hh"

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
      double lmass = aBody->getMass();
      matrixNxP leftoperand;

      MAL_C_eq_A_by_B(leftoperand,MAL_RET_TRANSPOSE(pLinearJacobian),pLinearJacobian);
      m_InertiaMatrix = m_InertiaMatrix + lmass * leftoperand;
      ODEBUG("mass* leftoperand: " << lmass*leftoperand);
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
