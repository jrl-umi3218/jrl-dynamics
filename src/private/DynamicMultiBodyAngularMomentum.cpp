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
/*! Implements the angular Momentum methods of DynMultiBodyPrivate */

void DynMultiBodyPrivate::angularMomentumWrtCoM(vector3d & angularmomentum)
{
  angularMomentumWrtToPt(positionCoMPondere, angularmomentum);
}

void DynMultiBodyPrivate::angularMomentumWrtToPt(vector3d &, vector3d & angularmomentum)
{
  /** Intermediate variables. The mantra is :
      "To optimize those variables, in the Compiler we trust"
      (with the appropriate compilation options).
  */
  vector3d NE_lP,NE_lw_c, NE_tmp3, NE_tmp2, NE_tmp,NE_lL;
  matrix3d NE_Rtmp, NE_Rt, NE_Ro, NE_Rot;
  /* End of intermediate */

  DynamicBodyPrivate *aDB=0;
  int currentNode = labelTheRoot;
  currentNode = m_listOfBodies[labelTheRoot]->child;
  vector3d lL(0.0,0.0,0.0);

  do
    {

      aDB = m_listOfBodies[currentNode];

      NE_lP = m_listOfBodies[currentNode]->P;
      ODEBUG("P: " << NE_lP );
      NE_lw_c = m_listOfBodies[currentNode]->w_c - positionCoMPondere;

      // Computes angular momentum matrix L
      // Lk = xc x Pk + R * I * Rt * w
      MAL_S3x3_TRANSPOSE_A_in_At(m_listOfBodies[currentNode]->R,NE_Rt);

      MAL_S3_VECTOR_CROSS_PRODUCT(NE_tmp3,NE_lw_c,NE_lP);

      MAL_S3x3_C_eq_A_by_B(NE_tmp2,NE_Rt , m_listOfBodies[currentNode]->w);
      MAL_S3x3_C_eq_A_by_B(NE_tmp, m_listOfBodies[currentNode]->getInertie(),NE_tmp2);
      MAL_S3x3_C_eq_A_by_B(NE_tmp2, m_listOfBodies[currentNode]->R,NE_tmp);
      NE_lL = NE_tmp3 + NE_tmp2;
      ODEBUG("L: " << lL);

      lL += NE_lL;

      int step=0;
      int NextNode=0;
      do
        {

	  if (step==0)
            {
	      NextNode = m_listOfBodies[currentNode]->child;
	      step++;
            }
	  else if(step==1)
            {
	      NextNode = m_listOfBodies[currentNode]->sister;
	      step++;
            }
	  else if (step==2)
            {
	      NextNode = m_listOfBodies[currentNode]->getLabelMother();
	      if (NextNode>=0)
                {
		  /* Test if current node is leaf,
		     because in this case the force are not set properly. */
		  if (m_ComputeBackwardDynamics)
                    {
		      if ((m_listOfBodies[currentNode]->sister==-1) &&
			  (m_listOfBodies[currentNode]->child==-1))
			BackwardDynamics(*m_listOfBodies[currentNode]);

		      /* Compute backward dynamics */
		      BackwardDynamics(*m_listOfBodies[NextNode]);
                    }
		  currentNode = NextNode;
		  NextNode = m_listOfBodies[currentNode]->sister;
                }
	      else
		NextNode=labelTheRoot;
            }


        }
      while (NextNode==-1);
      currentNode = NextNode;

    }
  while(currentNode!=labelTheRoot);

  angularmomentum = lL;
}

/**
   \brief Get the angular momentum of the robot at the center of mass.
*/
const MAL_S3_VECTOR_TYPE(double)& DynMultiBodyPrivate::angularMomentumRobot()
{
  return m_L;

}

/**
   \brief Get the time-derivative of the angular momentum at the center of mass.
*/
const MAL_S3_VECTOR_TYPE(double)& DynMultiBodyPrivate::derivativeAngularMomentum()
{

  return m_dL;

}

MAL_S3_VECTOR_TYPE(double) DynMultiBodyPrivate::GetL(int JointID)
{
  MAL_S3_VECTOR(empty,double);
  if ((JointID>=0) &&
      ((unsigned int)JointID<m_listOfBodies.size()))
    return m_listOfBodies[ConvertIDInActuatedToBodyID[JointID]]->L;
  return empty;
}

void DynMultiBodyPrivate::getJacobianAngularMomentumWrtCoM(matrixNxP &outjacobian)
{
  if ((MAL_MATRIX_NB_ROWS(outjacobian) != 3) ||
      (MAL_MATRIX_NB_COLS(outjacobian) != numberDof()))
    MAL_MATRIX_RESIZE(outjacobian,3,numberDof());

  MAL_MATRIX_FILL(outjacobian,0);

  unsigned int rank;
  JointPrivate* aJoint;
  DynamicBodyPrivate* aBody;

  for(unsigned int i=0;i<m_ConfigurationToJoints.size();i++)
    {
      if (m_ConfigurationToJoints[i] == rootJoint())
	continue;

      aJoint = m_ConfigurationToJoints[i];
      aBody=  aJoint->linkedDBody();
      rank = aJoint->rankInConfiguration();

      matrixNxP pJacobian;
      vector3d av(0,0,0); // Dummy
      MAL_MATRIX_RESIZE(pJacobian,6, numberDof());
      getJacobian(*rootJoint(),*aJoint,av,pJacobian,true);

      ODEBUG("pJacobian:" <<pJacobian);
      matrixNxP pLinearJacobian;
      MAL_MATRIX_RESIZE(pLinearJacobian,3,MAL_MATRIX_NB_COLS(pJacobian));
      MAL_MATRIX_C_eq_EXTRACT_A(pLinearJacobian,pJacobian,double,0,0,3,
				MAL_MATRIX_NB_COLS(pJacobian));
      ODEBUG("pLinearJacobian:" <<endl <<pLinearJacobian);

      matrixNxP pAngularJacobian;
      MAL_MATRIX_RESIZE(pAngularJacobian,3,MAL_MATRIX_NB_COLS(pJacobian));
      MAL_MATRIX_C_eq_EXTRACT_A(pAngularJacobian,pJacobian,double,3,0,3,
				MAL_MATRIX_NB_COLS(pJacobian));

      ODEBUG("pAngularJacobian:" <<endl <<pAngularJacobian);

      // Used to compute the anti-symmetric matrix.
      matrixNxP xkmxg_cp;double lmass = aBody->getMass();
      MAL_MATRIX_RESIZE(xkmxg_cp,3,3);
      av =aBody->w_c - positionCoMPondere;
      xkmxg_cp(0,0) =          0.0; xkmxg_cp(0,1) = -lmass*av(2); xkmxg_cp(0,2) = lmass*av(1);
      xkmxg_cp(1,0) = lmass*av(2); xkmxg_cp(1,1) =           0.0; xkmxg_cp(1,2) =-lmass*av(0);
      xkmxg_cp(2,0) =-lmass*av(1); xkmxg_cp(2,1) =  lmass*av(0); xkmxg_cp(2,2) =         0.0;

      ODEBUG("xkmxg_cp: " <<xkmxg_cp);

      matrixNxP leftoperand;
      MAL_C_eq_A_by_B(leftoperand,xkmxg_cp,pLinearJacobian);
      outjacobian = outjacobian + leftoperand;

      matrixNxP rightoperand;
      matrix3d tmp2_3d;
      matrixNxP tmp2;
      MAL_MATRIX_RESIZE(tmp2,3,3);
      MAL_S3x3_C_eq_A_by_B(tmp2_3d,aBody->R,aBody->getInertie());
      for(unsigned int i=0;i<3;++i)
	for(unsigned int j=0;j<3;++j)
	  tmp2(i,j) = tmp2_3d(i,j);

      MAL_C_eq_A_by_B(rightoperand,tmp2,pAngularJacobian);

      outjacobian = outjacobian + rightoperand;
    }

}
