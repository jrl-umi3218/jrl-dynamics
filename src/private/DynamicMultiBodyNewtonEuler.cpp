/*
 * Copyright 2009, 2010,
 *
 * Francois Keith
 * Florent Lamiraux
 * Layale Saab
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
#include <assert.h>
#include "Debug.h"

/*! Local library includes. */
#include "jrl/mal/matrixabstractlayer.hh"
#include "jrl/dynamics/dynamicbody.hh"
#include "DynMultiBodyPrivate.h"
#include "abstract-robot-dynamics/body.hh"

#include "fileReader.h"

using namespace dynamicsJRLJapan;

/*! Kept for backward compatibility. */
void DynMultiBodyPrivate::ForwardVelocity(MAL_S3_VECTOR(&PosForRoot,double),
					  MAL_S3x3_MATRIX(&OrientationForRoot,double),
					  MAL_S3_VECTOR(&v0ForRoot,double),
					  MAL_S3_VECTOR(&wForRoot,double),
					  MAL_S3_VECTOR(&dvForRoot,double),
					  MAL_S3_VECTOR(&dwForRoot,double))
{
  NewtonEulerAlgorithm(PosForRoot,OrientationForRoot,v0ForRoot,wForRoot,dvForRoot,dwForRoot);
}


/*! Implementation of the Newton-Euler algorithm */
void DynMultiBodyPrivate::NewtonEulerAlgorithm(MAL_S3_VECTOR(&PosForRoot,double),
					       MAL_S3x3_MATRIX(&OrientationForRoot,double),
					       MAL_S3_VECTOR(&v0ForRoot,double),
					       MAL_S3_VECTOR(&wForRoot,double),
					       MAL_S3_VECTOR(&dvForRoot,double),
					       MAL_S3_VECTOR(&dwForRoot,double))
{

  int currentNode = labelTheRoot;
  DynamicBodyPrivate *currentBody=0;

  currentBody = m_listOfBodies[currentNode];
  JointPrivate * currentJoint = currentBody->getJointPrivate();

  currentBody->p = PosForRoot;
  currentBody->v0 = v0ForRoot;
  currentBody->R = OrientationForRoot;
  MAL_S3x3_MATRIX_SET_IDENTITY(currentBody->R_static);
  currentBody->w = wForRoot;
  currentBody->dv = dvForRoot;
  currentBody->ldv = dvForRoot;
  currentBody->ldv_c = dvForRoot;
  currentBody->dw = dwForRoot;
  currentBody->ldw = dwForRoot;

  // Initialize momentum
  if (m_ComputeMomentum)
    {
      MAL_S3_VECTOR_FILL(m_P,0);
      MAL_S3_VECTOR_FILL(m_L,0);
      //currentJoint->updateMomentum();
    }

  // Initialize CoM value.
  positionCoMPondere[0] = 0;
  positionCoMPondere[1] = 0;
  positionCoMPondere[2] = 0;

  currentNode = m_listOfBodies[labelTheRoot]->child;

  do
    {

      currentBody = m_listOfBodies[currentNode];
      currentJoint = currentBody->getJointPrivate();

      // Position and orientation in reference frame
      currentJoint->updateTransformation(m_Configuration);

      if (m_ComputeVelocity)
	currentJoint->updateVelocity(m_Configuration,
				     m_Velocity);
      // Computes also the center of mass in the reference frame.
      if (m_ComputeCoM)
        {
	  currentJoint->updateWorldCoMPosition();
	  positionCoMPondere +=  currentBody->w_c * currentBody->getMass();
        }

      // Update the momentum
      if (m_ComputeMomentum)
        {
	  currentJoint->updateMomentum();
	  m_P += currentBody->P;
	  m_L+= currentBody->L;
        }

      // Update the acceleration of the body.
      if (m_ComputeAcceleration)
	currentJoint->updateAcceleration(m_Configuration,
					 m_Velocity,
					 m_Acceleration);

      // Update the acceleration of the joint's CoM
      if (m_ComputeAccCoM)
	  currentJoint->updateAccelerationCoM();

      // TO DO if necessary : cross velocity.
      int step=0;
      int NextNode=0;
      do
        {

	  if (step==0)
            {
	      NextNode = currentBody->child;
	      step++;
            }
	  else if(step==1)
            {
	      NextNode = currentBody->sister;
	      step++;
            }
	  else if (step==2)
            {
	      NextNode = currentBody->getLabelMother();
	      if (NextNode>=0)
                {
		  /* Test if current node is leaf,
		     because in this case the force are not set properly. */
		  if (m_ComputeBackwardDynamics)
                    {
		      if ((currentBody->sister==-1) &&
			  (currentBody->child==-1))
			BackwardDynamics(*currentBody);

		      /* Compute backward dynamics */
		      if (NextNode!=labelTheRoot)
			BackwardDynamics(*m_listOfBodies[NextNode]);
                    }

		  currentNode = NextNode;
		  currentBody = m_listOfBodies[currentNode];

		  NextNode = currentBody->sister;

                }
	      else
		NextNode=labelTheRoot;
            }


        }
      while (NextNode==-1);
      currentNode = NextNode;

    }
  while(currentNode!=labelTheRoot);

  if (m_ComputeSkewCoM)
    {
      SkewCoM(0,0) = 0;
      SkewCoM(0,1) =-positionCoMPondere[2];
      SkewCoM(0,2) = positionCoMPondere[1];
      SkewCoM(1,0) = positionCoMPondere[2];
      SkewCoM(1,1) = 0;
      SkewCoM(1,2) =-positionCoMPondere[0];
      SkewCoM(2,0) =-positionCoMPondere[1];
      SkewCoM(2,1) = positionCoMPondere[0];
      SkewCoM(2,2) = 0;
    }

  positionCoMPondere = positionCoMPondere/m_mass;

  // Zero Momentum Point Computation.
  if (m_ComputeZMP)
    {

      // Update the momentum derivative
      if (m_IterationNumber>1)
        {
	  m_dP = (m_P - m_Prev_P)/m_TimeStep;
	  m_dL = (m_L - m_Prev_L)/m_TimeStep;

	  // Update the ZMP value.
	  double px,py,pz=0.0;
	  CalculateZMP(px,py,
		       m_dP, m_dL,pz);

	  m_ZMP(0) = px;
	  m_ZMP(1) = py;
	  m_ZMP(2) = pz;

	  ODEBUG4(m_ZMP<< " | " << m_dP << " | " << m_dL << "|" << m_IterationNumber,"DebugDataZMP.dat");

        }
      else
        {
	  m_ZMP = positionCoMPondere;
	  m_ZMP(2) = 0.0;
        }

     // Update the store previous value.
      if (m_IterationNumber>=1)
        {
	  m_Prev_P = m_P;
	  m_Prev_L = m_L;
        }
    }


  m_IterationNumber++;
}

/**
   \name Forward kinematics and dynamics
*/


/**
   \brief Compute forward kinematics.

   Update the position, velocity and accelerations of each
   wrt \f${\bf {q}}\f$, \f${\bf \dot{q}}\f$, \f${\bf \ddot{q}}\f$.

*/
bool DynMultiBodyPrivate::computeForwardKinematics()
{
  MAL_S3_VECTOR(,double) lPositionForRoot;
  MAL_S3x3_MATRIX(,double) lOrientationForRoot;
  MAL_S3_VECTOR(,double) lLinearVelocityForRoot;
  MAL_S3_VECTOR(,double) lAngularVelocityForRoot;
  MAL_S3_VECTOR(,double) lLinearAccelerationForRoot;
  MAL_S3_VECTOR(,double) lAngularAccelerationForRoot;

  for(unsigned int i=0;i<3;i++)
    {
      lPositionForRoot(i)=(*m_RootOfTheJointsTree)(i,3);
    }

  for(unsigned int i=0;i<3;i++)
    for(unsigned int j=0;j<3;j++)
      lOrientationForRoot(i,j)=(*m_RootOfTheJointsTree)(i,j);

  if (rootJoint()->numberDof() == 6)
    {
      for(unsigned int i=0;i<3;i++)
	lLinearVelocityForRoot(i)  = m_Velocity(i);

      for(unsigned int i=3;i<6;i++)
	lAngularVelocityForRoot(i-3)  = m_Velocity(i);
    }
  else
    {
      MAL_S3_VECTOR_FILL(lLinearVelocityForRoot, 0);
      MAL_S3_VECTOR_FILL(lAngularVelocityForRoot, 0);
    }

  if (rootJoint()->numberDof() == 6)
    {
      for(unsigned int i=0;i<3;i++)
	lLinearAccelerationForRoot(i)  = m_Acceleration(i);

      for(unsigned int i=3;i<6;i++)
	lAngularAccelerationForRoot(i-3)  = m_Acceleration(i);
    }
  else
    {
      MAL_S3_VECTOR_FILL(lLinearVelocityForRoot, 0);
      MAL_S3_VECTOR_FILL(lAngularVelocityForRoot, 0);
    }
  ODEBUG(" Position for Root: " << lPositionForRoot);
  ForwardVelocity(lPositionForRoot,
		  lOrientationForRoot,
		  lLinearVelocityForRoot,
		  lAngularVelocityForRoot,
		  lLinearAccelerationForRoot,
		  lAngularAccelerationForRoot);

  return true;
}
