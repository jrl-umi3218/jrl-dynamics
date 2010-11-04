/*
 * Copyright 2010, 
 *
 * Francois Keith
 * Olivier Stasse,
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
/* @doc \file A two link robot model.
   This file test the results of dynamicsJRLJapan
   torque and forces computation against the model
   provided by Spong's book: Robot Modeling and control. */

#ifndef _DYNAMIC_JRL_JAPAN_TWO_LINKS_MODEL_H_
#define _DYNAMIC_JRL_JAPAN_TWO_LINKS_MODEL_H_

#include <string>
#include <iostream>
#include "jrl/mal/matrixabstractlayer.hh"
#include <jrl/dynamics/humanoiddynamicrobot.hh>
#include <jrl/dynamics/dynamicsfactory.hh>

namespace dynamicsJRLJapan
{
  /*! \brief Parameters of the Planar elbow robot.
   */
  typedef struct 
  {
    /*! \brief Length of the links. */
    double l[2];
    /*! \brief Position of the center of mass. */
    double lc[2];
    /*! \brief Masses of the links. */
    double m[2];
    /*! \brief Inertia matrices. */
    matrix3d I[2];
  } TwoLinksModelParameters;

  /*! \class Planar Elbow robot.

   */
  class CTwoLinksModel: public virtual jrlDelegate::dynamicRobot
    {
    public:
      /** Constructor 
	  \param anObjectFactory: Classical factory object for robot construction.
	  \param l1: Length of first link.
	  \param l2: Length of second link. */
      CTwoLinksModel(CjrlRobotDynamicsObjectFactory *anObjectFactory,
		     TwoLinksModelParameters &aSetOfParameters);
      
      /*! Compute analytical BackwardDynamics. */
      void computeAnalyticalBackwardDynamics(vector3d Forces[2],
					     vector3d Torques[2]);
      
      /*! Test if the instance is identical between generic and 
	analytical computation. Return true is this is the case.*/
      bool TestInstance(vectorN &aCurrentConf,
			vectorN &aCurrentVelocity,
			vectorN &aCurrentAcceleration,
			std::string &testname);
      
    private:
      /* Parameters of the two links model. */
      TwoLinksModelParameters m_SetOfParameters;
      
      /*! \brief Forward recursion link 1. */
      void ForwardRecursionLink1(vector3d &ac1,
				 vector3d &g1,
				 vector3d &ae1);

      /*! \brief Forward recursion link 2. */
      void ForwardRecursionLink2(vector3d &ae1,
				 vector3d &ac2,
				 vector3d &g2);
      

      /*! \brief Backward recursion link 2. */
      void BackwardRecursionLink2(vector3d &ac2,
				  vector3d &g2,
				  vector3d &w2,
				  vector3d &ldw2,
				  vector3d &f2,
				  vector3d &t2);
		
      /*! \brief Backward recursion link 1. */
      void BackwardRecursionLink1(vector3d &ac1,
				  vector3d &g1,
				  vector3d &f2,
				  vector3d &t2,
				  vector3d &w1,
				  vector3d &ldw1,
				  vector3d &f1,
				  vector3d &t1);
      
      double m_g;

      /*! Orthonormal basis */
      vector3d m_i, m_j, m_k;

    };
};

#endif /* _DYNAMIC_JRL_JAPAN_TWO_LINKS_MODEL_H_ */
