/* @doc \file A two link robot model.
   This file test the results of dynamicsJRLJapan
   torque and forces computation against the model
   provided by Spong's book: Robot Modeling and control.

   Copyright (c) 2010
   @author Olivier Stasse
   JRL-Japan, CNRS/AIST
 
   All rights reserved.

   Please refers to file License.txt for details on the license.

*/

#ifndef _DYNAMIC_JRL_JAPAN_TWO_LINKS_MODEL_H_
#define _DYNAMIC_JRL_JAPAN_TWO_LINKS_MODEL_H_

#include <string>
#include <iostream>
#include <MatrixAbstractLayer/MatrixAbstractLayer.h>
#include <dynamicsJRLJapan/humanoidDynamicRobot.h>
#include <dynamicsJRLJapan/dynamicsJRLJapanFactory.h>

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
