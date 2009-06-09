/* @doc Inverse Kinematics for legs and arms of a canonical
   humanoid robot. The arm are supposed to have 2 links.
   The legs are supposed to have 3 links.
   Please look at the documentation for more information.


   Copyright (c) 2005-2009, 
   @author Olivier Stasse, Ramzi Sellouati, Francois Keith, 
   
   JRL-Japan, CNRS/AIST

   All rights reserved.

   please see license.txt for more information on license.
*/

#ifndef _INVERSE_KINEMATICS_H_
#define _INVERSE_KINEMATICS_H_

#include <MatrixAbstractLayer/MatrixAbstractLayer.h>
#include <iostream>

#include "dynamics-config.h"
#include <robotDynamics/jrlHumanoidDynamicRobot.h>

namespace dynamicsJRLJapan
{
  /** Inverse Kinematics for generic humanoids.
      Please modify the code if needed. 
  */  
  class DYN_JRL_JAPAN_EXPORT InverseKinematics 
  {
  public:
    /*! Constructor. */
    InverseKinematics(CjrlHumanoidDynamicRobot *aHS);
      
    /*! Destructor */ 
    ~InverseKinematics();
      
      
    /*! Compute InverseKinematics for legs. */
    int ComputeInverseKinematicsForLegs(MAL_S3x3_MATRIX( ,double)& Body_R,
					MAL_S3_VECTOR(,double) &Body_P,
					MAL_S3_VECTOR(,double) &Dt,
					MAL_S3x3_MATRIX(,double) &Foot_R,
					MAL_S3_VECTOR(,double) &Foot_P,
					MAL_VECTOR( ,double) &q);
      
    /*! Compute InverseKinematics for arms. */
    int ComputeInverseKinematicsForArms(double X,
					double Z,
					double &Alpha,
					double &Beta);
      
    /*! Compute Arm swing maximum amplitude. */
    double ComputeXmax(double & lZ);
      
  protected:
      
    double m_KneeAngleBoundCos,m_KneeAngleBound;
    double m_KneeAngleBoundCos1,m_KneeAngleBound1;
    double m_KneeAngleBoundCos2;
      
    double m_FemurLength,m_TibiaLength;
      
    CjrlHumanoidDynamicRobot *m_HS;
  };
};
#endif /*_INVERSE_KINEMATICS_H_ */

