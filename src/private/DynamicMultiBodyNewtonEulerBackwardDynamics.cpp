/* @doc Computation of the torques and forces.
   Copyright (c) 2009-2010
   @author Olivier Stasse,
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

void DynMultiBodyPrivate::BackwardDynamics(DynamicBodyPrivate & CurrentBody )
{
  MAL_S3x3_MATRIX(,double) aRt;

  MAL_S3x3_MATRIX(,double) currentBodyRt;
  currentBodyRt = MAL_S3x3_RET_TRANSPOSE(CurrentBody.R);
  
  /*//MAL_S3x3_MATRIX(,double) currentBodyinitialRt;
  matrix4d initialTransform  = CurrentBody.joint()->initialPosition();
  for(unsigned int li=0;li<3;li++)
    for(unsigned int lj=0;lj<3;lj++)
      currentBodyRt = MAL_S4x4_MATRIX_ACCESS_I_J(initialTransform,lj,li);

  // currentBodyRt = MAL_S3x3_RET_A_by_B(currentBodyRt,currentBodyinitialRt);
  */
  MAL_S3_VECTOR(,double) lg;
  lg(0) = 0.0;
  lg(1) = 0.0;
  lg(2) = -9.81;

  /* lg should be expressed in frame i */
  lg = MAL_S3x3_RET_A_by_B(currentBodyRt,lg);
  ODEBUG(" lg :" << lg);
  /* Compute the torque
   * with eq. (7.147) Spong RMC p. 277
   *
   *
   */
  MAL_S3_VECTOR(,double) firstterm,
    sndterm, thirdterm, fifthterm,tmp;
  // Do not fourth term because it is the angular acceleration.

  /* Force - Constant part: 2nd and 3rd term of eq.(7.146) 
     m_i a_{c,i} - m_i g_i
   */
  ODEBUG(" Body name: " << CurrentBody.getName() << " : " << lg << " mass: " << CurrentBody.mass());
  tmp = CurrentBody.ldv_c - lg;
  CurrentBody.m_Force =  tmp * CurrentBody.mass();
  /* Get the local center of mass */
  vector3d lc = CurrentBody.localCenterOfMass();

  /* Torque - 5th term : w_i x (I_i w_i)*/  
  MAL_S3x3_MATRIX(,double) lI = CurrentBody.getInertie();
  tmp = MAL_S3x3_RET_A_by_B(lI,CurrentBody.lw);
  //  tmp = MAL_S3x3_RET_A_by_B(lI,CurrentBody.w);


  MAL_S3_VECTOR_CROSS_PRODUCT(fifthterm,CurrentBody.lw,tmp);

  /* Torque - 4th term and 5th term 
  Torque_i = I_i * alpha_i +  (R_i w_i )x (I_i R_i w_i) */
  MAL_S3x3_C_eq_A_by_B(tmp,lI,CurrentBody.ldw);
  CurrentBody.m_Torque =  tmp + fifthterm ;
  //std::cout << "alpha_i: " << CurrentBody.ldw << std::endl;
  /* Compute with the force
   * eq. (7.146) Spong RMC p. 277
   * fi = R^i_{i+1} * f_{i+1} + m_i * a_{c,i} - m_i * g_i
   * g_i is the gravity express in the i reference frame.
   */

  matrix4d curtri;

  if (CurrentBody.joint()!=0)
    curtri = CurrentBody.joint()->currentTransformation();
  else
    MAL_S4x4_MATRIX_SET_IDENTITY(curtri);


  matrix4d invcurtri;
  MAL_S4x4_INVERSE(curtri,invcurtri,double);      

  int IndexChild = CurrentBody.child;

  while(IndexChild!=-1)
    {
      DynamicBodyPrivate *Child = m_listOfBodies[IndexChild];
      //cout << "Child Bodies : " << Child->getName() << endl;
      aRt = MAL_S3x3_RET_A_by_B(Child->R_static,Child->Riip1);

      // /* Force computation. */
      // R_i_{i+1} f_{i+1}
      tmp= MAL_S3x3_RET_A_by_B(aRt, Child->m_Force);
      CurrentBody.m_Force += tmp;

      /* Torque computation. */
      /* 1st term : R^i_{i+1} t_{i+1} */
      firstterm = MAL_S3x3_RET_A_by_B(aRt, Child->m_Torque);

      /* 3rd term : (R_i_{i+1} f_{i+1}) x rip1,ci */
      matrix4d curtrip1= Child->joint()->currentTransformation();
      matrix4d ip1Mi;
      MAL_S4x4_C_eq_A_by_B(ip1Mi, invcurtri, curtrip1);

      /* rip1,ci = (riip1)^(-1)rici 
	 Note: rip1,c1 is expressed in the frame of link i.
       */
      vector3d res3d; 
      MAL_S3_VECTOR_ACCESS(res3d,0) = 
	MAL_S3_VECTOR_ACCESS(lc,0)-
	MAL_S4x4_MATRIX_ACCESS_I_J(ip1Mi,0,3); 
      MAL_S3_VECTOR_ACCESS(res3d,1) = 
	MAL_S3_VECTOR_ACCESS(lc,1) - 
	MAL_S4x4_MATRIX_ACCESS_I_J(ip1Mi,1,3) ;
      MAL_S3_VECTOR_ACCESS(res3d,2) = 
	MAL_S3_VECTOR_ACCESS(lc,2)-
	MAL_S4x4_MATRIX_ACCESS_I_J(ip1Mi,2,3);
      MAL_S3_VECTOR_CROSS_PRODUCT(thirdterm,tmp, res3d);

      CurrentBody.m_Torque += firstterm + thirdterm;

      /* Body selection. */
      IndexChild = m_listOfBodies[IndexChild]->sister;
      if (IndexChild!=-1)
	Child=m_listOfBodies[IndexChild];
    }

  /* 2nd term : -f_i x r_{i,ci} */
  MAL_S3_VECTOR_CROSS_PRODUCT(sndterm,CurrentBody.m_Force, lc);
  CurrentBody.m_Torque = CurrentBody.m_Torque - sndterm;

  
  // Update the vector related to the computed quantities.
  for(unsigned int i=0;i<m_StateVectorToJoint.size();i++)
    {
      unsigned int StateRankComputed=false;

      JointPrivate * aJoint = (JointPrivate *)m_JointVector[m_StateVectorToJoint[i]];
      if (aJoint!=0)
        {
	  DynamicBodyPrivate *aDB = (DynamicBodyPrivate *) aJoint->linkedBody();
	  if (aDB!=0)
            {
	      StateRankComputed = true;
	      for(unsigned int k=0;k<3;k++)
                {
		  m_Forces(i,k)=aDB->m_Force[k];
		  m_Torques(i,k) = aDB->m_Torque[k];
                }

            }
        }

      if (!StateRankComputed)
        {
	  for(unsigned int k=0;k<3;k++)
            {
	      m_Forces(i,k)=0.0;
	      m_Torques(i,k)=0.0;
            }
        }
    }
}
