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
  /** Intermediate variables. The mantra is :
      "To optimize those variables, in the Compiler we trust"
      (with the appropriate compilation options).
  */
  vector3d NE_tmp3, NE_tmp2, NE_wn,NE_cl, NE_lw_c, NE_aRc, NE_aRb,  
    NE_lpComP, NE_RotByMotherdv, NE_lP,NE_lL, NE_tmp;
  matrix3d NE_Rtmp, NE_Rt, NE_Ro, NE_Rot;
  /* End of intermediate */

  double norm_w, th;
  int currentNode = labelTheRoot;
  int lMother=0;
  unsigned int i,j;
  double NORME_EPSILON=1e-6;
  DynamicBodyPrivate *aDB;

  m_listOfBodies[labelTheRoot]->p = PosForRoot;
  m_listOfBodies[labelTheRoot]->v0 = v0ForRoot;
  m_listOfBodies[labelTheRoot]->R = OrientationForRoot;
  MAL_S3x3_MATRIX_SET_IDENTITY(m_listOfBodies[labelTheRoot]->R_static);
  m_listOfBodies[labelTheRoot]->w = wForRoot;
  m_listOfBodies[labelTheRoot]->dv = dvForRoot;
  m_listOfBodies[labelTheRoot]->dw = dwForRoot;

  currentNode = m_listOfBodies[labelTheRoot]->child;

  if (m_ComputeMomentum)
    {
      MAL_S3_VECTOR_FILL(m_P,0);
      MAL_S3_VECTOR_FILL(m_L,0);
    }

  positionCoMPondere[0] = 0;
  positionCoMPondere[1] = 0;
  positionCoMPondere[2] = 0;

  ODEBUG("PosForRoot: " << PosForRoot );
  ODEBUG("v0ForRoot: " << v0ForRoot );
  ODEBUG("OrientationForRoot: " << OrientationForRoot );
  do
    {

      aDB = m_listOfBodies[currentNode];

      norm_w = MAL_S3_VECTOR_NORM(aDB->a);
      lMother = aDB->getLabelMother();

      ODEBUG("CurrentBody " << m_listOfBodies[currentNode]->getName());

      // ----------------------------------
      // Rodrigues formula. (p33)
      if (norm_w< NORME_EPSILON)
        {
	  MAL_S3x3_MATRIX_SET_IDENTITY(NE_Ro);
        }
      else
        {
	  th = norm_w * aDB->q;
	  NE_wn = aDB->a / norm_w;
	  ODEBUG("aDB->a :" << aDB->a );
	  ODEBUG("norm_w:" << norm_w);

	  double ct = cos(th);
	  double lct= (1-ct);
	  double st = sin(th);
	  NE_Ro(0,0) = ct + NE_wn[0]*NE_wn[0]* lct;
	  NE_Ro(0,1) = NE_wn[0]*NE_wn[1]*lct-NE_wn[2]*st;
	  NE_Ro(0,2) = NE_wn[1] * st+NE_wn[0]*NE_wn[2]*lct;
	  NE_Ro(1,0) = NE_wn[2]*st +NE_wn[0]*NE_wn[1]*lct;
	  NE_Ro(1,1) = ct + NE_wn[1]*NE_wn[1]*lct;
	  NE_Ro(1,2) = -NE_wn[0]*st+NE_wn[1]*NE_wn[2]*lct;
	  NE_Ro(2,0) = -NE_wn[1]*st+NE_wn[0]*NE_wn[2]*lct;
	  NE_Ro(2,1) = NE_wn[0]*st + NE_wn[1]*NE_wn[2]*lct;
	  NE_Ro(2,2) = ct + NE_wn[2]*NE_wn[2]*lct;
        }

      ODEBUG("Ro:" << endl << NE_Ro );
      ODEBUG("MR:" << m_listOfBodies[lMother]->R );
      ODEBUG("b: " << aDB->b);
      ODEBUG("Mp: " << m_listOfBodies[lMother]->p);
      // End Rodrigues formula
      //-------------------------------

      // Position and orientation in reference frame
      m_listOfBodies[currentNode]->p = 
	MAL_S3x3_RET_A_by_B(m_listOfBodies[lMother]->R , aDB->b )
	+ m_listOfBodies[lMother]->p;
      MAL_S3x3_C_eq_A_by_B(NE_Rtmp ,m_listOfBodies[lMother]->R , 
			   m_listOfBodies[currentNode]->R_static);
      MAL_S3x3_C_eq_A_by_B(m_listOfBodies[currentNode]->R ,NE_Rtmp , NE_Ro);
      m_listOfBodies[currentNode]->Riip1 = NE_Ro;

      for( i=0;i<3;i++)
	for( j=0;j<3;j++)
	  MAL_S4x4_MATRIX_ACCESS_I_J(m_listOfBodies[currentNode]->m_transformation,i,j) 
	    = m_listOfBodies[currentNode]->R(i,j);

      for( i=0;i<3;i++)
	MAL_S4x4_MATRIX_ACCESS_I_J(m_listOfBodies[currentNode]->m_transformation,i,3) 
	  = m_listOfBodies[currentNode]->p(i);

      ODEBUG("q: "<< aDB->q );
      ODEBUG("p: "
	     << m_listOfBodies[currentNode]->p[0] << " "
	     << m_listOfBodies[currentNode]->p[1] << " "
	     << m_listOfBodies[currentNode]->p[2] << " " );
      ODEBUG(m_listOfBodies[currentNode]->getName());


      //update the translation/rotation axis of joint
      ODEBUG("a:" << endl << aDB->a );
      MAL_S3x3_C_eq_A_by_B(m_listOfBodies[currentNode]->w_a,
			   m_listOfBodies[currentNode]->R, aDB->a);

      if (m_ComputeVelocity)
        {

	  ODEBUG("dq: "<< aDB->dq );
	  NE_tmp = m_listOfBodies[currentNode]->a * m_listOfBodies[currentNode]->dq;
	  NE_tmp = MAL_S3x3_RET_A_by_B(m_listOfBodies[currentNode]->R,NE_tmp);

	  m_listOfBodies[currentNode]->w  = m_listOfBodies[lMother]->w  + NE_tmp;

	  ODEBUG("w: " << m_listOfBodies[currentNode]->w );

	  // Computes the linear velocity.
	  MAL_S3x3_C_eq_A_by_B(NE_tmp,m_listOfBodies[lMother]->R,
			       m_listOfBodies[currentNode]->b);

	  MAL_S3_VECTOR_CROSS_PRODUCT(NE_tmp2,m_listOfBodies[lMother]->w , NE_tmp);

	  m_listOfBodies[currentNode]->v0 = m_listOfBodies[lMother]->v0 + NE_tmp2;

	  ODEBUG("v0: "
		 << m_listOfBodies[currentNode]->v0[0] << " "
		 << m_listOfBodies[currentNode]->v0[1] << " "
		 << m_listOfBodies[currentNode]->v0[2] << " " );
        }

      vector3d lc = aDB->localCenterOfMass();
	    
      // Computes also the center of mass in the reference frame.
      if (m_ComputeCoM)
        {
	  ODEBUG(" c: " << lc);
	  MAL_S3x3_C_eq_A_by_B(NE_cl,m_listOfBodies[currentNode]->R,lc);
	  NE_lw_c = NE_cl + m_listOfBodies[currentNode]->p;
	  ODEBUG(" lw_c: "<< currentNode <<" "<< NE_lw_c[0] 
		 << " " << NE_lw_c[1] << " " << NE_lw_c[2]);
	  positionCoMPondere +=  
	    NE_lw_c * m_listOfBodies[currentNode]->getMasse();
	  ODEBUG(" w_c: " << NE_lw_c[0] << " " 
		 << NE_lw_c[1] << " " << NE_lw_c[2]);
	  ODEBUG(" Masse " << m_listOfBodies[currentNode]->getMasse());
	  ODEBUG(" positionCoMPondere " << positionCoMPondere);
	  m_listOfBodies[currentNode]->w_c = NE_lw_c;
        }
      if (m_ComputeMomentum)
        {

	  // Computes momentum matrix P.
	  ODEBUG("w: " << m_listOfBodies[currentNode]->w );
	  MAL_S3_VECTOR_CROSS_PRODUCT(NE_tmp,m_listOfBodies[currentNode]->w, NE_cl);
	  ODEBUG("cl^w: " << NE_tmp);
	  ODEBUG("masse: " << m_listOfBodies[currentNode]->getMasse());
	  ODEBUG("v0: " << m_listOfBodies[currentNode]->v0 );
	  NE_lP=  (m_listOfBodies[currentNode]->v0 +
		   NE_tmp )* m_listOfBodies[currentNode]->getMasse();
	  m_listOfBodies[currentNode]->P = NE_lP;
	  ODEBUG("P: " << NE_lP );
	  m_P += NE_lP;

	  // Computes angular momentum matrix L
	  // Lk = xc x Pk + R * I * Rt * w
	  MAL_S3x3_TRANSPOSE_A_in_At(m_listOfBodies[currentNode]->R,NE_Rt);

	  MAL_S3_VECTOR_CROSS_PRODUCT(NE_tmp3,NE_lw_c,NE_lP);

	  MAL_S3x3_C_eq_A_by_B(NE_tmp2,NE_Rt , m_listOfBodies[currentNode]->w);
	  MAL_S3x3_C_eq_A_by_B(NE_tmp, m_listOfBodies[currentNode]->getInertie(),NE_tmp2);
	  MAL_S3x3_C_eq_A_by_B(NE_tmp2, m_listOfBodies[currentNode]->R,NE_tmp);
	  NE_lL = NE_tmp3 + NE_tmp2;
	  ODEBUG("L: " << lL);

	  m_listOfBodies[currentNode]->L = NE_lL;
	  m_L+= NE_lL;

        }

      if (m_ComputeAcceleration)
        {
	  // ******************* Computes the angular acceleration for joint i. ********************
	  // NE_tmp2 = z_{i-1} * dqi
	  NE_tmp2 = m_listOfBodies[currentNode]->w_a * m_listOfBodies[currentNode]->dq;
	  // NE_tmp3 = w^{(0)}_i x z_{i-1} * dqi
	  MAL_S3_VECTOR_CROSS_PRODUCT(NE_tmp3,m_listOfBodies[currentNode]->w,NE_tmp2);
	  // NE_tmp2 = z_{i-1} * ddqi
	  NE_tmp2 = m_listOfBodies[currentNode]->w_a * m_listOfBodies[currentNode]->ddq;
	  m_listOfBodies[currentNode]->dw = NE_tmp2 + NE_tmp3 + m_listOfBodies[lMother]->dw;

	  // ******************* Computes the linear acceleration for joint i. ********************
	  MAL_S3x3_C_eq_A_by_B(NE_aRb, m_listOfBodies[currentNode]->R, aDB->b);
	  // NE_tmp3 = w_i x (w_i x r_{i,i+1})
	  MAL_S3_VECTOR_CROSS_PRODUCT(NE_tmp2,m_listOfBodies[currentNode]->w,NE_aRb);
	  MAL_S3_VECTOR_CROSS_PRODUCT(NE_tmp3,m_listOfBodies[currentNode]->w,NE_tmp2);

	  // NE_tmp2 = dw_I x r_{i,i+1}
	  MAL_S3_VECTOR_CROSS_PRODUCT(NE_tmp2,m_listOfBodies[currentNode]->dw,NE_aRb);
	  NE_Rot = MAL_S3x3_RET_TRANSPOSE(NE_Ro);
	  MAL_S3x3_C_eq_A_by_B(NE_RotByMotherdv,NE_Rot,m_listOfBodies[lMother]->dv);
	  m_listOfBodies[currentNode]->dv = NE_RotByMotherdv + NE_tmp2 + NE_tmp3;
        }

      if (m_ComputeAccCoM)
        {

	  // *******************  Acceleration for the center of mass of body  i ************************
	  MAL_S3x3_C_eq_A_by_B(NE_aRc, m_listOfBodies[currentNode]->R, lc);
	  MAL_S3_VECTOR_CROSS_PRODUCT(NE_tmp2,m_listOfBodies[currentNode]->w,NE_aRc);
	  MAL_S3_VECTOR_CROSS_PRODUCT(NE_tmp3,m_listOfBodies[currentNode]->w,NE_tmp2);

	  // NE_tmp2 = dw_I x r_{i,i+1}
	  MAL_S3_VECTOR_CROSS_PRODUCT(NE_tmp2,m_listOfBodies[currentNode]->dw,NE_aRc);
	  //
	  m_listOfBodies[currentNode]->dv_c = NE_RotByMotherdv + NE_tmp2 + NE_tmp3;
        }

      // TO DO if necessary : cross velocity.
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

  if (m_ComputeSkewCoM)
    {
      SkewCoM(0,0) =         0;
      SkewCoM(0,1) = - positionCoMPondere[2];
      SkewCoM(0,2) = positionCoMPondere[1];
      SkewCoM(1,0) = positionCoMPondere[2];
      SkewCoM(1,1) =           0;
      SkewCoM(1,2) =-positionCoMPondere[0];
      SkewCoM(2,0) =-positionCoMPondere[1];
      SkewCoM(2,1) =   positionCoMPondere[0];
      SkewCoM(2,2) =         0;
    }

  positionCoMPondere = positionCoMPondere/masse;
  // Zero Momentum Point Computation.
  if (m_ComputeZMP)
    {

      ODEBUG4( m_P << " " << m_L,"DebugDataPL.dat");
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

      ODEBUG4( m_IterationNumber << " "
	       << m_ZMP(0) << " "
	       << m_ZMP(1) << " "
	       << m_ZMP(2) << " "
	       << m_P << " "
	       << m_L ,"DebugDataDMB_ZMP.dat" );

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
      lLinearVelocityForRoot(i)=m_Velocity(i);
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
