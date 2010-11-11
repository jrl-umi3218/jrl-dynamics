/*
 * Copyright 2010,
 *
 * Olivier Stasse
 *
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
/* \file This part implements the re-normalization of the joint along the x-axis.
  Copyright (c) 2009,
*/

#include "Debug.h"

#include "JointPrivate.h"
#include "DynamicBodyPrivate.h"

using namespace dynamicsJRLJapan;


void JointPrivate::computeLocalAndGlobalPoseFromGlobalFrame()
{
  /*
    The pose of the joint has been defined in global frame at construction.
    Compute pose in local frame of parent joint.
    It is assumed that the rotation axis is already x.
  */

  /* Get global pose of parent joint */
  MAL_S4x4_MATRIX_TYPE( double) invParentGlobalPose;
  MAL_S4x4_INVERSE(m_FatherJoint->m_globalPoseAtConstruction, invParentGlobalPose, double);
  MAL_S4x4_MATRIX_TYPE( double) jointGlobalPose = m_globalPoseAtConstruction;
  /*
    parent     /  global \  -1   global
    R         = | R        |     R
    joint      \  parent /       joint
  */
  m_globalPoseAtConstructionNormalized = m_globalPoseAtConstruction;
  m_poseInParentFrame = MAL_S4x4_RET_A_by_B(invParentGlobalPose, jointGlobalPose);
  ODEBUG(" m_FatherJoint->m_globalPoseAtConstruction=" << m_FatherJoint->m_globalPoseAtConstruction);
  ODEBUG(" invParentGlobalPose=" << invParentGlobalPose);
  ODEBUG(" jointGlobalPose=" << jointGlobalPose);
  ODEBUG(" m_poseInParentFrame=" << m_poseInParentFrame);

  if (m_Body!=0)
    {
      DynamicBodyPrivate* aDBP=0;
      aDBP =dynamic_cast<DynamicBodyPrivate *>(m_Body);
      if (aDBP!=0)
	{
	  for (unsigned int i=0;i<3;i++)
	    {
	      MAL_S3_VECTOR_ACCESS(aDBP->b,i) =
		MAL_S4x4_MATRIX_ACCESS_I_J(m_poseInParentFrame,i,3);

	      for (unsigned int j=0;j<3;j++)
		{
		  MAL_S3x3_MATRIX_ACCESS_I_J(aDBP->R_static,i,j)=
		    MAL_S4x4_MATRIX_ACCESS_I_J(m_poseInParentFrame,i,j);
		}
	    }

	}
    }
}

void JointPrivate::NormalizeRotationFromAxis(vector4d &Axis, matrix3d &NormalizedRotation)
{
  // Start normalization.
  vector3d v1,v2,v3;

  v1[0] = Axis[0];
  v1[1] = Axis[1];
  v1[2] = Axis[2];
  v2[0] = v2[1]=v2[2]=0.0;
  unsigned int smallestComponent=0;
  double valueSmallestComponent = fabs(v1[0]);

  if (fabs(v1[1]) < fabs(v1[smallestComponent])) {
    smallestComponent = 1;
    valueSmallestComponent = fabs(v1[1]);
  }

  if (fabs(v1[2]) < fabs(v1[smallestComponent])) {
    smallestComponent = 2;
    valueSmallestComponent = fabs(v1[2]);
  }
  // (v1, v2, v3) form an orthonormal basis
  v2[smallestComponent] = 1.0;
  MAL_S3_VECTOR_CROSS_PRODUCT(v3,v1,v2);
  double n = MAL_S3_VECTOR_NORM(v3);
  v3 = v3/n;
  MAL_S3_VECTOR_CROSS_PRODUCT(v2,v3,v1);

  // Prepare the fixed rotation following the axis.
  for (unsigned int iRow=0; iRow < 3; iRow++) {
    MAL_S3x3_MATRIX_ACCESS_I_J(NormalizedRotation,iRow,0) = v1[iRow];
    MAL_S3x3_MATRIX_ACCESS_I_J(NormalizedRotation,iRow,1) = v2[iRow];
    MAL_S3x3_MATRIX_ACCESS_I_J(NormalizedRotation,iRow,2) = v3[iRow];
  }
}

void JointPrivate::computeLocalAndGlobalPoseFromLocalFrame()
{
  /*
    The pose of the joint has been defined in local frame of parent joint at construction.
    Compute pose in global frame.
  */

  /*
    global       global      global
    R         =  R           R
    joint        parent      joint
  */

  // Initial static rotation after the joint.
  matrix3d InitialRstatic;

  ODEBUG(getName() << " m_poseInParentFrame=" << m_poseInParentFrame);
  ODEBUG(" m_FatherJoint->m_globalPoseAtConstruction=" << m_FatherJoint->m_globalPoseAtConstruction);

  // Compute the global pose at construction by using
  // father global pose and local pose in the father's frame.
  MAL_S4x4_C_eq_A_by_B(m_globalPoseAtConstruction,
		       m_FatherJoint->m_globalPoseAtConstruction,
		       m_poseInParentFrame);

  if (m_Body!=0)
    {
      DynamicBodyPrivate * aDBP =dynamic_cast<DynamicBodyPrivate *>(m_Body);
      InitialRstatic = aDBP->R_static;
    }

  // Express local axis in the global frame.
  vector4d GlobalAxis,LocalAxis,GlobalCenter,LocalCenter;
  LocalAxis[0] = m_axis[0];
  LocalAxis[1] = m_axis[1];
  LocalAxis[2] = m_axis[2];
  LocalAxis[3] = 0;
  MAL_S4x4_C_eq_A_by_B(GlobalAxis,m_globalPoseAtConstruction,LocalAxis);

  matrix3d NormalizedRotation;
  NormalizeRotationFromAxis(GlobalAxis, NormalizedRotation);

  // Default value.

  MAL_S4x4_MATRIX_SET_IDENTITY(m_globalPoseAtConstructionNormalized);

  // Build normalized frame.
  for (unsigned int iRow=0; iRow < 3; iRow++) {
    MAL_S4x4_MATRIX_ACCESS_I_J(m_globalPoseAtConstructionNormalized,iRow, 0) =
      MAL_S3x3_MATRIX_ACCESS_I_J(NormalizedRotation,iRow,0);
    MAL_S4x4_MATRIX_ACCESS_I_J(m_globalPoseAtConstructionNormalized,iRow, 1) =
      MAL_S3x3_MATRIX_ACCESS_I_J(NormalizedRotation,iRow,1);
    MAL_S4x4_MATRIX_ACCESS_I_J(m_globalPoseAtConstructionNormalized,iRow, 2) =
      MAL_S3x3_MATRIX_ACCESS_I_J(NormalizedRotation,iRow,2);
    MAL_S4x4_MATRIX_ACCESS_I_J(m_globalPoseAtConstructionNormalized,iRow, 3) =
      MAL_S4x4_MATRIX_ACCESS_I_J(m_globalPoseAtConstruction,iRow,3);
  }

  MAL_S4x4_MATRIX_TYPE(double) poseInParentFrameUnnormalized;
  poseInParentFrameUnnormalized = m_poseInParentFrame;

  /* Get normalized global pose of parent joint
     to compute relative position in normalized local reference frame.  */
  MAL_S4x4_MATRIX_TYPE( double) invParentGlobalPoseN;
  MAL_S4x4_INVERSE(m_FatherJoint->m_globalPoseAtConstructionNormalized, invParentGlobalPoseN, double);
  MAL_S4x4_MATRIX_TYPE( double) jointGlobalPoseN = m_globalPoseAtConstructionNormalized;

  /*
    parent     /  global \  -1   global
    R         = | R        |     R
    joint      \  parent /       joint
  */
  m_poseInParentFrame = MAL_S4x4_RET_A_by_B(invParentGlobalPoseN, jointGlobalPoseN);

  // Rotate local center of mass and inertia matrix if present.
  if (m_Body!=0)
    {
      // Compute transformation for rotation from Unnormalized to Normalized.
      MAL_S4x4_MATRIX_TYPE( double) fromUnnormalizedToNormalized;
      MAL_S4x4_MATRIX_TYPE( double) invglobalPoseAtConstructionNormalized;
      MAL_S4x4_INVERSE(m_globalPoseAtConstructionNormalized,
		       invglobalPoseAtConstructionNormalized,
		       double);

      MAL_S4x4_C_eq_A_by_B(fromUnnormalizedToNormalized,
			   invglobalPoseAtConstructionNormalized,
			   m_globalPoseAtConstruction);

      // Put it in a 3x3 matrix.
      MAL_S3x3_MATRIX_TYPE( double) rotParams;
      for(unsigned int li=0;li<3;li++)
	for(unsigned int lj=0;lj<3;lj++)
	  MAL_S3x3_MATRIX_ACCESS_I_J(rotParams, li,lj) =
	    MAL_S4x4_MATRIX_ACCESS_I_J(fromUnnormalizedToNormalized, li,lj);

      MAL_S3x3_MATRIX_TYPE( double) trRotParams;
      MAL_S3x3_TRANSPOSE_A_in_At(rotParams,trRotParams);

      // Transform local parameters.
      // Com
      vector3d lcom = m_Body->localCenterOfMass();
      ODEBUG("old lcom: " << lcom );
      lcom = MAL_S3x3_RET_A_by_B(rotParams,lcom);
      m_Body->localCenterOfMass(lcom);
      ODEBUG("new lcom: " << lcom );
      ODEBUG("rotParams:" << rotParams);

      // Inertia matrix
      // Rotation using similarity transformation.
      matrix3d linertiam = m_Body->inertiaMatrix();
      ODEBUG(getName() << endl <<
	      "old inertia:" << endl <<
	      linertiam);
      linertiam = MAL_S3x3_RET_A_by_B(linertiam, trRotParams);
      linertiam = MAL_S3x3_RET_A_by_B(rotParams,linertiam);
      m_Body->inertiaMatrix(linertiam);
      ODEBUG("new inertia:" << endl << linertiam) ;

      DynamicBodyPrivate* aDBP=0;
      if (m_Body!=0)
	{
	  aDBP =dynamic_cast<DynamicBodyPrivate *>(m_Body);
	  if (aDBP!=0)
	    {
	      vector3d laxis;
	      laxis = m_axis;

	      for (unsigned int i=0;i<3;i++)
		{
		  MAL_S3_VECTOR_ACCESS(aDBP->b,i) =
		    MAL_S4x4_MATRIX_ACCESS_I_J(m_poseInParentFrame,i,3);

		  for (unsigned int j=0;j<3;j++)
		    MAL_S3x3_MATRIX_ACCESS_I_J(aDBP->R_static,i,j) =
		      MAL_S4x4_MATRIX_ACCESS_I_J(m_poseInParentFrame,i,j);
		}

	      MAL_S3_VECTOR_ACCESS(m_axis,0)=1.0;
	      MAL_S3_VECTOR_ACCESS(m_axis,1)=0.0;
	      MAL_S3_VECTOR_ACCESS(m_axis,2)=0.0;
	      aDBP->a = m_axis;
	    }
	}

    }
}

void JointPrivate::computeLocalAndGlobalPose()
{
  if (m_FatherJoint==0)
    {
      MAL_S4x4_MATRIX_SET_IDENTITY(m_globalPoseAtConstruction);
      MAL_S4x4_MATRIX_SET_IDENTITY(m_globalPoseAtConstructionNormalized);
      return;
    }

  if (m_inGlobalFrame)
    {
      computeLocalAndGlobalPoseFromGlobalFrame();
    }
  else
    {
      computeLocalAndGlobalPoseFromLocalFrame();
    }
  /*! Initialize spatial representation of the joint
    in the link reference frame. */
  initXL();
}
