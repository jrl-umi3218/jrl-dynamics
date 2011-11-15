/*
 * Copyright 2009, 2010,
 *
 * Oussama Kanoun
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

bool DynMultiBodyPrivate::getJacobian ( const CjrlJoint& inStartJoint,
                                        const CjrlJoint& inEndJoint,
                                        const vector3d& inFrameLocalPosition,
                                        matrixNxP& outjacobian,
                                        unsigned int outOffset,
                                        bool includeFreeFlyer )
{
  unsigned int valNumberDof = ( includeFreeFlyer==true ) ?numberDof() :numberDof()-6;
  unsigned int lengthJacobian = MAL_MATRIX_NB_COLS ( outjacobian );
  unsigned int reqJacobianLength = valNumberDof+outOffset;
  unsigned int nbRows = MAL_MATRIX_NB_ROWS ( outjacobian );
  if ( ( nbRows != 6 ) ||
       ( lengthJacobian < reqJacobianLength ) )
    return false;

  unsigned int i,j;
  // This function should compute a jacobian and let the rest untouched.
  for(i=0;
      i<nbRows;
      i++)
    {
      for(j=outOffset;
	  j<reqJacobianLength;
	  j++)
        {
	  outjacobian(i,j)=0.0;
        }
    }

  //determine participating joints
  std::vector<CjrlJoint *> robotRoot2StartJoint, robotRoot2EndJoint;
  const JointPrivate* StartJoint = (const JointPrivate*) &inStartJoint;
  const JointPrivate* EndJoint =  (const JointPrivate* )&inEndJoint;

  if (StartJoint==0)
    LTHROW("getJacobian: Starting joint not good" );
  if (EndJoint==0)
    LTHROW("getJacobian: End joint not good" );

  robotRoot2StartJoint = StartJoint->jointsFromRootToThis();
  robotRoot2EndJoint = EndJoint->jointsFromRootToThis();

  //std::cout << "length of route of the start joint = " << robotRoot2StartJoint.size() << std::endl;
  //std::cout << "length of route of the end joint = " << robotRoot2EndJoint.size() << std::endl;

  unsigned int offset = 1;
  unsigned int minChain = ( robotRoot2StartJoint.size() <robotRoot2EndJoint.size() )
    ?robotRoot2StartJoint.size() :robotRoot2EndJoint.size();

  for ( i=1; i< minChain; i++ )
    {
      if ( ( robotRoot2StartJoint[i]==robotRoot2EndJoint[i] ) )
	offset++;
    }

  unsigned int rank;
  JointPrivate* aJoint;
  DynamicBodyPrivate* aBody = EndJoint->linkedDBody();
  tempP = aBody->p + MAL_S3x3_RET_A_by_B ( aBody->R, inFrameLocalPosition );

  for ( i=offset;i<robotRoot2EndJoint.size();i++ )
    {
      aJoint= ( JointPrivate * ) robotRoot2EndJoint[i];
      aBody=  aJoint->linkedDBody();
      if ( includeFreeFlyer )
	rank = aJoint->rankInConfiguration()+outOffset;
      else
	rank = aJoint->rankInConfiguration()-6+outOffset;

      tempDP = tempP - aBody->p;

      //std::cout << "rank of endjoint_route = " << rank << std::endl;

      switch ( aJoint->type() )
        {
	case JointPrivate::REVOLUTE_JOINT:
	  MAL_S3_VECTOR_CROSS_PRODUCT ( tempLV,aBody->w_a,tempDP );
	  for ( j=0;j<3;j++ )
	    {
	      outjacobian ( j,rank ) = tempLV[j];
	      outjacobian ( j+3,rank ) = aBody->w_a[j];
	    }
	  break;
	case JointPrivate::PRISMATIC_JOINT:
	  for ( j=0;j<3;j++ )
	    {
	      outjacobian ( j,rank ) = aBody->w_a[j];
	    }
	  break;
	case JointPrivate::FREE_JOINT:
	  for ( j=0;j<3;j++ )
	    {
	      outjacobian ( j,rank+j ) = 1.0;
	      outjacobian ( j+3,rank+j+3 ) = 1.0;
	    }

	  outjacobian ( 0,rank+3 ) = 0.0;
	  outjacobian ( 0,rank+4 ) = tempDP[2];
	  outjacobian ( 0,rank+5 ) = -tempDP[1];
	  outjacobian ( 1,rank+3 ) = -tempDP[2];
	  outjacobian ( 1,rank+4 ) = 0.0;
	  outjacobian ( 1,rank+5 ) = tempDP[0];

	  outjacobian ( 2,rank+3 ) = tempDP[1];
	  outjacobian ( 2,rank+4 ) = -tempDP[0];
	  outjacobian ( 2,rank+5 ) = 0.0;
	  break;
        }
    }

  for ( i=offset;i<robotRoot2StartJoint.size();i++ )
    {
      aJoint = ( JointPrivate * ) robotRoot2StartJoint[i];
      aBody=  aJoint->linkedDBody();
      if ( includeFreeFlyer )
	rank = aJoint->rankInConfiguration()+outOffset;
      else
	rank = aJoint->rankInConfiguration()-6+outOffset;

		
      //std::cout << "rank of startjoint_route = " << rank << std::endl;

      tempDP = tempP - aBody->p;

      switch ( aJoint->type() )
        {
	case JointPrivate::REVOLUTE_JOINT:
	  MAL_S3_VECTOR_CROSS_PRODUCT ( tempLV,aBody->w_a,tempDP );
	  for ( j=0;j<3;j++ )
	    {
	      outjacobian ( j,rank ) = -tempLV[j];
	      outjacobian ( j+3,rank ) = -aBody->w_a[j];

	    }
	  break;
	case JointPrivate::PRISMATIC_JOINT:
	  for ( j=0;j<3;j++ )
	    {
	      outjacobian ( j,rank ) = -aBody->w_a[j];
	    }
	  break;
	case JointPrivate::FREE_JOINT:
	  for ( j=0;j<3;j++ )
	    {
	      outjacobian ( j,rank+j ) = -1.0;
	      outjacobian ( j+3,rank+j+3 ) = -1.0;
	    }
	  outjacobian ( 0,rank+3 ) = 0.0;
	  outjacobian ( 0,rank+4 ) = -tempDP[2];
	  outjacobian ( 0,rank+5 ) = tempDP[1];
	  outjacobian ( 1,rank+3 ) = tempDP[2];
	  outjacobian ( 1,rank+4 ) = 0.0;
	  outjacobian ( 1,rank+5 ) = -tempDP[0];
	  outjacobian ( 2,rank+3 ) = -tempDP[1];
	  outjacobian ( 2,rank+4 ) = tempDP[0];
	  outjacobian ( 2,rank+5 ) = 0.0;
	  break;
        }
    }

  if ( includeFreeFlyer )
    {
      tempDP = tempP - StartJoint->linkedDBody()->p;

      //std::cout << "x_endjoint - x_startjoint = " << tempDP << std::endl;

      unsigned int ii,jj;
      for (ii=0;ii<6;ii++)
        {
	  for (jj=outOffset;jj<outOffset+6;jj++)
	    outjacobian(ii,jj) = 0.0;
	  outjacobian(ii,outOffset+ii) = 1.0;
        }

      unsigned k = outOffset+3;
      outjacobian ( 1,k ) = -tempDP[2]; outjacobian ( 2,k ) = tempDP[1];k++;
      outjacobian ( 0,k ) = tempDP[2]; outjacobian ( 2,k ) = -tempDP[0];k++;
      outjacobian ( 0,k ) =  -tempDP[1]; outjacobian ( 1,k ) =  tempDP[0];;

    }

  return true;
}

bool DynMultiBodyPrivate::getPositionJacobian ( const CjrlJoint& inStartJoint,
						const CjrlJoint& inEndJoint,
						const vector3d& inFrameLocalPosition,
						matrixNxP& outjacobian,
						unsigned int outOffset,
						bool includeFreeFlyer )
{

  unsigned int valNumberDof = ( includeFreeFlyer==true ) ?numberDof() :numberDof()-6;
  unsigned int lengthJacobian = MAL_MATRIX_NB_COLS ( outjacobian );
  unsigned int reqJacobianLength = valNumberDof+outOffset;
  unsigned int nbRows = MAL_MATRIX_NB_ROWS ( outjacobian );
  if ( ( nbRows != 3 ) ||
       ( lengthJacobian < reqJacobianLength ) )
    return false;

  unsigned int i,j;
  // This function should compute a jacobian and let the rest untouched.
  for(i=0;
      i<nbRows;
      i++)
    {
      for(j=outOffset;
	  j<reqJacobianLength;
	  j++)
        {
	  outjacobian(i,j)=0.0;
        }

    }


  //determine participating joints
  std::vector<CjrlJoint *> robotRoot2StartJoint, robotRoot2EndJoint;
  const JointPrivate* StartJoint = ( const JointPrivate* ) ( &inStartJoint );
  const JointPrivate* EndJoint = ( const JointPrivate* ) ( &inEndJoint );
  robotRoot2StartJoint = StartJoint->jointsFromRootToThis();
  robotRoot2EndJoint = EndJoint->jointsFromRootToThis();

  unsigned int offset = 1;
  unsigned int minChain = ( robotRoot2StartJoint.size() <robotRoot2EndJoint.size() )
    ?robotRoot2StartJoint.size() :robotRoot2EndJoint.size();

  for ( i=1; i< minChain; i++ )
    {
      if ( ( robotRoot2StartJoint[i]==robotRoot2EndJoint[i] ) )
	offset++;
    }

  unsigned int rank;
  JointPrivate* aJoint;
  DynamicBodyPrivate* aBody = EndJoint->linkedDBody();
  tempP = aBody->p + MAL_S3x3_RET_A_by_B ( aBody->R, inFrameLocalPosition );

  for ( i=offset;i<robotRoot2EndJoint.size();i++ )
    {
      aJoint= ( JointPrivate * ) robotRoot2EndJoint[i];
      aBody=  aJoint->linkedDBody();
      if ( includeFreeFlyer )
	rank = aJoint->rankInConfiguration()+outOffset;
      else
	rank = aJoint->rankInConfiguration()-6+outOffset;

      tempDP = tempP - aBody->p;

      switch ( aJoint->type() )
        {
	case JointPrivate::REVOLUTE_JOINT:
	  MAL_S3_VECTOR_CROSS_PRODUCT ( tempLV,aBody->w_a,tempDP );
	  for ( j=0;j<3;j++ )
	    outjacobian ( j,rank ) =  tempLV[j];
	  break;
	case JointPrivate::PRISMATIC_JOINT:
	  for ( j=0;j<3;j++ )
	    outjacobian ( j,rank ) = aBody->w_a[j];
	  break;
	case JointPrivate::FREE_JOINT:
	  for ( j=0;j<3;j++ )
	    outjacobian ( j,rank+j ) = 1.0;
	  outjacobian ( 1,rank+3 ) =  -tempDP[2];
	  outjacobian ( 2,rank+3 ) =  tempDP[1];
	  outjacobian ( 0,rank+4 ) =  tempDP[2];
	  outjacobian ( 2,rank+4 ) =  -tempDP[0];
	  outjacobian ( 0,rank+5 ) =  -tempDP[1];
	  outjacobian ( 1,rank+5 ) =  tempDP[0];
	  break;
        }
    }

  for ( i=offset;i<robotRoot2StartJoint.size();i++ )
    {
      aJoint = ( JointPrivate * ) robotRoot2StartJoint[i];
      aBody=  aJoint->linkedDBody();
      if ( includeFreeFlyer )
	rank = aJoint->rankInConfiguration() +outOffset;
      else
	rank = aJoint->rankInConfiguration()-6+outOffset;

      tempDP = tempP - aBody->p;

      switch ( aJoint->type() )
        {
	case JointPrivate::REVOLUTE_JOINT:
	  MAL_S3_VECTOR_CROSS_PRODUCT ( tempLV,aBody->w_a,tempDP );
	  for ( j=0;j<3;j++ )
	    outjacobian ( j,rank ) = -tempLV[j];
	  break;
	case JointPrivate::PRISMATIC_JOINT:
	  for ( j=0;j<3;j++ )
	    outjacobian ( j,rank ) = -aBody->w_a[j];
	  break;
	case JointPrivate::FREE_JOINT:
	  for ( j=0;j<3;j++ )
	    outjacobian ( j,rank+j ) = -1.0;

	  outjacobian ( 1,rank+3 ) =  tempDP[2];
	  outjacobian ( 2,rank+3 ) =  -tempDP[1];
	  outjacobian ( 0,rank+4 ) =  -tempDP[2];
	  outjacobian ( 2,rank+4 ) =  tempDP[0];
	  outjacobian ( 0,rank+5 ) =  tempDP[1];
	  outjacobian ( 1,rank+5 ) =  -tempDP[0];
	  break;
        }
    }
  if ( includeFreeFlyer && rootJoint()->numberDof() > 0)
    {
      tempDP = tempP - StartJoint->linkedDBody()->p;

      unsigned k = outOffset;
      outjacobian(0,k) = 1.0; outjacobian(1,k) = 0.0; outjacobian(2,k) = 0.0; k++;
      outjacobian(0,k) = 0.0; outjacobian(1,k) = 1.0; outjacobian(2,k) = 0.0; k++;
      outjacobian(0,k) = 0.0; outjacobian(1,k) = 0.0; outjacobian(2,k) = 1.0; k++;

      outjacobian ( 0,k ) =  0.0; outjacobian ( 1,k ) = -tempDP[2]; outjacobian ( 2,k ) = tempDP[1];k++;
      outjacobian ( 0,k ) = tempDP[2]; outjacobian ( 1,k ) =  0.0; outjacobian ( 2,k ) = -tempDP[0];k++;
      outjacobian ( 0,k ) =  -tempDP[1]; outjacobian ( 1,k ) =  tempDP[0]; outjacobian ( 2,k ) =  0.0;

    }
  return true;
}

bool DynMultiBodyPrivate::getOrientationJacobian ( const CjrlJoint& inStartJoint,
						   const CjrlJoint& inEndJoint,
						   matrixNxP& outjacobian,
						   unsigned int outOffset,
						   bool includeFreeFlyer )
{
  unsigned int valNumberDof = ( includeFreeFlyer==true ) ?numberDof() :numberDof()-6;
  unsigned int lengthJacobian = MAL_MATRIX_NB_COLS ( outjacobian );
  unsigned int reqJacobianLength = valNumberDof+outOffset;
  unsigned int nbRows = MAL_MATRIX_NB_ROWS ( outjacobian );
  if ( ( nbRows != 3 ) ||
       ( lengthJacobian < reqJacobianLength ) )
    return false;

  unsigned int i,j;
  // This function should compute a jacobian and let the rest untouched.
  for(i=0;
      i<nbRows;
      i++)
    {
      for(j=outOffset;
	  j<reqJacobianLength;
	  j++)
        {
	  outjacobian(i,j)=0.0;
        }

    }


  //determine participating joints
  std::vector<CjrlJoint *> robotRoot2StartJoint, robotRoot2EndJoint;
  const JointPrivate* StartJoint = dynamic_cast<const JointPrivate* > ( &inStartJoint );
  const JointPrivate* EndJoint = dynamic_cast<const JointPrivate* > ( &inEndJoint );
  robotRoot2StartJoint = StartJoint->jointsFromRootToThis();
  robotRoot2EndJoint = EndJoint->jointsFromRootToThis();

  unsigned int offset = 1;
  unsigned int minChain = ( robotRoot2StartJoint.size() <robotRoot2EndJoint.size() )
    ?robotRoot2StartJoint.size() :robotRoot2EndJoint.size();

  for ( i=1; i< minChain; i++ )
    {
      if ( ( robotRoot2StartJoint[i]==robotRoot2EndJoint[i] ) )
	offset++;
    }

  unsigned int rank;
  JointPrivate* aJoint;
  DynamicBodyPrivate* aBody;

  for ( i=offset;i<robotRoot2EndJoint.size();i++ )
    {
      aJoint= ( JointPrivate * ) robotRoot2EndJoint[i];
      aBody=  aJoint->linkedDBody();
      if ( includeFreeFlyer )
	rank = aJoint->rankInConfiguration() +outOffset;
      else
	rank = aJoint->rankInConfiguration()-6+outOffset;

      switch ( aJoint->type() )
        {
	case JointPrivate::REVOLUTE_JOINT:
	  for ( j=0;j<3;j++ )
	    outjacobian ( j,rank ) = aBody->w_a[j];

	  break;
	case JointPrivate::PRISMATIC_JOINT:
	  break;
	case JointPrivate::FREE_JOINT:
	  for ( j=0;j<3;j++ )
	    outjacobian ( j,rank+j ) =  1.0;
	  break;
        }
    }

  for ( i=offset;i<robotRoot2StartJoint.size();i++ )
    {
      aJoint = ( JointPrivate * ) robotRoot2StartJoint[i];
      aBody=  aJoint->linkedDBody();
      if ( includeFreeFlyer )
	rank = aJoint->rankInConfiguration() +outOffset;
      else
	rank = aJoint->rankInConfiguration()-6+outOffset;

      tempDP = tempP - aBody->p;

      switch ( aJoint->type() )
        {
	case JointPrivate::REVOLUTE_JOINT:
	  for ( j=0;j<3;j++ )
	    outjacobian ( j,rank ) = -aBody->w_a[j];
	  break;
	case JointPrivate::PRISMATIC_JOINT:
	  break;
	case JointPrivate::FREE_JOINT:
	  for ( j=0;j<3;j++ )
	    outjacobian ( j,rank+j ) =  -1.0;
	  break;
        }
    }

  if ( includeFreeFlyer )
    {
      unsigned int ii,jj;
      for (ii=0;ii<3;ii++)
	for (jj=outOffset;jj<outOffset+3;jj++)
	  outjacobian(ii,jj) = 0.0;

      unsigned k = outOffset+3;
      outjacobian(0,k) = 1.0; outjacobian(1,k) = 0.0; outjacobian(2,k) = 0.0; k++;
      outjacobian(0,k) = 0.0; outjacobian(1,k) = 1.0; outjacobian(2,k) = 0.0; k++;
      outjacobian(0,k) = 0.0; outjacobian(1,k) = 0.0; outjacobian(2,k) = 1.0;
    }
  return true;
}
