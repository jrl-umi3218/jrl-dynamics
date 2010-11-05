/*
 * Copyright 2009, 2010, 
 *
 * Paul Evrard
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
#include "abstract-robot-dynamics/jrlbody.hh"

#include "fileReader.h"

using namespace dynamicsJRLJapan;


MAL_MATRIX(,double) &DynMultiBodyPrivate::getJacobianOfTheCoM()
{
  return m_JacobianOfTheCoM;
}

/**
   \brief Get the acceleration of the center of mass.
*/
const MAL_S3_VECTOR(,double)& DynMultiBodyPrivate::accelerationCenterOfMass()
{
  return m_AccelerationCenterOfMass;
};

/**
   \brief Get the position of the center of mass.
*/
const MAL_S3_VECTOR(,double)&  DynMultiBodyPrivate::positionCenterOfMass() const
{
  return positionCoMPondere;
}


/**
   \brief Get the velocity of the center of mass.
*/
const MAL_S3_VECTOR(,double)& DynMultiBodyPrivate::velocityCenterOfMass()
{
  return m_VelocityCenterOfMass;
};


bool DynMultiBodyPrivate::getJacobianCenterOfMass ( const CjrlJoint& inStartJoint, 
						 matrixNxP& outjacobian, 
						 unsigned int outOffset, 
						 bool includeFreeFlyer )
{
  unsigned int valNumberDof = ( includeFreeFlyer==true ) ?numberDof() :numberDof()-6;
  unsigned int lengthJacobian = MAL_MATRIX_NB_COLS(outjacobian);
  if ( ( MAL_MATRIX_NB_ROWS(outjacobian) != 3 ) || ( lengthJacobian < valNumberDof + outOffset ) )
    return false;

  unsigned int i,j;
  double ** outTable;
  outTable = new double* [3];
  for ( i=0; i<3; i++ )
    outTable[i] = new double [valNumberDof];

  for ( i=0; i<3; i++ )
    memset ( outTable[i], 0, valNumberDof*sizeof ( double ) );

  std::vector<int> jointsigns ( numberDof(),1 );

  JointPrivate* StartJoint = ( JointPrivate* ) ( &inStartJoint );
  //determine participating joints
  if ( rootJoint() !=&inStartJoint )
    {
      std::vector<CjrlJoint *> robotRoot2StartJoint = StartJoint->jointsFromRootToThis();

      for ( i = 1; i<robotRoot2StartJoint.size();i++ )
	jointsigns[robotRoot2StartJoint[i]->rankInConfiguration() ] = -1;

      std::vector<vector3d> tempoS;
      std::vector<double> liftedS;

      for ( i = 0; i<robotRoot2StartJoint.size()-1;i++ )
        {
	  JointPrivate *aJoint = (JointPrivate *)robotRoot2StartJoint[i],
	    *aJointp1 = (JointPrivate *)robotRoot2StartJoint[i+1];
	  aJoint->computeSubTreeMComExceptChild ( aJointp1);
	  tempoS.push_back ( aJoint->subTreeMCom() );
	  liftedS.push_back ( aJoint->subTreeCoef() );
        }

      ((JointPrivate *)robotRoot2StartJoint[1])->subTreeMCom ( tempoS[0] );
      ((JointPrivate *)robotRoot2StartJoint[1])->subTreeCoef ( liftedS[0] );

      for ( i = 2; i<robotRoot2StartJoint.size();i++ )
        {
	  JointPrivate *aJoint = (JointPrivate *)robotRoot2StartJoint[i],
	    *aJointm1 = (JointPrivate *)robotRoot2StartJoint[i-1];
	  aJoint->subTreeMCom ( aJointm1->subTreeMCom() +tempoS[i-1] );
	  aJoint->subTreeCoef ( aJointm1->subTreeCoef() +liftedS[i-1] );
        }
    }
  else
    StartJoint->computeSubTreeMCom();

  unsigned int rank;
  JointPrivate* aJoint;
  DynamicBodyPrivate* aBody;

  for ( i=0;i<m_ConfigurationToJoints.size();i++ )
    {
      if ( m_ConfigurationToJoints[i] == rootJoint() )
	continue;

      aJoint = m_ConfigurationToJoints[i];
      aBody=  aJoint->linkedDBody();
      if ( includeFreeFlyer )
	rank = aJoint->rankInConfiguration();
      else
	rank = aJoint->rankInConfiguration()-6;

      switch ( aJoint->type() )
        {
	case JointPrivate::REVOLUTE_JOINT:
	  for ( j=0;j<3;j++ )
	    tempDP[j] = aJoint->subTreeMCom() [j]- aJoint->subTreeCoef() *aBody->p[j];
	  MAL_S3_VECTOR_CROSS_PRODUCT ( tempLV,aBody->w_a,tempDP );
	  for ( j=0;j<3;j++ )
	    if ( jointsigns[m_ConfigurationToJoints[i]->rankInConfiguration() ]>0 )
	      outTable[j][rank] =  tempLV[j];
	    else
	      outTable[j][rank] =  -tempLV[j];
	  break;
	case JointPrivate::PRISMATIC_JOINT:
	  for ( j=0;j<3;j++ )
	    {
	      if ( jointsigns[m_ConfigurationToJoints[i]->rankInConfiguration() ]>0 )
		outTable[j][rank] = aBody->w_a[j];
	      else
		outTable[j][rank] = -aBody->w_a[j];
	    }
	  break;
	case JointPrivate::FREE_JOINT:
	  for ( j=0;j<3;j++ )
	    {
	      if ( jointsigns[m_ConfigurationToJoints[i]->rankInConfiguration() ]>0 )
		outTable[j][rank+j] =  1.0;
	      else
		outTable[j][rank+j] =  -1.0;
	    }
	  for ( j=0;j<3;j++ )
	    tempDP[j] = aJoint->subTreeMCom() [j]- aJoint->subTreeCoef() *aBody->p[j];
	  if ( jointsigns[m_ConfigurationToJoints[i]->rankInConfiguration() ]>0 )
	    {
	      outTable[1][rank+3] =  -tempDP[2];
	      outTable[2][rank+3] =  tempDP[1];
	      outTable[0][rank+4] =  tempDP[2];
	      outTable[2][rank+4] =  -tempDP[0];
	      outTable[0][rank+5] =  -tempDP[1];
	      outTable[1][rank+5] =  tempDP[0];
	    }
	  else
	    {
	      outTable[1][rank+3] =  tempDP[2];
	      outTable[2][rank+3] =  -tempDP[1];
	      outTable[0][rank+4] =  -tempDP[2];
	      outTable[2][rank+4] =  tempDP[0];
	      outTable[0][rank+5] =  tempDP[1];
	      outTable[1][rank+5] =  -tempDP[0];
	    }

	  break;
        }

    }
  if ( includeFreeFlyer )
    {

      for ( j=0;j<3;j++ )
	outTable[j][j] =  1.0;
      tempDP = positionCoMPondere - StartJoint->linkedDBody()->p;
      outTable[1][3] =  -tempDP[2];
      outTable[2][3] =  tempDP[1];
      outTable[0][4] =  tempDP[2];
      outTable[2][4] =  -tempDP[0];
      outTable[0][5] =  -tempDP[1];
      outTable[1][5] =  tempDP[0];
    }
  for ( i=0; i<3; i++ )
    memcpy ( ( &outjacobian.data() [i*lengthJacobian+outOffset] ),outTable[i],valNumberDof *sizeof ( double ) );

  //clean
  for ( i=0; i<3; i++ )
    delete[] ( outTable[i] );
  delete[] ( outTable );
  return true;
}

void DynMultiBodyPrivate::getJacobianLinearMomentumWrtCoM(matrixNxP &outjacobian)
{
  matrixNxP JCoM;
  getJacobianCenterOfMass(*rootJoint(),JCoM);
  outjacobian = m_mass * JCoM;
}

/**
   @}
*/
/*
  \brief Compute the dynamics of the center of mass.
  
  Compute the linear and  angular momentum and their time derivatives, at the center of mass.
*/
bool DynMultiBodyPrivate::computeCenterOfMassDynamics()
{
  computeForwardKinematics();
  return true;
};


/**
   \brief Get the linear momentum of the robot.
*/
const MAL_S3_VECTOR(,double)& DynMultiBodyPrivate::linearMomentumRobot()
{
  return m_P;
};

/**
   \brief Get the time-derivative of the linear momentum.
*/
const MAL_S3_VECTOR(,double)& DynMultiBodyPrivate::derivativeLinearMomentum()
{
  return m_dP;
};

MAL_S3_VECTOR(,double) DynMultiBodyPrivate::getPositionCoM(void)
{
  return (positionCoMPondere);
}

void DynMultiBodyPrivate::GetPandL(MAL_S3_VECTOR(,double) &aP, MAL_S3_VECTOR(,double) &aL)
{
  aP = m_P;
  aL = m_L;
}
