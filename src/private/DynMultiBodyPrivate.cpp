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

#include "SpiritVRMLReader.h"

using namespace dynamicsJRLJapan;

DynMultiBodyPrivate::DynMultiBodyPrivate()
{
  m_NbDofs=0;
  m_ComputeVelocity = true;
  m_ComputeCoM = true;
  m_ComputeMomentum = true;
  m_ComputeZMP = false;
  m_ComputeAcceleration=false;
  m_ComputeAccCoM = false;
  m_ComputeBackwardDynamics=false;
  m_ComputeSkewCoM = true;
  m_IterationNumber = 0;
  m_TimeStep = 0.005;
  m_FileLinkJointRank="";

  labelTheRoot=1;

  // Create a default Root tree which
  // is a free joint.
  MAL_S3_VECTOR_TYPE(double) anAxis;

  m_RootOfTheJointsTree = 0;

  m_Prev_P(0) =   m_Prev_P(1) =   m_Prev_P(2) = 0.0;
  m_Prev_L(0) =   m_Prev_L(1) =   m_Prev_L(2) = 0.0;

  m_NbOfVRMLIDs = -1;

  m_SynchronizationBetweenActuatedVectorAndJoints = false;

  RESETDEBUG4("DebugDataPL.dat");
  RESETDEBUG4("DebugDataZMP.dat");
  RESETDEBUG4("DebugDataDMB_ZMP.dat");
}

DynMultiBodyPrivate::~DynMultiBodyPrivate()
{
  for(unsigned int li=0; li<m_listOfBodies.size();li++)
    {
      Body *body = m_listOfBodies[li];
      std::vector<Body *>::iterator it;
      for (it = listBodies.begin(); it != listBodies.end(); it++)
        {
	  if (*it == body)
            {
	      listBodies.erase(it);
	      break;
            }
        }
      delete m_listOfBodies[li];
    }
}


bool DynMultiBodyPrivate::initialize()
{
  setComputeZMP(true);
  InitializeFromJointsTree();
  unsigned int nbDof = numberDof();
  MAL_VECTOR_DIM(config, double, nbDof);
  MAL_VECTOR_FILL(config, 0);
  BuildLinkBetweenActuatedVectorAndJoints();
  currentConfiguration(config);
  computeForwardKinematics();
  return true;
}


void DynMultiBodyPrivate::parserVRML(string path,
				     const char *option,
				     vector<BodyGeometricalData> &aListOfURLs,
				     bool ReadGeometry)
{
  m_listOfBodies.clear();
  // It is assumed that the parser has done a proper feedback.
  if (dynamicsJRLJapan::VRMLReader::ParseVRMLFile(this,path, aListOfURLs,ReadGeometry)==0)
    return;
  CreatesTreeStructure(option);
}


double DynMultiBodyPrivate::Getq(int JointID) const
{
  if ((JointID>=0) &&
      ((unsigned int)JointID<m_listOfBodies.size()))
    return m_listOfBodies[ConvertIDInActuatedToBodyID[JointID]]->q;
  return 0.0;
}

void DynMultiBodyPrivate::Setq(int JointID,double q)
{
  if ((JointID>=0) &&
      ((unsigned int)JointID<m_listOfBodies.size()))
    {
      m_listOfBodies[ConvertIDInActuatedToBodyID[JointID]]->q= q;
      ((JointPrivate *)m_listOfBodies[ConvertIDInActuatedToBodyID[JointID]]->joint())->quantity(q);
    }
}

void DynMultiBodyPrivate::Setdq(int JointID, double dq)
{
  if ((JointID>=0) &&
      ((unsigned int)JointID<m_listOfBodies.size()))
    {
      m_listOfBodies[ConvertIDInActuatedToBodyID[JointID]]->dq= dq;
    }
}

void DynMultiBodyPrivate::Setv(int JointID, MAL_S3_VECTOR_TYPE(double) v0)
{
  if ((JointID>=0) &&
      ((unsigned int)JointID<m_listOfBodies.size()))
    m_listOfBodies[ConvertIDInActuatedToBodyID[JointID]]->v0 = v0;
}

MAL_S3_VECTOR_TYPE(double) DynMultiBodyPrivate::Getv(int JointID)
{
  if ((JointID>=0) &&
      ((unsigned int)JointID<m_listOfBodies.size()))
    return m_listOfBodies[ConvertIDInActuatedToBodyID[JointID]]->v0;

  MAL_S3_VECTOR(dr,double);
  dr[0] = 0.0;
  dr[1] = 0.0;
  dr[2] = 0.0;
  return dr;
}

MAL_S3_VECTOR_TYPE(double) DynMultiBodyPrivate::GetvBody(int BodyID)
{
  if ((BodyID>=0) &&
      ((unsigned int)BodyID<m_listOfBodies.size()))
    return m_listOfBodies[BodyID]->v0;
  MAL_S3_VECTOR(dr,double);
  dr[0] = 0.0;
  dr[1] = 0.0;
  dr[2] = 0.0;
  return dr;
}

MAL_S3_VECTOR_TYPE(double) DynMultiBodyPrivate::GetwBody(int BodyID)
{
  if ((BodyID>=0) &&
      ((unsigned int)BodyID<m_listOfBodies.size()))
    return m_listOfBodies[BodyID]->w;
  MAL_S3_VECTOR(dr,double);
  dr[0] = 0.0;
  dr[1] = 0.0;
  dr[2] = 0.0;
  return dr;
}

MAL_S3_VECTOR_TYPE(double) DynMultiBodyPrivate::Getw(int JointID)
{
  if ((JointID>=0) &&
      ((unsigned int)JointID<m_listOfBodies.size()))
    return m_listOfBodies[ConvertIDInActuatedToBodyID[JointID]]->w;

  MAL_S3_VECTOR(dr,double);
  dr[0] = 0.0;
  dr[1] = 0.0;
  dr[2] = 0.0;
  return dr;
}

void DynMultiBodyPrivate::Setw(int JointID, MAL_S3_VECTOR_TYPE(double) w)
{
  if ((JointID>=0) &&
      ((unsigned int)JointID<m_listOfBodies.size()))
    m_listOfBodies[ConvertIDInActuatedToBodyID[JointID]]->w = w;
}

MAL_S3_VECTOR_TYPE(double) DynMultiBodyPrivate::Getp(int JointID)
{
  MAL_S3_VECTOR(empty,double);
  if ((JointID>=0) &&
      ((unsigned int)JointID<m_listOfBodies.size()))
    return m_listOfBodies[ConvertIDInActuatedToBodyID[JointID]]->p;
  return empty;
}

void DynMultiBodyPrivate::Setp(int JointID, MAL_S3_VECTOR_TYPE(double) apos)
{
  if ((JointID>=0) &&
      ((unsigned int)JointID<m_listOfBodies.size()))
    m_listOfBodies[ConvertIDInActuatedToBodyID[JointID]]->p = apos;
}



void DynMultiBodyPrivate::CalculateZMP(double &px, double &py,
                                    MAL_S3_VECTOR_TYPE(double) dP,
                                    MAL_S3_VECTOR_TYPE(double) dL,
                                    double zmpz)
{
  double g= 9.81;

  px = ( g * positionCoMPondere[0]*m_mass +
	 zmpz * dP[0] - dL[1])/(m_mass * g + dP[2]);
  py = ( g * positionCoMPondere[1]*m_mass +
	 zmpz * dP[1] + dL[0])/(m_mass * g + dP[2]);

  ODEBUG(" CalculateZMP : Masse :"<< m_mass << " g:" << g  << " "
	 << dP << " " << dL << " " << positionCoMPondere );


}

string DynMultiBodyPrivate::GetName(int JointID)
{
  string empty;
  if ((JointID>=0) &&
      ((unsigned int)JointID<m_listOfBodies.size()))
    return m_listOfBodies[ConvertIDInActuatedToBodyID[JointID]]->getName();
  return empty;

}



MAL_S3_VECTOR_TYPE(double) DynMultiBodyPrivate::GetP(int JointID)
{
  MAL_S3_VECTOR(empty,double);
  if ((JointID>=0) &&
      ((unsigned int)JointID<m_listOfBodies.size()))
    return m_listOfBodies[ConvertIDInActuatedToBodyID[JointID]]->P;
  return empty;
}

double DynMultiBodyPrivate::Getdq(int JointID) const
{
  double empty=0.0;
  if ((JointID>=0) &&
      ((unsigned int)JointID<m_listOfBodies.size()))
    return m_listOfBodies[ConvertIDInActuatedToBodyID[JointID]]->dq;
  return empty;
}



void DynMultiBodyPrivate::SetRBody(int BodyID, MAL_S3x3_MATRIX_TYPE(double) R)
{
  if ((BodyID>=0) &&
      ((unsigned int)BodyID<m_listOfBodies.size()))
    m_listOfBodies[BodyID]->R = R;
}

/***********************************************/
/* Implementation of the generic JRL interface */
/***********************************************/

// Set the root joint of the robot.
void DynMultiBodyPrivate::rootJoint(CjrlJoint& inJoint)
{
  m_RootOfTheJointsTree = & (JointPrivate &)inJoint;

}

// Get the robot joint of the robot.
CjrlJoint* DynMultiBodyPrivate::rootJoint() const
{
  return m_RootOfTheJointsTree;
}

// Get a vector containning all the joints
std::vector<CjrlJoint*> DynMultiBodyPrivate::jointVector()
{
  return m_JointVector;
}

// Get a vector containning all the joints
const std::vector<CjrlJoint *> DynMultiBodyPrivate::jointVectorCst() const
{
  return m_JointVector;
}

std::vector<CjrlJoint*> DynMultiBodyPrivate::jointsBetween(const CjrlJoint& inStartJoint,
							const CjrlJoint& inEndJoint) const
{
  std::vector<CjrlJoint*> outJoints;
  std::vector<CjrlJoint *> robotRoot2StartJoint, robotRoot2EndJoint;
  robotRoot2StartJoint = inStartJoint.jointsFromRootToThis();
  robotRoot2EndJoint = inEndJoint.jointsFromRootToThis();

  unsigned int i;
  unsigned int lastCommonJointRank = 0;
  unsigned int minChainLength = (robotRoot2StartJoint.size()<robotRoot2EndJoint.size())
    ?robotRoot2StartJoint.size():robotRoot2EndJoint.size();

  for(i=1; i< minChainLength; i++)
    {
      if ((robotRoot2StartJoint[i]==robotRoot2EndJoint[i]))
	lastCommonJointRank++;
    }

  for(i = robotRoot2StartJoint.size()-1; i>lastCommonJointRank; i--)
    outJoints.push_back(robotRoot2StartJoint[i]);
  if(lastCommonJointRank==0)
      outJoints.push_back(robotRoot2EndJoint[0]);
  for(i=lastCommonJointRank+1; i< robotRoot2EndJoint.size(); i++)
    outJoints.push_back(robotRoot2EndJoint[i]);

  return outJoints;
}





void DynMultiBodyPrivate::ComputeNumberOfJoints()
{
  unsigned int r=0;
  ODEBUG("JointVector :" << m_JointVector.size());
  for(unsigned int i=0;i<m_JointVector.size();i++)
    {
      ODEBUG("JointPrivate " << i << " : "
	     << m_JointVector[i]->numberDof() <<  " "
	     << ((JointPrivate *)m_JointVector[i])->getIDinActuated() << " "
	     << ((JointPrivate *)m_JointVector[i])->getName());
      r += m_JointVector[i]->numberDof();
    }

  if (r!=m_NbDofs)
    m_NbDofs=r;
  ODEBUG("m_NbDofs: " << m_NbDofs);
  // Resize the jacobian of the CoM.
  MAL_MATRIX_RESIZE(m_JacobianOfTheCoM,3,m_NbDofs);
  MAL_MATRIX_RESIZE(m_InertiaMatrix,m_NbDofs,m_NbDofs);
  MAL_MATRIX_RESIZE(m_attCalcJointJacobian,6,m_NbDofs);
}

void DynMultiBodyPrivate::ReadSpecificities(string aFileName)
{
  FILE *fp;
  fp = fopen(aFileName.c_str(),"r");

  if (fp==0)
    {
      cerr << "Unable to read " << aFileName << endl;
      return;
    }

  if (look_for(fp,"LinkJointNameAndRank"))
    {
      NameAndRank_t aNameAndRank;

      if (look_for(fp,"Nb"))
        {
	  int r = fscanf(fp,"%d", &m_LinksBetweenJointNamesAndRankNb);
	  if (r<0)
	    perror("fscanf");
	  ODEBUG("Links: " << m_LinksBetweenJointNamesAndRankNb);
        }
      for(int j=0;j<m_LinksBetweenJointNamesAndRankNb;j++)
        {
	  if(look_for(fp,"Link"))
            {
	      char Buffer[128];
	      memset(Buffer,0,128);
	      int r = fscanf(fp,"%s",Buffer);
	      if (r<0)
		perror("fscanf");

	      strcpy(aNameAndRank.LinkName,Buffer);
	      r= fscanf(fp,"%u", &aNameAndRank.RankInConfiguration);
	      if (r<0)
		perror("fscanf");

	      look_for(fp,"</Link>");
	      m_LinksBetweenJointNamesAndRank.insert(m_LinksBetweenJointNamesAndRank.end(),
						     aNameAndRank);
            }

	  ODEBUG( aNameAndRank.LinkName << " " << aNameAndRank.RankInConfiguration);
        }
      //     ODEBUG((char *)((JointPrivate *)m_JointVector[0])->getName().c_str()
      // << " "  << m_JointVector[0]->rankInConfiguration());
    }
  fclose(fp);
}



void DynMultiBodyPrivate::BuildStateVectorToJointAndDOFs()
{
  m_StateVectorToJoint.resize(m_NbDofs);
  int lindex=0,StateVectorIndexDefault=0;
  for(unsigned int i=0;i<m_JointVector.size();i++)
    {
      lindex = JointRankFromName((JointPrivate *)m_JointVector[i]);
      if (lindex==-1)
	lindex = StateVectorIndexDefault;

      for(unsigned int j=0;j<m_JointVector[i]->numberDof();j++)
        {
	  m_StateVectorToJoint[lindex++]=i;
	  StateVectorIndexDefault++;
        }
    }


  m_StateVectorToDOFs.clear();
  m_StateVectorToDOFs.resize(m_NbDofs);
  lindex=0;
  StateVectorIndexDefault=0;
  for(unsigned int i=0;i<m_JointVector.size();i++)
    {
      lindex = JointRankFromName((JointPrivate *)m_JointVector[i]);
      if (lindex==-1)
	lindex = StateVectorIndexDefault;

      ODEBUG(((JointPrivate *)m_JointVector[i])->getName() << " " << lindex);
      for(unsigned int j=0;j<m_JointVector[i]->numberDof();j++)
        {
	  m_StateVectorToDOFs[lindex++]=j;
	  StateVectorIndexDefault++;
        }
    }

  m_ActuatedIDToConfiguration.resize(m_NbOfVRMLIDs+1);
  for(unsigned int i=0;i<m_StateVectorToJoint.size();)
    {
      int r;
      JointPrivate * aJoint = (JointPrivate *)m_JointVector[m_StateVectorToJoint[i]];

      // ASSUMPTION: The joint in the VRML have only one degree of freedom.
      if ((r=aJoint->getIDinActuated())!=-1)
        {
	  m_ActuatedIDToConfiguration[r] = i;
        }

      aJoint->stateVectorPosition((unsigned int)i);
      i+= aJoint->numberDof();
    }

  //added by oussama 30.03.2007
  //build m_ConfigurationToJoints (supposes the robot structure remains the same)
  for (unsigned int i=0;i<m_NbDofs;i++)
    m_ConfigurationToJoints.push_back(JointFromRank(i));
}

void DynMultiBodyPrivate::UpdateTheSizeOfJointsJacobian()
{
  for(unsigned int i=0;i<m_JointVector.size();i++)
    {
      ((JointPrivate *)m_JointVector[i])->resizeJacobianJointWrtConfig(m_NbDofs);
    }
}


// Get the number of degrees of freedom of the robot
unsigned int DynMultiBodyPrivate::numberDof() const
{
  return m_NbDofs;
}

/**
   \name Configuration, velocity and acceleration
*/

/**
   \brief Set the current configuration of the robot.


   \param inConfig the configuration vector \f${\bf q}\f$.

   \return true if success, false if failure (the dimension of the
   input vector does not fit the number of degrees of freedom of the
   robot).
*/
bool DynMultiBodyPrivate::currentConfiguration(const MAL_VECTOR_TYPE(double)& inConfig)
{
  if (MAL_VECTOR_SIZE(inConfig)!=
      MAL_VECTOR_SIZE(m_Configuration))
    return false;

  // Copy the configuration
  m_Configuration =  inConfig;

  ODEBUG("Went through here");
  int lindex=0;
  for (unsigned int i=0;i<m_ConfigurationToJoints.size();i++)
    {
      lindex = m_ConfigurationToJoints[i]->rankInConfiguration();

      // Update the pose of the joint when it is a free joint
      if (m_ConfigurationToJoints[i]->numberDof()==6)
        {
	  MAL_VECTOR_DIM(a6DPose,double,6);
	  for(int j=0;j<6;j++)
	    a6DPose(j) = inConfig[lindex++];

	  m_ConfigurationToJoints[i]->UpdatePoseFrom6DOFsVector(a6DPose);
        }
      else if (m_ConfigurationToJoints[i]->numberDof()==0)
        {
	  // do nothing
        }
      // Update only the quantity when it is a revolute or a prismatic joint.
      else
        {
	  int lIDinActuated = m_ConfigurationToJoints[i]->getIDinActuated();
	  if (lIDinActuated!=-1)
            {
	      m_ConfigurationToJoints[i]->quantity(inConfig[lindex]);
	      m_listOfBodies[ConvertIDInActuatedToBodyID[lIDinActuated]]->q = inConfig[lindex];
            }
	  else
            {
	      m_ConfigurationToJoints[i]->quantity(inConfig[lindex]);
	      m_ConfigurationToJoints[lindex]->linkedDBody()->q = inConfig[lindex];
            }

	  lindex++;
        }

    }

  if (0)
    {
      for(unsigned int i=0;i<m_listOfBodies.size();i++)
        {
	  cout << m_listOfBodies[i]->q * 180/M_PI << endl;
        }
    }
  return true;
}

/**
   \brief Get the current configuration of the robot.

   \return the configuration vector \f${\bf q}\f$.
*/
const MAL_VECTOR_TYPE(double)& DynMultiBodyPrivate::currentConfiguration() const
{
  return m_Configuration;
}

/**
   \brief Set the current velocity of the robot.

   \param inVelocity the velocity vector \f${\bf \dot{q}}\f$.

   \return true if success, false if failure (the dimension of the
   input vector does not fit the number of degrees of freedom of the
   robot).
*/
bool DynMultiBodyPrivate::currentVelocity(const MAL_VECTOR_TYPE(double)& inVelocity)
{
  if (MAL_VECTOR_SIZE(inVelocity)!=
      MAL_VECTOR_SIZE(m_Velocity))
    MAL_VECTOR_RESIZE(m_Velocity,MAL_VECTOR_SIZE(inVelocity));

  // Copy the configuration
  for(unsigned int i=0;i<MAL_VECTOR_SIZE(inVelocity);i++)
    {
      // ODEBUG("Velocity : " << i << " " << inVelocity(i) );
      m_Velocity(i)= inVelocity(i);
    }

  int lindex=0;
  for (unsigned int i=0;i<m_JointVector.size();i++)
    {

      lindex = m_JointVector[i]->rankInConfiguration();

      // Update the pose of the joint when it is a free joint
      if (m_JointVector[i]->numberDof()==6)
        {
	  MAL_S3_VECTOR_TYPE(double) alinearVelocity;
	  MAL_S3_VECTOR_TYPE(double) anAngularVelocity;
	  for(int j=0;j<3;j++)
            {
	      alinearVelocity(j) = inVelocity[lindex];
	      anAngularVelocity(j) = inVelocity[lindex+3];
	      lindex++;
            }

	  ((JointPrivate *)m_JointVector[i])->UpdateVelocityFrom2x3DOFsVector(alinearVelocity,
								       anAngularVelocity);
	  lindex+=3;
        }
      // Update only the quantity when it is a revolute or a prismatic joint.
      else
        {
	  int lIDinActuated = ((JointPrivate *)m_JointVector[i])->getIDinActuated();
	  if (lIDinActuated>=0)
            {
	      m_listOfBodies[ConvertIDInActuatedToBodyID[lIDinActuated]]->dq = inVelocity[lindex];
	      // Update dq.
	      /* ODEBUG(" Id (Vs) :" << lindex
		 << " Id (JV) : " << i
		 << " Id (VRML): " << lIDinActuated
		 << " Id (Body): " << ConvertIDInActuatedToBodyID[lIDinActuated]
		 << " dq: " << m_listOfBodies[ConvertIDInActuatedToBodyID[lIDinActuated]]->dq ); */
            }
	  else
	    m_ConfigurationToJoints[lindex]->linkedDBody()->dq = inVelocity[lindex];

	  lindex++;
        }

    }

  return true;

}

/**
   \brief Get the current velocity of the robot.

   \return the velocity vector \f${\bf \dot{q}}\f$.
*/
const MAL_VECTOR_TYPE(double)& DynMultiBodyPrivate::currentVelocity() const
{
  return m_Velocity;

}
/**
   \brief Set the current acceleration of the robot.

   \param inAcceleration the acceleration vector \f${\bf \ddot{q}}\f$.

   \return true if success, false if failure (the dimension of the
   input vector does not fit the number of degrees of freedom of the
   robot).
*/
bool DynMultiBodyPrivate::currentAcceleration(const MAL_VECTOR_TYPE(double)& inAcceleration)
{
  if (MAL_VECTOR_SIZE(inAcceleration)!=
      MAL_VECTOR_SIZE(m_Acceleration))
    MAL_VECTOR_RESIZE(m_Acceleration,MAL_VECTOR_SIZE(inAcceleration));

  // Copy the configuration
  for(unsigned int i=0;i<MAL_VECTOR_SIZE(inAcceleration);i++)
    {
      // ODEBUG("Velocity : " << i << " " << inVelocity(i) );
      m_Acceleration(i)= inAcceleration(i);
    }

  int lindex=0;
  for (unsigned int i=0;i<m_JointVector.size();i++)
    {

      lindex = m_JointVector[i]->rankInConfiguration();

      // Update the pose of the joint when it is a free joint
      if (m_JointVector[i]->numberDof()==6)
        {
	  MAL_S3_VECTOR_TYPE(double) alinearAcceleration;
	  MAL_S3_VECTOR_TYPE(double) anAngularAcceleration;
	  for(int j=0;j<3;j++)
            {
	      alinearAcceleration(j) = inAcceleration[lindex];
	      anAngularAcceleration(j) = inAcceleration[lindex+3];
	      lindex++;
            }

	  //((JointPrivate *)m_JointVector[i])->UpdateVelocityFrom2x3DOFsVector(alinearVelocity,
	  //								       anAngularVelocity);
	  lindex+=3;
        }
      // Update only the quantity when it is a revolute or a prismatic joint.
      else
        {
	  int lIDinActuated = ((JointPrivate *)m_JointVector[i])->getIDinActuated();
	  if (lIDinActuated>=0)
            {
	      m_listOfBodies[ConvertIDInActuatedToBodyID[lIDinActuated]]->ddq = inAcceleration[lindex];
	      // Update dq.
	      /* ODEBUG(" Id (Vs) :" << lindex
		 << " Id (JV) : " << i
		 << " Id (VRML): " << lIDinActuated
		 << " Id (Body): " << ConvertIDInActuatedToBodyID[lIDinActuated]
		 << " dq: " << m_listOfBodies[ConvertIDInActuatedToBodyID[lIDinActuated]]->dq ); */
            }
	  else
	    m_ConfigurationToJoints[lindex]->linkedDBody()->ddq = inAcceleration[lindex];

	  lindex++;
        }

    }

  return true;
}

/**
   \brief Get the current acceleration of the robot.

   \return the acceleration vector \f${\bf \ddot{q}}\f$.
*/
const MAL_VECTOR_TYPE(double)& DynMultiBodyPrivate::currentAcceleration() const
{
  return m_Acceleration;
}


/* Jacobian functions */


double DynMultiBodyPrivate::upperBoundDof(unsigned int inRankInConfiguration)
{
  //     if (inRankInConfiguration == m_RootOfTheJointsTree->rankInConfiguration())
  //         return 0;

  return m_ConfigurationToJoints[inRankInConfiguration]->upperBound(0);
  //WARNING: this code does not work if the joints have more than a single degree of freedom
}

double DynMultiBodyPrivate::lowerBoundDof(unsigned int inRankInConfiguration)
{
  //     if (inRankInConfiguration == m_RootOfTheJointsTree->rankInConfiguration())
  //         return 0;

  return m_ConfigurationToJoints[inRankInConfiguration]->lowerBound(0);
  //WARNING: this code does not work if the joints have more than a single degree of freedom
}

double DynMultiBodyPrivate::upperBoundDof(unsigned int inRankInConfiguration,
					  const vectorN& )
{
  return upperBoundDof(inRankInConfiguration);
}

double DynMultiBodyPrivate::lowerBoundDof(unsigned int inRankInConfiguration,
					  const vectorN& )
{
  return lowerBoundDof(inRankInConfiguration);
}
const matrixNxP & DynMultiBodyPrivate::currentTorques() const
{
  return m_Torques;
}

const matrixNxP & DynMultiBodyPrivate::currentForces() const
{
  return m_Forces;
}

const vectorN & DynMultiBodyPrivate::currentJointTorques() const
{
  return m_JointTorques;
}
