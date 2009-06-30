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
#include "DynamicMultiBody.h"
#include "robotDynamics/jrlBody.h"

#include "fileReader.h"

using namespace dynamicsJRLJapan;

DynamicMultiBody::DynamicMultiBody()
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
  MAL_S3_VECTOR(,double) anAxis;

  m_RootOfTheJointsTree = 0;

  m_Prev_P(0) =   m_Prev_P(1) =   m_Prev_P(2) = 0.0;
  m_Prev_L(0) =   m_Prev_L(1) =   m_Prev_L(2) = 0.0;

  m_NbOfVRMLIDs = -1;

  m_SynchronizationBetweenActuatedVectorAndJoints = false;

  RESETDEBUG4("DebugDataPL.dat");
  RESETDEBUG4("DebugDataZMP.dat");
  RESETDEBUG4("DebugDataDMB_ZMP.dat");
}

DynamicMultiBody::~DynamicMultiBody()
{
  for(unsigned int li=0; li<m_listOfBodies.size();li++)
    {
      Body *body = m_listOfBodies[li];
      std::vector<Body *>::iterator it;
      for (it = listeCorps.begin(); it != listeCorps.end(); it++)
        {
	  if (*it == body)
            {
	      listeCorps.erase(it);
	      break;
            }
        }
      delete m_listOfBodies[li];
    }
}


bool DynamicMultiBody::initialize()
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


void DynamicMultiBody::parserVRML(string path,
                                  string nom,
                                  const char *option)
{
  m_listOfBodies.clear();
  MultiBody::parserVRML(path, nom, option);
  CreatesTreeStructure(option);
}

void DynamicMultiBody::parserVRML(string nom,
                                  const char *option)
{
  m_listOfBodies.clear();
  MultiBody::parserVRML(nom, option);
  CreatesTreeStructure(option);
}


double DynamicMultiBody::Getq(int JointID) const
{
  if ((JointID>=0) &&
      ((unsigned int)JointID<m_listOfBodies.size()))
    return m_listOfBodies[ConvertIDInActuatedToBodyID[JointID]]->q;
  return 0.0;
}

void DynamicMultiBody::Setq(int JointID,double q)
{
  if ((JointID>=0) &&
      ((unsigned int)JointID<m_listOfBodies.size()))
    {
      m_listOfBodies[ConvertIDInActuatedToBodyID[JointID]]->q= q;
      ((Joint *)m_listOfBodies[ConvertIDInActuatedToBodyID[JointID]]->joint())->quantity(q);
    }
}

void DynamicMultiBody::Setdq(int JointID, double dq)
{
  if ((JointID>=0) &&
      ((unsigned int)JointID<m_listOfBodies.size()))
    {
      m_listOfBodies[ConvertIDInActuatedToBodyID[JointID]]->dq= dq;
    }
}

void DynamicMultiBody::Setv(int JointID, MAL_S3_VECTOR(,double) v0)
{
  if ((JointID>=0) &&
      ((unsigned int)JointID<m_listOfBodies.size()))
    m_listOfBodies[ConvertIDInActuatedToBodyID[JointID]]->v0 = v0;
}

MAL_S3_VECTOR(,double) DynamicMultiBody::Getv(int JointID)
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

MAL_S3_VECTOR(,double) DynamicMultiBody::GetvBody(int BodyID)
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

MAL_S3_VECTOR(,double) DynamicMultiBody::GetwBody(int BodyID)
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

MAL_S3_VECTOR(,double) DynamicMultiBody::Getw(int JointID)
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

void DynamicMultiBody::Setw(int JointID, MAL_S3_VECTOR(,double) w)
{
  if ((JointID>=0) &&
      ((unsigned int)JointID<m_listOfBodies.size()))
    m_listOfBodies[ConvertIDInActuatedToBodyID[JointID]]->w = w;
}

MAL_S3_VECTOR(,double) DynamicMultiBody::Getp(int JointID)
{
  MAL_S3_VECTOR(empty,double);
  if ((JointID>=0) &&
      ((unsigned int)JointID<m_listOfBodies.size()))
    return m_listOfBodies[ConvertIDInActuatedToBodyID[JointID]]->p;
  return empty;
}

void DynamicMultiBody::Setp(int JointID, MAL_S3_VECTOR(,double) apos)
{
  if ((JointID>=0) &&
      ((unsigned int)JointID<m_listOfBodies.size()))
    m_listOfBodies[ConvertIDInActuatedToBodyID[JointID]]->p = apos;
}



void DynamicMultiBody::CalculateZMP(double &px, double &py,
                                    MAL_S3_VECTOR(,double) dP,
                                    MAL_S3_VECTOR(,double) dL,
                                    double zmpz)
{
  double g= 9.80665;

  px = ( g * positionCoMPondere[0]*masse +
	 zmpz * dP[0] - dL[1])/(masse * g + dP[2]);
  py = ( g * positionCoMPondere[1]*masse +
	 zmpz * dP[1] + dL[0])/(masse * g + dP[2]);

  ODEBUG(" CalculateZMP : Masse :"<< masse << " g:" << g  << " "
	 << dP << " " << dL << " " << positionCoMPondere );


}

string DynamicMultiBody::GetName(int JointID)
{
  string empty;
  if ((JointID>=0) &&
      ((unsigned int)JointID<m_listOfBodies.size()))
    return m_listOfBodies[ConvertIDInActuatedToBodyID[JointID]]->getName();
  return empty;

}



MAL_S3_VECTOR(,double) DynamicMultiBody::GetP(int JointID)
{
  MAL_S3_VECTOR(empty,double);
  if ((JointID>=0) &&
      ((unsigned int)JointID<m_listOfBodies.size()))
    return m_listOfBodies[ConvertIDInActuatedToBodyID[JointID]]->P;
  return empty;
}

double DynamicMultiBody::Getdq(int JointID) const
{
  double empty=0.0;
  if ((JointID>=0) &&
      ((unsigned int)JointID<m_listOfBodies.size()))
    return m_listOfBodies[ConvertIDInActuatedToBodyID[JointID]]->dq;
  return empty;
}



void DynamicMultiBody::SetRBody(int BodyID, MAL_S3x3_MATRIX(,double) R)
{
  if ((BodyID>=0) &&
      ((unsigned int)BodyID<m_listOfBodies.size()))
    m_listOfBodies[BodyID]->R = R;
}

/***********************************************/
/* Implementation of the generic JRL interface */
/***********************************************/

// Set the root joint of the robot.
void DynamicMultiBody::rootJoint(CjrlJoint& inJoint)
{
  m_RootOfTheJointsTree = & (Joint &)inJoint;

}

// Get the robot joint of the robot.
CjrlJoint* DynamicMultiBody::rootJoint() const
{
  return m_RootOfTheJointsTree;
}

// Get a vector containning all the joints
std::vector<CjrlJoint*> DynamicMultiBody::jointVector()
{
  return m_JointVector;
}

std::vector<CjrlJoint*> DynamicMultiBody::jointsBetween(const CjrlJoint& inStartJoint, 
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

  for(i=lastCommonJointRank+1; i< robotRoot2EndJoint.size(); i++)
    outJoints.push_back(robotRoot2EndJoint[i]);

  return outJoints;
}





void DynamicMultiBody::ComputeNumberOfJoints()
{
  unsigned int r=0;
  ODEBUG("JointVector :" << m_JointVector.size());
  for(unsigned int i=0;i<m_JointVector.size();i++)
    {
      ODEBUG("Joint " << i << " : "
	     << m_JointVector[i]->numberDof() <<  " "
	     << ((Joint *)m_JointVector[i])->getIDinActuated() << " "
	     << ((Joint *)m_JointVector[i])->getName());
      r += m_JointVector[i]->numberDof();
    }

  if (r!=m_NbDofs)
    m_NbDofs=r;

  // Resize the jacobian of the CoM.
  MAL_MATRIX_RESIZE(m_JacobianOfTheCoM,3,m_NbDofs);
  MAL_MATRIX_RESIZE(m_InertiaMatrix,m_NbDofs,m_NbDofs);
  MAL_MATRIX_RESIZE(m_attCalcJointJacobian,6,m_NbDofs);
}

void DynamicMultiBody::ReadSpecificities(string aFileName)
{
  FILE *fp;
  fp = fopen((char *)aFileName.c_str(),"r");

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
	  fscanf(fp,"%d", &m_LinksBetweenJointNamesAndRankNb);
	  ODEBUG("Links: " << m_LinksBetweenJointNamesAndRankNb);
        }
      for(int j=0;j<m_LinksBetweenJointNamesAndRankNb;j++)
        {
	  if(look_for(fp,"Link"))
            {
	      char Buffer[128];
	      memset(Buffer,0,128);
	      fscanf(fp,"%s",Buffer);

	      strcpy(aNameAndRank.LinkName,Buffer);
	      fscanf(fp,"%d", &aNameAndRank.RankInConfiguration);

	      look_for(fp,"</Link>");
	      m_LinksBetweenJointNamesAndRank.insert(m_LinksBetweenJointNamesAndRank.end(),
						     aNameAndRank);
            }

	  ODEBUG( aNameAndRank.LinkName << " " << aNameAndRank.RankInConfiguration);
        }
      //     ODEBUG((char *)((Joint *)m_JointVector[0])->getName().c_str()
      // << " "  << m_JointVector[0]->rankInConfiguration());
    }
  fclose(fp);
}



void DynamicMultiBody::BuildStateVectorToJointAndDOFs()
{
  m_StateVectorToJoint.resize(m_NbDofs);
  int lindex=0,StateVectorIndexDefault=0;
  for(unsigned int i=0;i<m_JointVector.size();i++)
    {
      lindex = JointRankFromName((Joint *)m_JointVector[i]);
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
      lindex = JointRankFromName((Joint *)m_JointVector[i]);
      if (lindex==-1)
	lindex = StateVectorIndexDefault;

      ODEBUG(((Joint *)m_JointVector[i])->getName() << " " << lindex);
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
      Joint * aJoint = (Joint *)m_JointVector[m_StateVectorToJoint[i]];

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

void DynamicMultiBody::UpdateTheSizeOfJointsJacobian()
{
  for(unsigned int i=0;i<m_JointVector.size();i++)
    {
      ((Joint *)m_JointVector[i])->resizeJacobianJointWrtConfig(m_NbDofs);
    }
}


// Get the number of degrees of freedom of the robot
unsigned int DynamicMultiBody::numberDof() const
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
bool DynamicMultiBody::currentConfiguration(const MAL_VECTOR(,double)& inConfig)
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
	      m_ConfigurationToJoints[i]->quantity(0.0);
	      m_listOfBodies[ConvertIDInActuatedToBodyID[lIDinActuated]]->q = 0.0;
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
const MAL_VECTOR(,double)& DynamicMultiBody::currentConfiguration() const
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
bool DynamicMultiBody::currentVelocity(const MAL_VECTOR(,double)& inVelocity)
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
	  MAL_S3_VECTOR(,double) alinearVelocity;
	  MAL_S3_VECTOR(,double) anAngularVelocity;
	  for(int j=0;j<3;j++)
            {
	      alinearVelocity(j) = inVelocity[lindex];
	      anAngularVelocity(j) = inVelocity[lindex+3];
	      lindex++;
            }

	  ((Joint *)m_JointVector[i])->UpdateVelocityFrom2x3DOFsVector(alinearVelocity,
								       anAngularVelocity);
	  lindex+=3;
        }
      // Update only the quantity when it is a revolute or a prismatic joint.
      else
        {
	  int lIDinActuated = ((Joint *)m_JointVector[i])->getIDinActuated();
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
	    m_listOfBodies[ConvertIDInActuatedToBodyID[lIDinActuated]]->dq = 0.0;

	  lindex++;
        }

    }

  return true;

}

/**
   \brief Get the current velocity of the robot.
   
   \return the velocity vector \f${\bf \dot{q}}\f$.
*/
const MAL_VECTOR(,double)& DynamicMultiBody::currentVelocity() const
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
bool DynamicMultiBody::currentAcceleration(const MAL_VECTOR(,double)& inAcceleration)
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
	  MAL_S3_VECTOR(,double) alinearAcceleration;
	  MAL_S3_VECTOR(,double) anAngularAcceleration;
	  for(int j=0;j<3;j++)
            {
	      alinearAcceleration(j) = inAcceleration[lindex];
	      anAngularAcceleration(j) = inAcceleration[lindex+3];
	      lindex++;
            }

	  //((Joint *)m_JointVector[i])->UpdateVelocityFrom2x3DOFsVector(alinearVelocity,
	  //								       anAngularVelocity);
	  lindex+=3;
        }
      // Update only the quantity when it is a revolute or a prismatic joint.
      else
        {
	  int lIDinActuated = ((Joint *)m_JointVector[i])->getIDinActuated();
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
	    m_listOfBodies[ConvertIDInActuatedToBodyID[lIDinActuated]]->ddq = 0.0;

	  lindex++;
        }

    }

  return true;
}

/**
   \brief Get the current acceleration of the robot.
   
   \return the acceleration vector \f${\bf \ddot{q}}\f$.
*/
const MAL_VECTOR(,double)& DynamicMultiBody::currentAcceleration() const
{
  return m_Acceleration;
}


/* Jacobian functions */


double DynamicMultiBody::upperBoundDof(unsigned int inRankInConfiguration)
{
  //     if (inRankInConfiguration == m_RootOfTheJointsTree->rankInConfiguration())
  //         return 0;

  return m_ConfigurationToJoints[inRankInConfiguration]->upperBound(0);
  //WARNING: this code does not work if the joints have more than a single degree of freedom
}

double DynamicMultiBody::lowerBoundDof(unsigned int inRankInConfiguration)
{
  //     if (inRankInConfiguration == m_RootOfTheJointsTree->rankInConfiguration())
  //         return 0;

  return m_ConfigurationToJoints[inRankInConfiguration]->lowerBound(0);
  //WARNING: this code does not work if the joints have more than a single degree of freedom
}

double DynamicMultiBody::upperBoundDof(unsigned int inRankInConfiguration,
                                       const vectorN& inConfig)
{
  return upperBoundDof(inRankInConfiguration);
}

double DynamicMultiBody::lowerBoundDof(unsigned int inRankInConfiguration,
                                       const vectorN& inConfig)
{
  return lowerBoundDof(inRankInConfiguration);
}
const matrixNxP & DynamicMultiBody::currentTorques() const
{
  return m_Torques;
}

const matrixNxP & DynamicMultiBody::currentForces() const
{
  return m_Forces;
}

#include "DynamicMultiBodyArticularJacobian.cpp"
#include "DynamicMultiBodyTree.cpp"
#include "DynamicMultiBodyNewtonEuler.cpp"
#include "DynamicMultiBodyNewtonEulerBackwardDynamics.cpp"
#include "DynamicMultiBodyJointRank.cpp"
#include "DynamicMultiBodyCenterOfMass.cpp"
#include "DynamicMultiBodyProperties.cpp"
#include "DynamicMultiBodyAngularMomentum.cpp"
#include "DynamicMultiBodyInertiaMatrix.cpp"
#include "DynamicMultiBodyActuated.cpp"
