/* @doc \file Object to generate a file following AMELIF format.

   Copyright (c) 2007-2010
   @author Olivier Stasse
   JRL-Japan, CNRS/AIST
 
   All rights reserved.

   Please refers to file License.txt for details on the license.

*/

#include <string>
#include <iostream>
#include <fstream>
#include <dynamicsJRLJapan/dynamicsJRLJapanFactory.h>
#include "CommonTools.h"

using namespace std;
using namespace dynamicsJRLJapan;


void DisplayDynamicRobotInformation(CjrlDynamicRobot *aDynamicRobot,
				    ostream &tcout)
{
  std::vector<CjrlJoint *> aVec = aDynamicRobot->jointVector();
  int r = aVec.size();
  tcout << "Number of joints :" << r << endl;
  
}

int main(int argc, char *argv[])
{
  string aSpecificitiesFileName;
  string aPath;
  string aName;
  string aMapFromJointToRank;


  if (argc!=5)
    {
      aPath="./";
      aName="sample.wrl";
      aSpecificitiesFileName = "sampleSpecificities.xml";
      aMapFromJointToRank = "sampleLinkJointRank.xml";
    }
  else 
    {
      aSpecificitiesFileName = argv[3];
      aPath=argv[1];
      aName=argv[2];
      aMapFromJointToRank=argv[4];
    }
  dynamicsJRLJapan::ObjectFactory aRobotDynamicsObjectConstructor;
  
  CjrlHumanoidDynamicRobot * aHDR = aRobotDynamicsObjectConstructor.createHumanoidDynamicRobot();
  

  if (aHDR==0)
    { 
      cerr<< "Dynamic cast on HDR failed " << endl;
      exit(-1);
    }
  string RobotFileName = aPath+aName;
  dynamicsJRLJapan::parseOpenHRPVRMLFile(*aHDR,RobotFileName,aMapFromJointToRank,aSpecificitiesFileName);
  
  // Display tree of the joints.
  CjrlJoint* rootJoint = aHDR->rootJoint();  

  int NbOfDofs = aHDR->numberDof();
  if (NbOfDofs==0)
    {
      cerr << "Empty Robot..."<< endl;
      return -1;
    }

  string inProperty[5]={"TimeStep","ComputeAcceleration",
			"ComputeBackwardDynamics", "ComputeZMP","ComputeAccelerationCoM"};
  string inValue[5]={"0.005","true","true","true","true"};
  for(unsigned int i=0;i<5;i++)
    aHDR->setProperty(inProperty[i],inValue[i]);


  MAL_VECTOR_DIM(aCurrentConf,double,NbOfDofs);
  MAL_VECTOR_DIM(aCurrentVel,double,NbOfDofs);
  MAL_VECTOR_DIM(aCurrentAcc,double,NbOfDofs);
  string empty("");

  // Case 1
  for(int i=0;i<NbOfDofs;i++)
    {
      aCurrentConf[i] = 0.0;
      aCurrentAcc[i] = 0.0; 
      aCurrentVel[i] = 0.0;
    }
  aCurrentConf[9] = 1.0;
//   aCurrentVel[3] = 1.0;
//   aCurrentVel[6] = 1.0;
aCurrentAcc[2] = 1.0;
  aHDR->currentConfiguration(aCurrentConf);
  aHDR->currentVelocity(aCurrentVel);
  aHDR->currentAcceleration(aCurrentAcc);

  aHDR->computeForwardKinematics();

  ofstream tcout("case1.txt",ofstream::out);

  RecursiveDisplayOfJoints(rootJoint,tcout,10);
  DisplayForces(aHDR,empty,tcout);
  DisplayTorques(aHDR,empty, tcout);
    
  tcout.close();


  // Case 2
  for(int i=0;i<NbOfDofs;i++)
    {
      aCurrentConf[i] = 0.0;
      aCurrentAcc[i] = 0.0; 
      aCurrentVel[i] = 0.0;
    }
  aCurrentConf[9] = 1.0;
  aHDR->currentConfiguration(aCurrentConf);
  aHDR->currentVelocity(aCurrentVel);
  aHDR->currentAcceleration(aCurrentAcc);

  aHDR->computeForwardKinematics();

  tcout.open("case2.txt",ofstream::out);

  RecursiveDisplayOfJoints(rootJoint,tcout,10);
  DisplayForces(aHDR,empty,tcout);
  DisplayTorques(aHDR,empty, tcout);
    
  tcout.close();


  delete aHDR;
  return -1;
}
