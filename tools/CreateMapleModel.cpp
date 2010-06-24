/* @doc \file Object to generate a file following AMELIF format.

   Copyright (c) 2010
   @author Olivier Stasse
   JRL-Japan, CNRS/AIST
 
   All rights reserved.

   Please refers to file License.txt for details on the license.

*/
#include <string>
#include <iostream>
#include <fstream>

#include <dynamicsJRLJapan/dynamicsJRLJapanFactory.h>
#include "GenerateRobotForMaple.h"

using namespace std;
using namespace dynamicsJRLJapan;

void ExportToMapple(CjrlHumanoidDynamicRobot *aHDR,
		   std::vector<BodyGeometricalData> &aVectorOfURLs,
		   std::string &Path,
		   std::string &RobotFileName)
{
  dynamicsJRLJapan::Tools::GenerateRobotForMaple aGenerateRobotForMaple;

  string aRobotVrml="ROBOT";
  aGenerateRobotForMaple.GenerateRobot(aRobotVrml,aHDR);
}


void RobotSetPosition(CjrlHumanoidDynamicRobot *aHDR)
{
  unsigned int NbOfDofs = aHDR->numberDof();

  MAL_VECTOR_DIM(aCurrentConf,double,NbOfDofs);
  MAL_VECTOR_FILL(aCurrentConf,0.0);
  aHDR->currentConfiguration(aCurrentConf);  
  MAL_VECTOR_DIM(aCurrentVel,double,NbOfDofs);
  MAL_VECTOR_FILL(aCurrentVel,0.0);
  MAL_VECTOR_DIM(aCurrentAcc,double,NbOfDofs);
  MAL_VECTOR_FILL(aCurrentAcc,0.0);

  // This is mandatory for this implementation of computeForwardKinematics
  // to compute the derivative of the momentum.
  {
    string inProperty[4]={"TimeStep","ComputeAcceleration",
			  "ComputeBackwardDynamics", "ComputeZMP"};
    string inValue[4]={"0.005","false","false","true"};
    for(unsigned int i=0;i<4;i++)
      aHDR->setProperty(inProperty[i],inValue[i]);

  }
  aHDR->currentVelocity(aCurrentVel);
  aHDR->currentAcceleration(aCurrentAcc);
  aHDR->computeForwardKinematics();

}

int main(int argc, char *argv[])
{
  string aSpecificitiesFileName;
  string aPath;
  string aName;
  string aMapFromJointToRank;
  string aFileJointValue;

  ofstream tcout("output.txt",ofstream::out);

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
  cout << "RobotFileNAme :" << RobotFileName << endl;
  vector<BodyGeometricalData> aVectorOfURLs;
  dynamicsJRLJapan::parseOpenHRPVRMLFile(*aHDR,RobotFileName,
					 aMapFromJointToRank,
					 aSpecificitiesFileName,
					 aVectorOfURLs);

  RobotSetPosition(aHDR);
  
  ExportToMapple(aHDR,aVectorOfURLs,aPath, RobotFileName);

  delete aHDR;
}
