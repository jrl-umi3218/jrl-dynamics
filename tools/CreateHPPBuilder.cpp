/*
 * Copyright 2010, 
 *
 * Olivier Stasse,
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
/* @doc \file Object to generate a class deriving from abstractDynamicsJRLJapan and hpp.
*/
#include <string>
#include <iostream>
#include <fstream>

#include <jrl/dynamics/dynamicsfactory.hh>
#include "GenerateRobotForHppBuilder.h"

using namespace std;
using namespace dynamicsJRLJapan;

void ExportToHPPBuilder(CjrlHumanoidDynamicRobot *aHDR,
		   std::vector<BodyGeometricalData> &aVectorOfURLs,
		   std::string &Path,
		   std::string &)
{
  dynamicsJRLJapan::Tools::GenerateRobotForHppBuilder aGenerateRobotForHppBuilder;

  string aRobotVrml="ROBOT";
  aGenerateRobotForHppBuilder.SetAccessToData(aVectorOfURLs);
  aGenerateRobotForHppBuilder.SetPathToModelFiles(Path);
  aGenerateRobotForHppBuilder.GenerateRobot(aRobotVrml,aHDR);
}


void RobotSetPosition(CjrlHumanoidDynamicRobot *aHDR,
		      std::string FileOfJointValues,
		      int Verbosity)
{
  unsigned int NbOfDofs = aHDR->numberDof();

  ifstream aif(FileOfJointValues.c_str(),
		ifstream::in);

  MAL_VECTOR_DIM(aCurrentConf,double,NbOfDofs);
  int lindex=0;
  for(unsigned int i=0;i<6;i++)
      aCurrentConf[lindex++] = 0.0;

  if (!aif.is_open())
    for(unsigned int i=6;i<NbOfDofs;i++)
      aCurrentConf[lindex++] = 0.0;
  else
    {
      for(unsigned int i=6;i<NbOfDofs;i++)
	{
	  aif >> aCurrentConf[lindex] ;
	  aCurrentConf[lindex]= aCurrentConf[lindex] * M_PI/180.0; 
	  lindex++;
	}
      aif.close();
    }
  
  if (Verbosity>2)
    for(unsigned int i=0;i<NbOfDofs;i++)
      {
	cout << aCurrentConf[i] << endl;
      }
    
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

  if (argc!=6)
    {
      aPath="./";
      aName="sample.wrl";
      aSpecificitiesFileName = "sampleSpecificities.xml";
      aMapFromJointToRank = "sampleLinkJointRank.xml";
      aFileJointValue="position.articular";
    }
  else 
    {
      aSpecificitiesFileName = argv[3];
      aPath=argv[1];
      aName=argv[2];
      aMapFromJointToRank=argv[4];
      aFileJointValue = argv[5];
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
					 aVectorOfURLs,
					 true);

  RobotSetPosition(aHDR,aFileJointValue,0);
  
  ExportToHPPBuilder(aHDR,aVectorOfURLs,aPath, RobotFileName);

  delete aHDR;
}
