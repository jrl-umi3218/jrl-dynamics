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
/* @doc \file Object to generate a file following AMELIF format.
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
