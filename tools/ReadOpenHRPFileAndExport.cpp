/*
 * Copyright 2010, 
 *
 * Francois Keith,
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

/* @doc \file Object to generate a file following AMELIF format.*/
#include <string>
#include <iostream>
#include <fstream>

#include <jrl/dynamics/dynamicsfactory.hh>
#include "GenerateRobotForAMELIF.h"

using namespace std;
using namespace dynamicsJRLJapan;

void ExportToAMELIF(CjrlHumanoidDynamicRobot *aHDR,
		    std::vector<BodyGeometricalData> &aVectorOfURLs)
{
  dynamicsJRLJapan::GenerateRobotForAMELIF aGenerateRobotForAMELIF;

  string aRobotNormalized("RobotNormalized");
  aGenerateRobotForAMELIF.SetAccessToData(aVectorOfURLs);
  aGenerateRobotForAMELIF.GenerateRobot(aRobotNormalized,aHDR);

}


int main(int argc, char *argv[])
{
  string aSpecificitiesFileName;
  string aPath;
  string aName;
  string aMapFromJointToRank;

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
					 aVectorOfURLs, true);

  aHDR->computeForwardKinematics();

  ExportToAMELIF(aHDR,aVectorOfURLs);
}
