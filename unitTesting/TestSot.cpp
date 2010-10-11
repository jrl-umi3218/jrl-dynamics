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



int main(int argc, char *argv[])
{
  string aSpecificitiesFileName;
  string aPath;
  string aName;
  string aMapFromJointToRank;

  ofstream tcout("output.txt",ofstream::out);

  if (argc!=5)
	{
		std::cout << "Please explicitely specify the config files." << std::endl;
		exit(0);
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

  int NbOfDofs = aHDR->numberDof();
  if(NbOfDofs==0)
    {
      cerr << "Empty Robot..."<< endl;
      return -1;
    }

  MAL_VECTOR_DIM(aCurrentConf,double,NbOfDofs);
  aCurrentConf[0]= 0;    
  aCurrentConf[1]= 0;    
  aCurrentConf[2]= 0;
  aCurrentConf[3]= 0;
  aCurrentConf[4]= 0;
  aCurrentConf[5]= 0;
  aCurrentConf[6]= 0; 
  aCurrentConf[7]= 0;    
  aCurrentConf[8]= 0;    
  aCurrentConf[9]= 0;    
  aCurrentConf[10]= 0;    
  aCurrentConf[11]= 0;    

  MAL_VECTOR_DIM(aCurrentVel,double,NbOfDofs); 
  aCurrentVel[0] = 0.0;
  aCurrentVel[1] = 0.0;
  aCurrentVel[2] = 100.0;
  aCurrentVel[3] = 1.0;
  aCurrentVel[4] = 0.0;
  aCurrentVel[5] = 0.0;
  aCurrentVel[6] = 1.0;
  aCurrentVel[7] = 0.0;
  aCurrentVel[8] = 0.0;
  aCurrentVel[9] = 0.0;
  aCurrentVel[10] = 0.0;
  aCurrentVel[11] = 0.0;

  MAL_VECTOR_DIM(aCurrentAcc,double,NbOfDofs);
  aCurrentAcc[0] = 0.0;
  aCurrentAcc[1] = 0.0;
  aCurrentAcc[2] = 0.0;
  aCurrentAcc[3] = 0.0;
  aCurrentAcc[4] = 0.0;
  aCurrentAcc[5] = 0.0;
  aCurrentAcc[6] = 0.0;
  aCurrentAcc[7] = 0.0;
  aCurrentAcc[8] = 0.0;
  aCurrentAcc[9] = 0.0;
  aCurrentAcc[10] = 0.0;
  aCurrentAcc[11] = 0.0;

  aHDR->currentConfiguration(aCurrentConf);
  aHDR->currentVelocity(aCurrentVel);
  aHDR->currentAcceleration(aCurrentAcc);

  std::string prop,arg;
  
	//prop="TimeStep";  arg="0.005"; aHDR->setProperty( prop,arg ); // maybe not set  in sot
	//prop="ComputeAcceleration";  arg="true"; aHDR->setProperty( prop,arg ); // maybe false in sot
	//prop="ComputeBackwardDynamics"; arg="true";  aHDR->setProperty( prop,arg );
	//prop="ComputeZMP";  arg="true"; aHDR->setProperty( prop,arg ); // false in sot
	//prop="ComputeAccelerationCoM";  arg="true"; aHDR->setProperty( prop,arg ); // false in sot
	//prop="ComputeCoM";  arg="true"; aHDR->setProperty( prop,arg );
	//prop="ComputeVelocity";  arg="true"; aHDR->setProperty( prop,arg );
	//prop="ComputeMomentum";  arg="true"; aHDR->setProperty( prop,arg );


	// Properties SOT
	prop="ComputeAcceleration";  arg="true"; aHDR->setProperty( prop,arg ); // maybe false in sot -- take care to it
	prop="ComputeBackwardDynamics"; arg="true";  aHDR->setProperty( prop,arg );
	prop="ComputeZMP";  arg="false"; aHDR->setProperty( prop,arg ); // should be true?  --- no, ok like that
	prop="ComputeAccelerationCoM";  arg="true"; aHDR->setProperty( prop,arg ); // should be true ? --- YES in the normal code - could be false or true for the spatial code
	prop="ComputeCoM";  arg="true"; aHDR->setProperty( prop,arg ); // should be true in the normal code - could be false or true for the spatial code
	prop="ComputeVelocity";  arg="true"; aHDR->setProperty( prop,arg );
    prop="ComputeMomentum";  arg="false"; aHDR->setProperty( prop,arg );

	// Initialize the backward dynamics (2de order, should be processed 3 times before).
	for( int i=0;i<3;++i )
		aHDR->computeForwardKinematics();

  aHDR->computeForwardKinematics();
  const matrixNxP& Torques = aHDR->currentTorques();
  const matrixNxP& Forces = aHDR->currentForces();

  std::cout << "TorqueDrift = " << Torques << std::endl;
  std::cout << "ForceDrift = " << Forces << std::endl;

  matrixNxP InertiaMatrix;
  aHDR->computeInertiaMatrix();
  InertiaMatrix = aHDR->inertiaMatrix();

  aCurrentAcc[6] = 1.0;
  aHDR->currentAcceleration(aCurrentAcc);

  // Initialize the backward dynamics (2de order, should be processed 3 times before).
	for( int i=0;i<3;++i )
		aHDR->computeForwardKinematics();

  aHDR->computeForwardKinematics();

  const matrixNxP& Torques1 = aHDR->currentTorques();
  const matrixNxP& Forces1 = aHDR->currentForces();

  std::cout << "InertiaMatrix("
       << MAL_MATRIX_NB_ROWS(InertiaMatrix)<< "," 
	   << MAL_MATRIX_NB_COLS(InertiaMatrix)<< ")"<< std::endl;
  DisplayMatrix(InertiaMatrix,std::cout);

  for(unsigned int i=6;i<MAL_MATRIX_NB_ROWS(Torques);i++)
    {
      double torquefrominertia = 9.81 * InertiaMatrix(i,2);
	  std::cout << filterprecision(Torques1(i,0)) << " \t \t" << filterprecision(torquefrominertia) << " \t "  
		  << filterprecision(torquefrominertia)+Torques(i,0) << std::endl;
    }

  delete aHDR;
  return -1;
}
