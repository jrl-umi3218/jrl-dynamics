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
  dynamicsJRLJapan::parseOpenHRPVRMLFile(*aHDR,RobotFileName,aMapFromJointToRank,aSpecificitiesFileName);
  
  // Display tree of the joints.
  CjrlJoint* rootJoint = aHDR->rootJoint();  

  int NbOfDofs = aHDR->numberDof();
  tcout << "NbOfDofs :" << NbOfDofs << std::endl;
  if (NbOfDofs==0)
    {
      cerr << "Empty Robot..."<< endl;
      return -1;
    }
  MAL_VECTOR_DIM(aCurrentConf,double,NbOfDofs);
  int lindex=0;
  for(int i=0;i<6;i++)
    aCurrentConf[lindex++] = 0.0;
  aCurrentConf[2] = 0.705;
  for(int i=0;i<(NbOfDofs-6 < 40 ? NbOfDofs-6 : 40) ;i++)
  aCurrentConf[lindex++] = 0.0;
 /* aCurrentConf[8]= -0.4538;    
  aCurrentConf[9]= 0.8727;
  aCurrentConf[10]= -0.4189;
  aCurrentConf[14]= -0.4538;       
  aCurrentConf[15]= 0.8727;
  aCurrentConf[16]= -0.4189;
  aCurrentConf[22]= 0.2618;   
  aCurrentConf[23]= -0.1745;
  aCurrentConf[25]= -0.5236;
  aCurrentConf[28]=  0.1745;    
  aCurrentConf[29]= 0.2618;
  aCurrentConf[30]= 0.1745;
  aCurrentConf[32]= -0.5236;
  aCurrentConf[35]= 0.1745; */
  tcout << "NbOfDofs:" << NbOfDofs << std::endl; 
  tcout << "Current Configuration :" << aCurrentConf << std::endl;
  aHDR->currentConfiguration(aCurrentConf);

  MAL_VECTOR_DIM(aCurrentVel,double,NbOfDofs); 
  lindex=0;
  for(int i=0;i<NbOfDofs;i++)
    aCurrentVel[lindex++] = 0.0;
	aCurrentVel[6]=1;
  MAL_S3_VECTOR(ZMPval,double);

 /* {
	unsigned int i=0;
    aCurrentConf[i++] = 	0; aCurrentConf[i++] = 0; aCurrentConf[i++] = 0; aCurrentConf[i++] = 0; aCurrentConf[i++] = 0; aCurrentConf[i++] = 0; aCurrentConf[i++] = 2.83203; aCurrentConf[i++] = -1.15485e-005; aCurrentConf[i++] = -0.453805; aCurrentConf[i++] = 0.872699; aCurrentConf[i++] = -0.4189; aCurrentConf[i++] = -4.43866e-007; aCurrentConf[i++] = 3.95636e-024; aCurrentConf[i++] = -6.41384e-019; aCurrentConf[i++] = -0.4538; aCurrentConf[i++] = 0.8727; aCurrentConf[i++] = -0.4189; aCurrentConf[i++] = -1.22267e-019; aCurrentConf[i++] = 9.08039e-025; aCurrentConf[i++] = -1.22882e-018; aCurrentConf[i++] = 2.48871e-026; aCurrentConf[i++] = 4.32195e-020; aCurrentConf[i++] = 0.2618; aCurrentConf[i++] = -0.1745; aCurrentConf[i++] = 2.227e-021; aCurrentConf[i++] = -0.5236; aCurrentConf[i++] = 8.41298e-021; aCurrentConf[i++] = -5.98639e-020; aCurrentConf[i++] = 0.1745; aCurrentConf[i++] = 0.2618; aCurrentConf[i++] = 0.1745; aCurrentConf[i++] = 5.32895e-019; aCurrentConf[i++] = -0.5236; aCurrentConf[i++] = 1.39771e-019; aCurrentConf[i++] = 5.62899e-021; aCurrentConf[i++] = 0.1745;
  }
  tcout << "Current (real) Configuration :" << aCurrentConf << std::endl;
  aHDR->currentConfiguration(aCurrentConf);*/


//dynamicDrift = [ 0.00106856, -3.65827, -3.24512, 3.39716, -0.0415928, -0.0752126, -2.19959e-016, 3.65805, -3.2449, 3.38531, -0.0423655, 0.0753855, 1.11009e-016, 4.43083, -2.0292e-020, -0.12657, 1.03213, -2.57379, -0.499568, -1.39763, -0.00344316, -0.302932, -0.00679369, 1.07342, 2.5637, 0.489739, -1.35744, -2.65727e-005, -0.262741, 0.00685521]

  aHDR->currentVelocity(aCurrentVel);
  //  aHDR->setComputeZMP(true);
  string inProperty="ComputeZMP"; string aValue="true";
  aHDR->setProperty(inProperty,aValue);
  aHDR->computeForwardKinematics();
  ZMPval = aHDR->zeroMomentumPoint();
  tcout << "First value of ZMP : " 
	<< filterprecision(ZMPval(0)) << " " 
	<< filterprecision(ZMPval(1)) << " " 
	<< filterprecision(ZMPval(2)) << endl;
  MAL_S3_VECTOR(poscom,double);
  poscom = aHDR->positionCenterOfMass();
  tcout << "Should be equal to the CoM: " 
	<< filterprecision(poscom(0)) << " "
	<< filterprecision(poscom(1)) << " "  
	<< filterprecision(poscom(2)) << endl;


  matrixNxP InertiaMatrix;
  aHDR->computeInertiaMatrix();
  InertiaMatrix = aHDR->inertiaMatrix();
 
  tcout << "InertiaMatrix("
       << MAL_MATRIX_NB_ROWS(InertiaMatrix)<< "," 
       << MAL_MATRIX_NB_COLS(InertiaMatrix)<< ")"<< endl;
    
  DisplayMatrix(InertiaMatrix,tcout);
  
  ofstream aof;
  aof.open("InertiaMatrix.dat");
  for(unsigned int i=0;i<MAL_MATRIX_NB_ROWS(InertiaMatrix);i++)
    {
      for(unsigned int j=0;j<MAL_MATRIX_NB_COLS(InertiaMatrix);j++)
       {
         aof << InertiaMatrix(i,j) << " ";
       }
      aof << endl;
    }
  aof.close();

  std::vector<CjrlJoint *> aVec = aHDR->jointVector();
  
  // Get the Jacobian of the right ankle.
  CjrlJoint  * aJoint = aHDR->rightAnkle();
  aJoint->computeJacobianJointWrtConfig();

  tcout << "Jacobian of the right ankle." << endl;
  MAL_MATRIX(,double) aJ;
  aJ = aJoint->jacobianJointWrtConfig();  
  DisplayMatrix(aJ,tcout);
 
  // Get the articular Jacobian from the right ankle to the right wrist.
  vector3d origin; origin(0) = 0.0; origin(1) = 0.0; origin(2) = 0.0;
  aHDR->getJacobian(*aHDR->rightAnkle(),
		    *aHDR->rightWrist(),
		    origin,
		    aJ);
  tcout << "Jacobian from the right ankle to the right wrist. " << endl;
  DisplayMatrix(aJ,tcout);
  MAL_MATRIX_RESIZE(aJ,3, MAL_MATRIX_NB_COLS(aJ));
  // Get the linear part of the articular Jacobian from the right ankle to the right wrist.
  aHDR->getPositionJacobian(*aHDR->rightAnkle(),
			    *aHDR->rightWrist(),
			    origin,
			    aJ);
  tcout << "Jacobian from the right ankle to the right wrist. " << endl;
  DisplayMatrix(aJ,tcout);
  
  // Get the angular part of the articular Jacobian from the right ankle to the right wrist.
  aHDR->getOrientationJacobian(*aHDR->rightAnkle(),
			       *aHDR->rightWrist(),
			       aJ);
  
  tcout << "Jacobian from the right ankle to the right wrist. " << endl;
  DisplayMatrix(aJ,tcout);
  
  
  tcout << "****************************" << endl;
  rootJoint->computeJacobianJointWrtConfig();
  aJ = rootJoint->jacobianJointWrtConfig();  
    
  tcout << "Rank of Root: " << rootJoint->rankInConfiguration() << endl;

  //  DisplayMatrix(aJ);

  aJoint = aHDR->waist();
  tcout << "****************************" << endl;
  aHDR->computeJacobianCenterOfMass();
  DisplayMatrix(aHDR->jacobianCenterOfMass(),tcout);
  tcout << "****************************" << endl;
  RecursiveDisplayOfJoints(rootJoint,tcout,10);

  tcout << "****************************" << endl;

  // Test rank of the hands.
  tcout << "Rank of the right hand "<< endl;
  tcout << aHDR->rightWrist()->rankInConfiguration() << endl;
  CjrlHand *rightHand = aHDR->rightHand();
  string empty("");
  DisplayHand(rightHand,empty,tcout);

  tcout << "Rank of the left hand "<< endl;
  tcout << aHDR->leftWrist()->rankInConfiguration() << endl;
  CjrlHand *leftHand = aHDR->leftHand();
  DisplayHand(leftHand,empty,tcout);

  // Test rank of the feet.
  tcout << "Rank of the right foot "<< endl;
  tcout << aHDR->rightFoot()->associatedAnkle()->rankInConfiguration() << endl;
  CjrlFoot *rightFoot = aHDR->rightFoot();
  DisplayFoot(rightFoot,empty,tcout);

  tcout << "Rank of the left foot "<< endl;
  tcout << aHDR->leftFoot()->associatedAnkle()->rankInConfiguration() << endl;
  CjrlFoot *leftFoot = aHDR->leftFoot();
  DisplayFoot(leftFoot,empty,tcout);

  tcout << "Current transformation of left Ankle."<< endl;
  dm4d(aHDR->leftAnkle()->currentTransformation(),tcout,empty);
  tcout << endl;
  tcout << "Current transformation of right Ankle."<< endl;
  dm4d(aHDR->rightAnkle()->currentTransformation(),tcout,empty);
  tcout << endl;
  //MAL_VECTOR_FILL(aCurrentVel,0.0);
  MAL_VECTOR_DIM(aCurrentAcc,double,NbOfDofs);
  MAL_VECTOR_FILL(aCurrentAcc,0.0);

  // This is mandatory for this implementation of computeForwardKinematics
  // to compute the derivative of the momentum.
  {
    string inProperty[5]={"TimeStep","ComputeAcceleration",
			  "ComputeBackwardDynamics", "ComputeZMP","ComputeAccelerationCoM"};
    string inValue[5]={"0.005","true","true","true","true"};
    for(unsigned int i=0;i<5;i++)
      aHDR->setProperty(inProperty[i],inValue[i]);

  }
  for(int i=0;i<4;i++)
    {
      aHDR->currentVelocity(aCurrentVel);
      aHDR->currentAcceleration(aCurrentAcc);
      aHDR->computeForwardKinematics();
      ZMPval = aHDR->zeroMomentumPoint();
      tcout << i << "-th value of ZMP : " 	
	    << filterprecision(ZMPval(0)) << " " 
	    << filterprecision(ZMPval(1)) << " " 
	    << filterprecision(ZMPval(2)) << endl;
      poscom = aHDR->positionCenterOfMass();
      tcout << "Should be equal to the CoM: "  
	    << filterprecision(poscom(0)) << " "
	    << filterprecision(poscom(1)) << " "  
	    << filterprecision(poscom(2)) << endl;
    }


  // Check the information on actuated joints.
  std::vector<CjrlJoint *> ActuatedJoints = aHDR->getActuatedJoints();
  
  tcout << "Size of actuated Joints:" << ActuatedJoints.size() << endl;
  for(unsigned int i=0;i<ActuatedJoints.size();i++)
    tcout << "Rank of actuated joints ("<<i<< ") in configuration :" 
	  << ActuatedJoints[i]->rankInConfiguration() << endl;

  tcout << "Humanoid mass:" << aHDR->mass() << endl;

  DisplayForces(aHDR,empty,tcout);
  DisplayTorques(aHDR,empty, tcout);
  
  // Test torques.


  
  
  //double dynamicDrift [] = { 0,0,0,0,0,0, 0.00106856, -3.65827, -3.24512, 3.39716, -0.0415928, -0.0752126, -2.19959e-016, 3.65805, -3.2449, 3.38531, -0.0423655, 0.0753855, 1.11009e-016, 4.43083, -2.0292e-020, -0.12657, 1.03213, -2.57379, -0.499568, -1.39763, -0.00344316, -0.302932, -0.00679369, 1.07342, 2.5637, 0.489739, -1.35744, -2.65727e-005, -0.262741, 0.00685521 };
  tcout << "Test Torques:" << endl;
  const matrixNxP& Torques = aHDR->currentTorques();
  for(unsigned int i=6;i<MAL_MATRIX_NB_ROWS(Torques);i++)
    {
      double torquefrominertia = 9.81 * InertiaMatrix(i,2);
      tcout << filterprecision(Torques(i,0)) << " " 
	    << filterprecision(torquefrominertia) << " \t DD \t " << Torques(i,0)-torquefrominertia << endl;
    }
  tcout << "Test Linear Velocity:" << endl;
  DisplayLinearVelocity(aHDR,tcout);
  tcout << "Test Angular Velocity:" << endl;
  DisplayAngularVelocity(aHDR,tcout);

  tcout << "Test Linear Acceleration:" << endl;
  DisplayLinearAcceleration(aHDR,tcout);
  tcout << "Test Angular Acceleration:" << endl;
  DisplayAngularAcceleration(aHDR,tcout);

  tcout.close();

  // ASCII Comparison between the generated output and the reference one
  // given in argument.
  if (argc==2)
    {
      char OurFileName[120]="output.txt";
      char ReportFileName[120]="reportthdr.txt";
      if (CompareTwoFiles(argv[1],
			  OurFileName,
			  ReportFileName))
	{
	  delete aHDR;
	  return 0;
	}
    }
  delete aHDR;
  return -1;
}
