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
#include <../src/private/JointPrivate.h>
#include <../src/private/DynamicBodyPrivate.h>
#include <../src/private/DynMultiBodyPrivate.h>

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
  //aCurrentConf[0]= 1;    
  //aCurrentConf[2] = 0.705;
  //aCurrentConf[1]= 1;
  //aCurrentConf[4]= 2;
  //aCurrentConf[5]= 3;

  
  for(int i=0;i<(NbOfDofs-6 < 40 ? NbOfDofs-6 : 40) ;i++)
  aCurrentConf[lindex++] = 0.0; 
  /*aCurrentConf[0]= -3.99736e-16;
  aCurrentConf[1]=  0.250525;
  aCurrentConf[2]=   -0.490775;
  aCurrentConf[3]= 1.02102;    
  aCurrentConf[4]= -0.530246;
  aCurrentConf[5]= -0.250525;
  aCurrentConf[6]= 0.0557106;       
  aCurrentConf[7]= 0.41194;
  aCurrentConf[8]= -0.926154;
  aCurrentConf[9]= 1.85522;   
  aCurrentConf[10]= -1.20284;
  aCurrentConf[11]= -0.386769;
  aCurrentConf[12]= -0.0162314;    
  aCurrentConf[13]= 0.00348474;
  aCurrentConf[14]= -0.00121083;
  aCurrentConf[15]= 0.000730251;
  aCurrentConf[16]= 0.101374; 
  aCurrentConf[17]= -0.360891;
  aCurrentConf[18]= -0.231191;
  aCurrentConf[19]= -0.946601;
  aCurrentConf[20]= -7.69141e-05;
  aCurrentConf[21]= -0.0144256;
  aCurrentConf[22]= 0.100271;
  aCurrentConf[23]= 0.099589;
  aCurrentConf[24]= 0.109393;
  aCurrentConf[25]= 0.145344;
  aCurrentConf[26]= -0.952454;
  aCurrentConf[27]= -0.000751609;
  aCurrentConf[28]= -0.0157137;
  aCurrentConf[29]= 0.100424;*/
                         
  tcout << "NbOfDofs:" << NbOfDofs << std::endl; 
  tcout << "Current Configuration :" << aCurrentConf << std::endl;
  aHDR->currentConfiguration(aCurrentConf);

  MAL_VECTOR_DIM(aCurrentVel,double,NbOfDofs); 
  lindex=0;
  for(int i=0;i<NbOfDofs;i++)
    aCurrentVel[lindex++] = 0.0;
  //aCurrentVel[2] = 100.0;
  //aCurrentVel[3] = 1.0;
  //aCurrentVel[6] = 1.0;
  //  aCurrentVel[0]=1.0;
  //aCurrentVel[6]=1.0;
  //aCurrentVel[7]=2.0;
  //aCurrentVel[8]=3.0;
  //aCurrentVel[9]=4.0;
  //aCurrentVel[10]=5.0;
  //aCurrentVel[11]=6.0;

  MAL_S3_VECTOR(ZMPval,double);


  aHDR->currentVelocity(aCurrentVel);
  //MAL_VECTOR_FILL(aCurrentVel,0.0);
  MAL_VECTOR_DIM(aCurrentAcc,double,NbOfDofs);
  MAL_VECTOR_FILL(aCurrentAcc,0.0);
  /*aCurrentAcc[0] = -1.66477;          
  aCurrentAcc[1] = -0.314195;
  aCurrentAcc[2] = 3.09524; 
  aCurrentAcc[3] = 18.0223;
  aCurrentAcc[4] = 8.74466;
  aCurrentAcc[5] = -2.52987;
  aCurrentAcc[6] = 0.4446;
  aCurrentAcc[7] = 0.0365203;
  aCurrentAcc[8] = 2.80994;
  aCurrentAcc[9] = 1.3171;
  aCurrentAcc[10] = 0.037342;
  aCurrentAcc[11] = -0.0073251;
  aCurrentAcc[12] = -0.207606;
  aCurrentAcc[13] = 0.0514206;
  aCurrentAcc[14] = -0.0275567;
  aCurrentAcc[15] = 0.0869552;
  aCurrentAcc[16] = 0.186102;
  aCurrentAcc[17] = -0.0592583;
  aCurrentAcc[18] = -0.195104;
  aCurrentAcc[19] = 0.048196;
  aCurrentAcc[20] = -0.00373422;
  aCurrentAcc[21] = 0.0361445;
  aCurrentAcc[22] = -0.00642234;
  aCurrentAcc[23] = 0.136126;
  aCurrentAcc[24] = -0.0368792;
  aCurrentAcc[25] = -0.0538581;
  aCurrentAcc[26] = 0.0329828;
  aCurrentAcc[27] = -0.00517412;
  aCurrentAcc[28] = 0.0274472;
  aCurrentAcc[29] = 0.00428444;
  aCurrentAcc[30] = 0; 
  aCurrentAcc[31] = 0; 
  aCurrentAcc[32] = 0; 
  aCurrentAcc[33] = 0.135294;
  aCurrentAcc[34] = 0.0071321;
  aCurrentAcc[35] = 0.99078;*/

 // aCurrentAcc[1]=1.0;
 // aCurrentAcc[9]=1.0;
 // aCurrentAcc[12]=1.0;
  aHDR->currentAcceleration(aCurrentAcc);

  //  aHDR->setComputeZMP(true);
  // This is mandatory for this implementation of computeForwardKinematics
  // to compute the derivative of the momentum.
  {
	std::string prop,arg;
	//prop="ComputeAcceleration";  arg="true"; aHDR->setProperty( prop,arg ); // maybe false in sot -- take care to it
	//prop="ComputeBackwardDynamics"; arg="true";  aHDR->setProperty( prop,arg );
	//prop="ComputeZMP";  arg="false"; aHDR->setProperty( prop,arg ); // should be true?  --- no, ok like that
	//prop="ComputeAccelerationCoM";  arg="true"; aHDR->setProperty( prop,arg ); // should be true ? --- YES
	prop="ComputeCoM";  arg="true"; aHDR->setProperty( prop,arg );
	prop="ComputeVelocity";  arg="true"; aHDR->setProperty( prop,arg );
    prop="ComputeMomentum";  arg="false"; aHDR->setProperty( prop,arg );

    string inProperty[5]={"TimeStep","ComputeAcceleration",
			  "ComputeBackwardDynamics", "ComputeZMP","ComputeAccelerationCoM"};
    string inValue[5]={"0.005","true","true","true","true"};
    for(unsigned int i=0;i<5;i++)
      aHDR->setProperty(inProperty[i],inValue[i]);

  }

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


  /* --- Inertia matrix ----------------------------------------------------- */
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

  /* --- Jacobians ---------------------------------------------------------- */
  /* Get the Jacobian of the right ankle. */
  CjrlJoint  * aJoint = aHDR->rightAnkle();
  aJoint->computeJacobianJointWrtConfig();

  tcout << "Jacobian of the right ankle." << endl;
  MAL_MATRIX(,double) aJ;
  aJ = aJoint->jacobianJointWrtConfig();
  DisplayMatrix(aJ,tcout);

  // DynMultiBodyPrivate* FinalBody = (DynMultiBodyPrivate*)(aJoint->linkedBody());
  // matrixNxP TestJacobian;
  // MAL_MATRIX_RESIZE(TestJacobian, 6, NbOfDofs);
  // vector3d CoM = FinalBody->localCenterOfMass();
  // FinalBody->getJacobian(*FinalBody->rootJoint(),*aJoint,CoM,TestJacobian);
  // std::cout << "TestJacobian = " << TestJacobian << std::endl;

  /* Get the Jacobian of the left ankle. */
  CjrlJoint  * aJointl = aHDR->leftAnkle();
  aJointl->computeJacobianJointWrtConfig();
  MAL_MATRIX(,double) aJl;
  aJl = aJointl->jacobianJointWrtConfig();
  tcout << "Jacobian of the left ankle." << endl;
  DisplayMatrix(aJl,tcout);

  /* Get the Jacobian of the right ankle. */
  CjrlJoint  * aJointwr = aHDR->rightWrist();
  aJointwr->computeJacobianJointWrtConfig();
  MAL_MATRIX(,double) aJwr;
  aJwr = aJointwr->jacobianJointWrtConfig();
  tcout << "Jacobian of the right wrist." << endl;
  DisplayMatrix(aJwr,tcout);

  /* Get the articular Jacobian from the right ankle to the right wrist. */
  vector3d origin; origin(0) = 0.0; origin(1) = 0.0; origin(2) = 0.0;
  aHDR->getJacobian(*aHDR->rightAnkle(),
		    *aHDR->rightWrist(),
		    origin,aJ);
  tcout << "Jacobian from the right ankle to the right wrist. " << endl;
  DisplayMatrix(aJ,tcout);

  /* Translation part of the Jacobian from the right ankle to the right wrist. */
  MAL_MATRIX_RESIZE(aJ,3, MAL_MATRIX_NB_COLS(aJ));
  aHDR->getPositionJacobian(*aHDR->rightAnkle(),
			    *aHDR->rightWrist(),
			    origin,aJ);
  tcout << "Jacobian from the right ankle to the right wrist. " << endl;
  DisplayMatrix(aJ,tcout);

  /* Angular part of the articular Jacobian from the right ankle to the right wrist. */
  aHDR->getOrientationJacobian(*aHDR->rightAnkle(),
			       *aHDR->rightWrist(),
			       aJ);
  tcout << "Jacobian from the right ankle to the right wrist. " << endl;
  DisplayMatrix(aJ,tcout);

  tcout << "****************************" << endl;
  rootJoint->computeJacobianJointWrtConfig();
  aJ = rootJoint->jacobianJointWrtConfig();
  tcout << "Rank of Root: " << rootJoint->rankInConfiguration() << endl;
  aJoint = aHDR->waist();
  tcout << "****************************" << endl;
  aHDR->computeJacobianCenterOfMass();
  DisplayMatrix(aHDR->jacobianCenterOfMass(),tcout);
  tcout << "****************************" << endl;
  RecursiveDisplayOfJoints(rootJoint,tcout,10);
  tcout << "****************************" << endl;

  /* --- Positions ---------------------------------------------------------- */
  /* Test rank of the hands. */
  tcout << "Rank of the right hand "<< endl;
  tcout << aHDR->rightWrist()->rankInConfiguration() << endl;
  CjrlHand *rightHand = aHDR->rightHand();
  string empty("");
  DisplayHand(rightHand,empty,tcout);
  tcout << "Rank of the left hand "<< endl;
  tcout << aHDR->leftWrist()->rankInConfiguration() << endl;
  CjrlHand *leftHand = aHDR->leftHand();
  DisplayHand(leftHand,empty,tcout);

  /* Test rank of the feet. */
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

  /* --- ZMP ----------x------------------------------------------------------ */
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

  /* Check the information on actuated joints. */
  std::vector<CjrlJoint *> ActuatedJoints = aHDR->getActuatedJoints();
  tcout << "Size of actuated Joints:" << ActuatedJoints.size() << endl;
  for(unsigned int i=0;i<ActuatedJoints.size();i++)
    tcout << "Rank of actuated joints ("<<i<< ") in configuration :"
	  << ActuatedJoints[i]->rankInConfiguration() << endl;

  tcout << "Humanoid mass:" << aHDR->mass() << endl;
  DisplayForces(aHDR,empty,tcout);
  DisplayTorques(aHDR,empty, tcout);

  /* --- Test torques ------------------------------------------------------- */
  // double dynamicDrift [] = { 0,0,0,0,0,0, 0.00106856, -3.65827, -3.24512, 3.39716, -0.0415928, -0.0752126, -2.19959e-016, 3.65805, -3.2449, 3.38531, -0.0423655, 0.0753855, 1.11009e-016, 4.43083, -2.0292e-020, -0.12657, 1.03213, -2.57379, -0.499568, -1.39763, -0.00344316, -0.302932, -0.00679369, 1.07342, 2.5637, 0.489739, -1.35744, -2.65727e-005, -0.262741, 0.00685521 };
  tcout << "Test Torques:" << endl;
  const matrixNxP& Torques = aHDR->currentTorques();
  for(unsigned int i=6;i<MAL_MATRIX_NB_ROWS(Torques);i++)
    {
      double torquefrominertia = 9.81 * InertiaMatrix(i,2);
      tcout << filterprecision(Torques(i,0)) << " "
	    << filterprecision(torquefrominertia) << " \t DD \t "
	    << Torques(i,0)-torquefrominertia << endl;
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
