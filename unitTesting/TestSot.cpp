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
 /* aCurrentConf[0]= 0.0590831;//0.000299955; //0;    
  aCurrentConf[1]= 0.0417015;//0.000200055;//0;    
  aCurrentConf[2]= 0.0228716; // 0.000100025;//0;
  aCurrentConf[3]= 0.214488;//0.00100018;//0;
  aCurrentConf[4]= 0.0683055;//0.00049965; //0;
  aCurrentConf[5]= 0.148034;//0.00070025;//0;
  aCurrentConf[6]= 0.00729376;//0; 
  aCurrentConf[7]= -0.00761911;//0;    
  aCurrentConf[8]= -0.448976;//-0.4538; //0;    
  aCurrentConf[9]= 0.875863;//0.8727;//0;    
  aCurrentConf[10]= -0.422244;//-0.4189;//0;    
  aCurrentConf[11]= 0.00298282;//0; */   
  /*aCurrentConf[0]= -0.0000116;
  aCurrentConf[1]= -0.0002217;  
  aCurrentConf[2]= 0.0002158;  
  aCurrentConf[3]= 0.0998908;  
  aCurrentConf[4]= 0.0003911;  
  aCurrentConf[5]= -0.0002331;  
  aCurrentConf[6]= 0.0000045;  
  aCurrentConf[7]= 0.0002049;  
  aCurrentConf[8]= -0.453892;
  aCurrentConf[9]= 0.872821;    
  aCurrentConf[10]= -0.418906;   
  aCurrentConf[11]= -0.0001011;*/


  /*aCurrentConf[0]= 0.0094069;
  aCurrentConf[1]= 0.0491741;  
  aCurrentConf[2]= 0.0226911;
  aCurrentConf[3]= 0.101701008;   
  aCurrentConf[4]= 0.0175715;  
  aCurrentConf[5]= 0.0310992;*/  
  

  MAL_VECTOR_FILL(aCurrentConf,0);
  aCurrentConf[0]= 1;
  aCurrentConf[3]= 1;
  aCurrentConf[6]= 1;
 /* aCurrentConf[6]= -0.00031383896314636907;  
  aCurrentConf[7]= -0.0020434338840028521;  
  aCurrentConf[8]= -0.45436437474987751;   
  aCurrentConf[9]= 0.87311862541367136;   
  aCurrentConf[10]= -0.41883230120129078;   
  aCurrentConf[11]= 0.0012307167278743977;*/


  MAL_VECTOR_DIM(aCurrentVel,double,NbOfDofs); 
  /*aCurrentVel[0] = 0.333117;//0.300103;//0.0;
  aCurrentVel[1] = 0.181923;//0.199826;//0.0;
  aCurrentVel[2] = 0.0914952;//0.100024;//100.0; //0.0;
  aCurrentVel[3] = 1.10586;//1.00022;//1.0; //0.0;
  aCurrentVel[4] = 0.315793;//0.499354;//0.0;
  aCurrentVel[5] = 0.695789;//0.700173;//0.0;
  aCurrentVel[6] = 0.105624;//9.99976e-006;//1.0; //0.0;
  aCurrentVel[7] = -0.112176;//3.05505e-011;//0.0;
  aCurrentVel[8] = 0.0769272;//-2.03924e-011;//0.0;
  aCurrentVel[9] = 0.0443456;//0.0769272;//2e-005;//0.0;
  aCurrentVel[10] = -0.0511774;//5.74501e-011;//0.0;
  aCurrentVel[11] = 0.0431235;//-1.04372e-010;//0.0;*/

/*	  aCurrentVel[0] = -0.0003466;  
	  aCurrentVel[1] = -0.0042194;  
	  aCurrentVel[2] = 0.0048053;  
	  aCurrentVel[3] = 0.996474;   
	  aCurrentVel[4] = 0.0082077;  
	  aCurrentVel[5] = -0.0050175;  
	  aCurrentVel[6] = -0.0003585;  
	  aCurrentVel[7] = 0.0063449;  
	  aCurrentVel[8] = -0.0028108;  
	  aCurrentVel[9] = 0.0026119;  
	  aCurrentVel[10] = -0.0001361;  
	  aCurrentVel[11] = -0.0031421; 
   */
	 /*
	  aCurrentVel[0] = 0.1102012;  
	  aCurrentVel[1] = 0.5219338;  
	  aCurrentVel[2] = 0.1551658;  
	  aCurrentVel[3] = 1.0420189;   
	  aCurrentVel[4] = 0.1850155;  
	  aCurrentVel[5] = 0.3047333;  
	  aCurrentVel[6] = -0.0107727;  
	  aCurrentVel[7] = -0.0633635;  
	  aCurrentVel[8] = -0.0171790;  
	  aCurrentVel[9] = 0.0122498;  
	  aCurrentVel[10] = 0.0016150;  
	  aCurrentVel[11] = 0.0381558; 
	*/

	MAL_VECTOR_FILL(aCurrentVel,0);  

	aCurrentVel[0]= 1;
	aCurrentVel[3]= 1;
	aCurrentVel[6]= 1;

  MAL_VECTOR_DIM(aCurrentAcc,double,NbOfDofs);
  /*aCurrentAcc[0] = 0.226104;//0.103884;//0.0;
  aCurrentAcc[1] = 0.00758384;//-0.173497;//0.0;
  aCurrentAcc[2] = -0.134775;//0.0237292;//0.0;
  aCurrentAcc[3] = 0.819365;//0.225367;//0.0;
  aCurrentAcc[4] = -1.1723;//-0.64887;//0.0;
  aCurrentAcc[5] = -0.208712;//0.171335; //0.0;
  aCurrentAcc[6] = 0.966209;//0.0158453;//0.0;
  aCurrentAcc[7] = -1.05;//-0.00609627;//0.0;
  aCurrentAcc[8] = 0.810933;//0.00261375;// 0.0;
  aCurrentAcc[9] = 0.396812;//0.0223524;//0.0;
  aCurrentAcc[10] = -0.514392;//-0.00233262;//0.0;
  aCurrentAcc[11] = 0.393449;//0.00258034;//0.0;*/

  /*aCurrentAcc[0] = -0.0068404;  
  aCurrentAcc[1] = -0.0481746;  
  aCurrentAcc[2] = 0.0485708;  
  aCurrentAcc[3] = -0.0755116;  
  aCurrentAcc[4] = 0.101113;   
  aCurrentAcc[5] = -0.0433774;  
  aCurrentAcc[6] = -0.0166893;  
  aCurrentAcc[7] = 0.130767;  
  aCurrentAcc[8] = -0.0563197;  
  aCurrentAcc[9] = 0.0300066;  
  aCurrentAcc[10] = -0.0010402;  
  aCurrentAcc[11] = -0.0652021;*/
     
 /* aCurrentAcc[0] = 0.0904319;//0.0993418;
  aCurrentAcc[1] = 0.325748;//0.2551395;  
  aCurrentAcc[2] = -0.485666;//-0.4556101; 
  aCurrentAcc[3] = 0.83797; //0.8380054;  
  aCurrentAcc[4] = -0.0740766; //-0.0738968;  
  aCurrentAcc[5] = 0.0732335; //0.0730011;
  aCurrentAcc[6] = -0.256356;//-0.2563551;  
  aCurrentAcc[7] = -1.31678;//-1.316774;  
  aCurrentAcc[8] = -0.344538; //-0.3445347;  
  aCurrentAcc[9] = 0.246259;//0.2462491;  
  aCurrentAcc[10] = 0.014446;//0.0144518;  
  aCurrentAcc[11] = 0.792761;//0.7927590;*/

  aCurrentAcc[0] = 1;  
  aCurrentAcc[1] = 0;  
  aCurrentAcc[2] = 0;  
  aCurrentAcc[3] = 1;
  aCurrentAcc[4] = 0;   
  aCurrentAcc[5] = 0;  
  aCurrentAcc[6] = 1;  
  aCurrentAcc[7] = 0;  
  aCurrentAcc[8] = 0;  
  aCurrentAcc[9] = 0;  
  aCurrentAcc[10] = 0;  
  aCurrentAcc[11] = 0;

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

    for(unsigned int i=6;i<MAL_MATRIX_NB_ROWS(Torques);i++)
		std::cout << "\n"  << filterprecision(Torques(i,0))  << std::endl;

  /*aCurrentAcc[6] = 1.0;
  aHDR->currentAcceleration(aCurrentAcc);

  // Initialize the backward dynamics (2de order, should be processed 3 times before).
	for( int i=0;i<3;++i )
		aHDR->computeForwardKinematics();

  aHDR->computeForwardKinematics();

  const matrixNxP& Torques1 = aHDR->currentTorques();
  const matrixNxP& Forces1 = aHDR->currentForces();*/

  std::cout << "InertiaMatrix("
       << MAL_MATRIX_NB_ROWS(InertiaMatrix)<< "," 
	   << MAL_MATRIX_NB_COLS(InertiaMatrix)<< ")"<< std::endl;
  DisplayMatrix(InertiaMatrix,std::cout);

  MAL_VECTOR_DIM(T, double, MAL_MATRIX_NB_ROWS(InertiaMatrix));
  for(unsigned int i=0;i<MAL_MATRIX_NB_ROWS(InertiaMatrix);i++)
  {
	   for(unsigned int j=0;j<MAL_MATRIX_NB_ROWS(InertiaMatrix);j++)
		   T[i] += InertiaMatrix(i,j)*aCurrentAcc[j];
  }

  std::cout << "\n" << std::endl;

  std::cout << "q = " << aCurrentConf << std::endl;
  std::cout << "dq = " << aCurrentVel << std::endl;
  std::cout << "ddq = " << aCurrentAcc << std::endl;
  
 /* for(unsigned int i=6;i<MAL_MATRIX_NB_ROWS(Torques1);i++)
    {
      //double torquefrominertia = 9.81 * InertiaMatrix(i,2);
	  std::cout << filterprecision(Torques1(i,0)) << " \t \t" << filterprecision(T[i]) << "\t \t"  << Torques1(i,0)-T[i] << std::endl;
    }
*/
  delete aHDR;
  return -1;
}
