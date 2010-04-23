/* @doc \file Test Torques and forces for a planar elbow robot.

   Copyright (c) 2010
   @author Olivier Stasse
   JRL-Japan, CNRS/AIST
 
   All rights reserved.

   Please refers to file License.txt for details on the license.

*/

#include <string>
#include <fstream>
#include <dynamicsJRLJapan/dynamicsJRLJapanFactory.h>
using namespace std;
using namespace dynamicsJRLJapan;

#include "TwoLinksModel.h"


int main(int argc, char *argv[])
{
  dynamicsJRLJapan::ObjectFactory robotDynamicsObjectConstructor;
  TwoLinksModelParameters aSetOfPlanarElbowParameters;

  // Fix length of links.
  aSetOfPlanarElbowParameters.l[0] = 1.0;
  aSetOfPlanarElbowParameters.l[1] = 1.0;
  
  // Fix position of center of masses.
  aSetOfPlanarElbowParameters.lc[0] = 0.5;
  aSetOfPlanarElbowParameters.lc[1] = 0.5;
  
  // Fix masses.
  aSetOfPlanarElbowParameters.m[0] = 1.0;
  aSetOfPlanarElbowParameters.m[1] = 1.0;

  // Fix inertia matrices.
  MAL_S3x3_MATRIX_SET_IDENTITY(aSetOfPlanarElbowParameters.I[0]);
  MAL_S3x3_MATRIX_ACCESS_I_J(aSetOfPlanarElbowParameters.I[0],0,0)= 3.0;
  MAL_S3x3_MATRIX_ACCESS_I_J(aSetOfPlanarElbowParameters.I[0],1,1)= 2.0;
  MAL_S3x3_MATRIX_SET_IDENTITY(aSetOfPlanarElbowParameters.I[1]);
  MAL_S3x3_MATRIX_ACCESS_I_J(aSetOfPlanarElbowParameters.I[1],0,0)= 3.0;
  MAL_S3x3_MATRIX_ACCESS_I_J(aSetOfPlanarElbowParameters.I[1],1,1)= 2.0;

  CTwoLinksModel * aTwoLinksModel = new CTwoLinksModel(&robotDynamicsObjectConstructor,
						       aSetOfPlanarElbowParameters);

  MAL_VECTOR_DIM(aCurrentConf,double,2);
  MAL_VECTOR_DIM(aCurrentVelocity,double,2);
  MAL_VECTOR_DIM(aCurrentAcceleration,double,2);

  {
    string inProperty[5]={"TimeStep","ComputeAcceleration",
			  "ComputeBackwardDynamics", "ComputeZMP","ComputeAccelerationCoM"};
    string inValue[5]={"0.005","true","true","true","true"};
    for(unsigned int i=0;i<5;i++)
      aTwoLinksModel->setProperty(inProperty[i],inValue[i]);

  }
  string testname;

  // Test 1. The robot stands still
  for(int i=0;i<2;i++)
    {
      aCurrentConf[i] = 0.0;
      aCurrentVelocity[i] = 0.0;
      aCurrentAcceleration[i] = 0.0;
    }

  testname = "Test 1";
  if (!aTwoLinksModel->TestInstance(aCurrentConf,
				    aCurrentVelocity,
				    aCurrentAcceleration,
				    testname))
    return -1;
  else 
    cout << testname << " ok." << endl;

  // Test 2. We impose a velocity on the first link.
  aCurrentVelocity[0] = 1.0;
  testname = "Test 2";
  if (!aTwoLinksModel->TestInstance(aCurrentConf,
				    aCurrentVelocity,
				    aCurrentAcceleration,
				    testname))
    return -1;
  else 
    cout << testname << " ok." << endl;
  
  
  // Test 3. We change the configuration of the first link.
  aCurrentConf[0] = M_PI/4.0;
  testname = "Test 3";
  if (!aTwoLinksModel->TestInstance(aCurrentConf,
				    aCurrentVelocity,
				    aCurrentAcceleration,
				    testname))
    return -1;
  else 
    cout << testname << " ok." << endl;

  // Test 4. We change the configuration of the second link.
  aCurrentConf[1] = M_PI/3.0;
  testname = "Test 4";
  if (!aTwoLinksModel->TestInstance(aCurrentConf,
				    aCurrentVelocity,
				    aCurrentAcceleration,
				    testname))
    return -1;
  else 
    cout << testname << " ok." << endl;
 
  // Test 5. We change the velocity of the second link.
  aCurrentVelocity[1] = 1.0;
  testname = "Test 5";
  if (!aTwoLinksModel->TestInstance(aCurrentConf,
				    aCurrentVelocity,
				    aCurrentAcceleration,
				    testname))
    return -1;
  else 
    cout << testname << " ok." << endl;
   

  // Test 6. We change the acceleration of the second link.
  aCurrentAcceleration[1] = 1.0;
  testname = "Test 6";
  if (!aTwoLinksModel->TestInstance(aCurrentConf,
				    aCurrentVelocity,
				    aCurrentAcceleration,
				    testname))
    return -1;
  else 
    cout << testname << " ok." << endl;

  // Test 7. We change the acceleration of the first link.
  aCurrentAcceleration[0] = 1.0;
  testname = "Test 7";
  if (!aTwoLinksModel->TestInstance(aCurrentConf,
				    aCurrentVelocity,
				    aCurrentAcceleration,
				    testname))
    return -1;
  else 
    cout << testname << " ok." << endl;
  
}
