/* @doc \file Implements the planar elbow robot.

   Copyright (c) 2010
   @author Olivier Stasse
   JRL-Japan, CNRS/AIST
 
   All rights reserved.

   Please refers to file License.txt for details on the license.

*/
#include <iostream>
using namespace std;

#include "TwoLinksModel.h"

#define ODEBUG3(x) { cerr << __FILE__ << ":" << __LINE__ <<  " " << x << endl; }

#if 1
#define ODEBUG(x) { cerr << __FILE__ << ":" << __LINE__ <<  " " << x << endl; }
#else
#define ODEBUG(x)
#endif

namespace dynamicsJRLJapan
{
  CTwoLinksModel::CTwoLinksModel(CjrlRobotDynamicsObjectFactory *anObjectFactory,
				 TwoLinksModelParameters &aSetOfParameters) 
  : dynamicRobot(anObjectFactory)
  {
    m_g = 9.81;

    /*! Specify the orthonormal basis.
      (different from Spong orientation, because
      we don't use DH convention but a rotation around x-axis) */
    m_i[0] = 0; m_i[1] = 0.0; m_i[2] = 1.0;
    m_j[0] = 0; m_j[1] = 1.0; m_j[2] = 0.0;
    m_k[0] = 1; m_k[1] = 0.0; m_k[2] = 0.0;

    m_SetOfParameters = aSetOfParameters;
    
    matrix4d pose;
    MAL_S4x4_MATRIX_SET_IDENTITY(pose);

    // Create Joint 1
    CjrlJoint* j1=0;
    j1 = anObjectFactory->createJointRotation(pose);

    // Joint 1 is root of the tree.
    rootJoint(*j1);

    // Create Link 1
    CjrlBody* l1=0;
    l1= anObjectFactory->createBody();
    l1->mass(m_SetOfParameters.m[0]);
    l1->inertiaMatrix(m_SetOfParameters.I[0]);
    vector3d lc ;
    lc(0) = 0.0;
    lc(1) = 0.0;
    lc(2) = m_SetOfParameters.lc[0];
    l1->localCenterOfMass(lc);
    
    // Relate joint1 and link1
    j1->setLinkedBody(*l1);

    // Create Joint 2
    MAL_S4x4_MATRIX_ACCESS_I_J(pose,2,3)= m_SetOfParameters.l[0];
   
    CjrlJoint* j2=0;
    j2 = anObjectFactory->createJointRotation(pose);
    j1->addChildJoint(*j2);

    // Create Link 2
    CjrlBody* l2=0;
    l2= anObjectFactory->createBody();
    l2->mass(m_SetOfParameters.m[1]);
    l2->inertiaMatrix(m_SetOfParameters.I[1]);
    lc(0) = 0.0;
    lc(1) = 0.0;
    lc(2) = m_SetOfParameters.lc[1];
    l2->localCenterOfMass(lc);
    
    // Relate link 2 and joint 2
    j2->setLinkedBody(*l2);

    // Set actuated joints.
    std::vector<CjrlJoint *> ActuatedJoints;
    ActuatedJoints.resize(2);
    ActuatedJoints[0] = j1;
    ActuatedJoints[1] = j2;

    setActuatedJoints(ActuatedJoints);

    initialize();
  }

  void CTwoLinksModel::ForwardRecursionLink1(vector3d &ac1,
					     vector3d &g1,
					     vector3d &ae1)
  {
    ODEBUG("Forward recursion link 1:");
    // Forward recursion Link 1
    vectorN lcurrentConfiguration = currentConfiguration();
    vectorN lcurrentVelocity = currentVelocity();
    vectorN lcurrentAcceleration = currentAcceleration();

    
    // Compute linear acceleration of the center of mass for link 1.
    /*! Equation 7.165 */
    vector3d firstterm,secondterm,tmp,tmp2,tmp3; 
    tmp = m_k * lcurrentAcceleration[0];
    tmp2 = m_i * m_SetOfParameters.lc[0];
    ODEBUG("lc : " << tmp2);
    MAL_S3_VECTOR_CROSS_PRODUCT(firstterm,tmp,tmp2);
    
    tmp = m_k * lcurrentVelocity[0];
    MAL_S3_VECTOR_CROSS_PRODUCT(tmp3,tmp,tmp2);
    MAL_S3_VECTOR_CROSS_PRODUCT(secondterm,tmp,tmp3);
    ODEBUG("lw1 = "<< m_k*lcurrentVelocity[0]);
    ODEBUG("ldw1 = "<< m_k*lcurrentAcceleration[0]);
    ODEBUG("lw x (lw x lc): " << secondterm );
    ac1 = firstterm + secondterm;
    
    // Computes g1.
    matrix3d Rf1f0t;
    MAL_S3x3_MATRIX_SET_IDENTITY(Rf1f0t);
    MAL_S3x3_MATRIX_ACCESS_I_J(Rf1f0t,1,1) = 
      cos(lcurrentConfiguration[0]);
    MAL_S3x3_MATRIX_ACCESS_I_J(Rf1f0t,1,2) = 
      sin(lcurrentConfiguration[0]);
    MAL_S3x3_MATRIX_ACCESS_I_J(Rf1f0t,2,1) = 
      -sin(lcurrentConfiguration[0]);
    MAL_S3x3_MATRIX_ACCESS_I_J(Rf1f0t,2,2) = 
      cos(lcurrentConfiguration[0]);
    ODEBUG("Rf1f0t1: " << Rf1f0t );
    tmp = m_i * -m_g;
    ODEBUG("i * -g: " << tmp );
    MAL_S3x3_C_eq_A_by_B(g1,Rf1f0t,tmp);
    ODEBUG("g1: " << g1);
    // Compute linear acceleration of the end of link 1.
    // see remark for eq. 7167 
    tmp = m_k * lcurrentAcceleration[0];
    tmp2 = m_i * m_SetOfParameters.l[0];
    MAL_S3_VECTOR_CROSS_PRODUCT(firstterm,tmp,tmp2);
    
    tmp = m_k * lcurrentVelocity[0];
    MAL_S3_VECTOR_CROSS_PRODUCT(tmp3,tmp,tmp2);
    MAL_S3_VECTOR_CROSS_PRODUCT(secondterm,tmp,tmp3);
    ae1 = firstterm + secondterm;
    
  }

  void CTwoLinksModel::ForwardRecursionLink2(vector3d &ae1,
					     vector3d &ac2,
					     vector3d &g2)
  {
    ODEBUG("Forward recursion link 2:" );
    vectorN lcurrentConfiguration = currentConfiguration();
    vectorN lcurrentVelocity = currentVelocity();
    vectorN lcurrentAcceleration = currentAcceleration();

    matrix3d Rf2f1t;
    MAL_S3x3_MATRIX_SET_IDENTITY(Rf2f1t);
    MAL_S3x3_MATRIX_ACCESS_I_J(Rf2f1t,1,1) = 
      cos(lcurrentConfiguration[1]);
    MAL_S3x3_MATRIX_ACCESS_I_J(Rf2f1t,1,2) = 
      sin(lcurrentConfiguration[1]);
    MAL_S3x3_MATRIX_ACCESS_I_J(Rf2f1t,2,1) = 
      -sin(lcurrentConfiguration[1]);
    MAL_S3x3_MATRIX_ACCESS_I_J(Rf2f1t,2,2) = 
      cos(lcurrentConfiguration[1]);
    
    // Linear acceleration of the center of mass for link 2 (local frame).
    vector3d firstterm, sndterm, thirdterm;
    vector3d tmp,tmp2,tmp3;
    MAL_S3x3_C_eq_A_by_B(firstterm ,Rf2f1t,ae1);
   
    double addacceleration = (lcurrentAcceleration[0] + lcurrentAcceleration[1]);
    tmp = m_i * m_SetOfParameters.lc[1];
    MAL_S3_VECTOR_CROSS_PRODUCT(tmp2,m_k,tmp);
    sndterm = tmp2 * addacceleration;
   
    double addvelocity = (lcurrentVelocity[0] + lcurrentVelocity[1]);    
    tmp3 = tmp2 * addvelocity;
    tmp = m_k * addvelocity;
    MAL_S3_VECTOR_CROSS_PRODUCT(thirdterm,tmp,tmp3);
    ODEBUG("ac2 first term : " << firstterm);
    ac2 = firstterm + sndterm + thirdterm;

    // Gravity for link 2 
    matrix3d Rf2f0t;
    MAL_S3x3_MATRIX_SET_IDENTITY(Rf2f0t);
    MAL_S3x3_MATRIX_ACCESS_I_J(Rf2f0t,1,1) = 
      cos(lcurrentConfiguration[0]+
	  lcurrentConfiguration[1]);
    MAL_S3x3_MATRIX_ACCESS_I_J(Rf2f0t,1,2) = 
      sin(lcurrentConfiguration[0]+
	  lcurrentConfiguration[1]);
    MAL_S3x3_MATRIX_ACCESS_I_J(Rf2f0t,2,1) = 
      -sin(lcurrentConfiguration[0]+
	   lcurrentConfiguration[1]);
    MAL_S3x3_MATRIX_ACCESS_I_J(Rf2f0t,2,2) = 
      cos(lcurrentConfiguration[0]+
	  lcurrentConfiguration[1]);
    ODEBUG("Rf2f0t1: " << Rf2f0t );
    tmp = m_i * -m_g;
    ODEBUG("i * -g: " << tmp);
    MAL_S3x3_C_eq_A_by_B(g2,Rf2f0t,tmp);
    ODEBUG("g2: " << g2);
    ODEBUG("lw2 = "<< m_k*(lcurrentVelocity[0]+lcurrentVelocity[1]));
    ODEBUG("ldw2 = "<< m_k*(lcurrentAcceleration[0]+lcurrentAcceleration[1]));

  }


  void CTwoLinksModel::BackwardRecursionLink2(vector3d &ac2,
					      vector3d &g2,
					      vector3d &w2,
					      vector3d &ldw2,
					      vector3d &f2,
					      vector3d &t2)
  {
    ODEBUG("Backward recursion link 2:");
    vectorN lcurrentConfiguration = currentConfiguration();
    vectorN lcurrentVelocity = currentVelocity();
    vectorN lcurrentAcceleration = currentAcceleration();
    
    // Force. (7.172)
    f2 = ( ac2 -g2);
    f2 = f2 * m_SetOfParameters.m[1];
    ODEBUG("g2:" << g2 );
    ODEBUG("ac2:" << ac2);
    ODEBUG("Result f2:" << f2);

    // Torque (7.173)
    vector3d firstterm, secondterm, thirdterm, tmp;

    MAL_S3x3_C_eq_A_by_B(firstterm, m_SetOfParameters.I[1],ldw2);
    ODEBUG("first term:" << firstterm << " ldw2: " << ldw2);
    MAL_S3x3_C_eq_A_by_B(tmp,m_SetOfParameters.I[1],w2);
    MAL_S3_VECTOR_CROSS_PRODUCT(secondterm,w2,tmp);
    ODEBUG("second term:" << secondterm)
    tmp = m_i * m_SetOfParameters.lc[1];
    MAL_S3_VECTOR_CROSS_PRODUCT(thirdterm,f2,tmp);
    ODEBUG("third term:" << thirdterm );
	
    t2 = firstterm + secondterm - thirdterm;

    ODEBUG("Result t2:" << t2 );
  }

  void CTwoLinksModel::BackwardRecursionLink1(vector3d &ac1,
					      vector3d &g1,
					      vector3d &f2,
					      vector3d &t2,
					      vector3d &lw1,
					      vector3d &ldw1,
					      vector3d &f1,
					      vector3d &t1)
  {
    ODEBUG("Backward recursion link 1:" );
    vectorN lcurrentConfiguration = currentConfiguration();
    vectorN lcurrentVelocity = currentVelocity();
    vectorN lcurrentAcceleration = currentAcceleration();

    
    // Force. (7.175)
    matrix3d Rf2f1;
    MAL_S3x3_MATRIX_SET_IDENTITY(Rf2f1);
    MAL_S3x3_MATRIX_ACCESS_I_J(Rf2f1,1,1) = 
      cos(lcurrentConfiguration[1]);
    MAL_S3x3_MATRIX_ACCESS_I_J(Rf2f1,1,2) = 
      -sin(lcurrentConfiguration[1]);
    MAL_S3x3_MATRIX_ACCESS_I_J(Rf2f1,2,1) = 
      sin(lcurrentConfiguration[1]);
    MAL_S3x3_MATRIX_ACCESS_I_J(Rf2f1,2,2) = 
      cos(lcurrentConfiguration[1]);

    ODEBUG("Rf2f1t:" << Rf2f1 );
    vector3d f2inf1;
    MAL_S3x3_C_eq_A_by_B(f2inf1,Rf2f1,f2);
    ODEBUG("intermediate f1:" << f1 );
    ODEBUG("g1:" << g1 );
    ODEBUG("ac1:" << ac1 );
    f1 = f2inf1 + ( ac1 -g1) * m_SetOfParameters.m[0];
    ODEBUG("f1:" << f1 );

    // Torque
    vector3d firstterm, sndterm, thirdterm, fourthterm, fifthterm;
    vector3d tmp,tmp2;

    // Compute first term  R^2_1 tau2
    MAL_S3x3_C_eq_A_by_B(firstterm,Rf2f1,t2);

    // Compute second term  f_1 x lc1 i 
    tmp = m_i * m_SetOfParameters.lc[0];
    MAL_S3_VECTOR_CROSS_PRODUCT(sndterm,f1,tmp);

    // Compute third term  (R^2_1 f_2) x (l1 -lc1) i 
    tmp = m_i * (m_SetOfParameters.l[0] - m_SetOfParameters.lc[0]);
    MAL_S3_VECTOR_CROSS_PRODUCT(thirdterm,f2inf1,tmp);
    
    // Compute fourth term I1 ldw1
    MAL_S3x3_C_eq_A_by_B(fourthterm,m_SetOfParameters.I[0],ldw1);
									
    // Compute fifth term w1 x (I1 w1)
    MAL_S3x3_C_eq_A_by_B(tmp,m_SetOfParameters.I[0],lw1);
    MAL_S3_VECTOR_CROSS_PRODUCT(fifthterm,lw1,tmp);
    
    t1 = firstterm - sndterm - thirdterm + fourthterm + fifthterm;
  }

  void CTwoLinksModel::computeAnalyticalBackwardDynamics(vector3d Forces[2],
							 vector3d Torques[2])
  {

    // Angular velocity (local frame)
    vector3d lw[2];
    // Angular acceleration (local frame)
    vector3d ldw[2];
    // Vector from frame i to center of mass (local frame)
    vector3d rf1c1,rf2c2,rf2c1,rf3c2;
    // Vector from frame i to frame j (local frame)
    vector3d rf1f2,rf2f3;
    
    vectorN lcurrentConfiguration = currentConfiguration();
    vectorN lcurrentVelocity = currentVelocity();
    vectorN lcurrentAcceleration = currentAcceleration();
    
    // Compute angular velocity and acceleration. (7.162) p.279
    lw[0] = m_k * lcurrentVelocity[0];
    ldw[0] = m_k * lcurrentAcceleration[0];
    lw[1] = m_k * (lcurrentVelocity[0] + lcurrentVelocity[1]);
    ldw[1] = m_k * (lcurrentAcceleration[0] + lcurrentAcceleration[1]);
    
    // Compute vectors. 7.163 & 7.164
    rf1c1 = m_i * m_SetOfParameters.lc[0]; 
    rf2c1 = m_i * (m_SetOfParameters.l[0]-m_SetOfParameters.lc[0]);
    rf1f2 = m_i * m_SetOfParameters.l[0]; 

    rf2c2 = m_i * m_SetOfParameters.lc[1]; 
    rf3c2 = m_i * (m_SetOfParameters.l[1]-m_SetOfParameters.lc[1]);
    rf2f3 = m_i * m_SetOfParameters.l[1]; 


    // Forward Recursion Link 1.
    vector3d ac1,g1,ae1;
    ForwardRecursionLink1(ac1,g1,ae1);
    
    // Forward recursion Link 2 
    vector3d ac2,g2;
    ForwardRecursionLink2(ae1,ac2,g2);

    // Backward recursion Link 2 
    BackwardRecursionLink2(ac2,g2, lw[1], ldw[1], Forces[1],Torques[1]);

    // Backward recursion Link 1 
    BackwardRecursionLink1(ac1,g1,Forces[1],Torques[1],
			   lw[0],ldw[0],
			   Forces[0],Torques[0]);

    
    
  }

  bool CTwoLinksModel::TestInstance(vectorN &aCurrentConf,
				    vectorN &aCurrentVelocity,
				    vectorN &aCurrentAcceleration,
				    string &testname)
  {
    currentConfiguration(aCurrentConf);
    currentVelocity(aCurrentVelocity);
    currentAcceleration(aCurrentAcceleration);
    computeForwardKinematics();
    
    matrixNxP GenericTorques, GenericForces;
    GenericTorques = currentTorques();
    GenericForces = currentForces();
    
    ODEBUG("From generic algorithm : " );
    ODEBUG("Forces  : " << GenericForces );
    ODEBUG("Torques :" << GenericTorques );
    
    vector3d AnalyticalForces[2], AnalyticalTorques[2];
    
    computeAnalyticalBackwardDynamics(AnalyticalForces,
				      AnalyticalTorques);
    
    ODEBUG("From analytical model : " );
    ODEBUG("Forces  : " 
	    << AnalyticalForces[0] << endl
	    << AnalyticalForces[1]);
    ODEBUG("Torques  : " 
	    << AnalyticalTorques[0] << endl
	    << AnalyticalTorques[1]);
    for(unsigned int i=0;i<2;i++)
      for(unsigned int j=0;j<3;j++)
	{
	  if (fabs(AnalyticalForces[i](j) - 
		   GenericForces(i,j)) > 1e-8)
	    {
	      cout << testname << " Pb in link " << i << " Force component "<< j 
		   << " Generic: " << GenericForces(i,j) 
		   << " Analytical " << AnalyticalForces[i](j) << endl;
	      return false;
	    }
	  
	  if (fabs(AnalyticalTorques[i](j) - 
		   GenericTorques(i,j)) > 1e-8)
	    {
	      cout << testname << " Pb in link " << i << " Torque component "<< j 
		   << " Generic: " << GenericTorques(i,j) 
		   << " Analytical " << AnalyticalTorques[i](j) << endl;
	      return false;
	    }
      }
    
    
    return true;
  }
  
};
