/* 
Test of the operator + by L.S
*/


#include "Spatial.h"
#include <MatrixAbstractLayer/MatrixAbstractLayer.h>

#include "CommonTools.h"
#include "assertion.hh"

using namespace dynamicsJRLJapan;

/* 
1.1 inertia1 + inertia2 ---> inertia
*/
void run_test()
{
	matrix3d lI1,lI2;
	vector3d lh1,lh2;
	double lm1,lm2;

	{
	lI1(0,0)=0.5;
	lI1(1,0)=-0.1;
	lI1(2,0)=2;

	lI1(0,1)=-1.5;
	lI1(1,1)=0.1;
	lI1(2,1)=-0.05;

	lI1(0,2)=2;
	lI1(1,2)=-0.08;
	lI1(2,2)=0.5;
	}


	{
	lI2(0,0)=0.15;
	lI2(1,0)=-0.05;
	lI2(2,0)=0;

	lI2(0,1)=-0.05;
	lI2(1,1)=0.3;
	lI2(2,1)=0;

	lI2(0,2)=0;
	lI2(1,2)=0;
	lI2(2,2)=0.4;
	}

	lh1(0)= 0.5;    
	lh1(1)= 1;    
	lh1(2)= 2.5;
	lh2(0)= 2;
	lh2(1)= 3.5;
	lh2(2)= 5;

	lm1 = 0.1;
	lm2 = 0.2;
	Spatial::Inertia I1(lI1,lh1,lm1);

	Spatial::Inertia I2(lI2,lh2,lm2);

	Spatial::Inertia I;
	I = I1+I2;
	matrix3d lI;
	vector3d lh;
	double lm;

	lI = I.I();
	lh = I.h();
	lm = I.m();

	std::cout << "lI = " << lI << std::endl;
	std::cout << "lh = " << lh << std::endl;
	std::cout << "lm = " << lm << std::endl;

	JRL_DYNAMICS_ASSERT(lI(0,0) == 0.65);
	JRL_DYNAMICS_ASSERT(lI(1,0) == -0.15);
	JRL_DYNAMICS_ASSERT(lI(2,0) == 2);

	JRL_DYNAMICS_ASSERT(lI(0,1) == -1.55);
	JRL_DYNAMICS_ASSERT(lI(1,1) == 0.4);
	JRL_DYNAMICS_ASSERT(lI(2,1) == -0.05);

	JRL_DYNAMICS_ASSERT(lI(0,2) == 2);
	JRL_DYNAMICS_ASSERT(lI(1,2) == -0.08);
	JRL_DYNAMICS_ASSERT(lI(2,2) == 0.9);

	JRL_DYNAMICS_ASSERT(lh(0) == 2.5);
	JRL_DYNAMICS_ASSERT(lh(1) == 4.5);
	JRL_DYNAMICS_ASSERT(lh(2) == 7.5);

	JRL_DYNAMICS_ASSERT(lm == 0.3);
	std::cout << "Test iPlusi has succeeded." << std::endl;
}

GENERATE_TEST()
