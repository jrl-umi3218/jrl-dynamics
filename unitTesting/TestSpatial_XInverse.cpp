/* 
Test of the method inverse by L.S
*/

#include "Spatial.h"
#include <jrl/mal/matrixabstractlayer.hh>

#include "CommonTools.h"
#include "assertion.hh"

using namespace dynamicsJRLJapan;

/* 
3.1 inverse(pluckertransform) ---> void
*/
void run_test()
{
	double Error_Tolerated = 0.0000001;
	vector3d p1;
	p1(0)= 0.5;    
	p1(1)= 1;    
	p1(2)= 2.5;

	matrix3d R1;
	{
	R1(0,0)=0.5;
	R1(1,0)=-0.1;
	R1(2,0)=2;

	R1(0,1)=-1.5;
	R1(1,1)=0.1;
	R1(2,1)=-0.05;

	R1(0,2)=2;
	R1(1,2)=-0.08;
	R1(2,2)=0.5;
	}

	Spatial::PluckerTransform X1(R1,p1);

	Spatial::PluckerTransform X;
	X.inverse(X1);
	matrix3d R;
	vector3d p;
	R = X.R();
	p = X.p();

	JRL_DYNAMICS_ASSERT(R(0,0) - 0.5 <= Error_Tolerated);
	JRL_DYNAMICS_ASSERT(R(1,0) + 1.5 <= Error_Tolerated);
	JRL_DYNAMICS_ASSERT(R(2,0) - 2 <= Error_Tolerated);

	JRL_DYNAMICS_ASSERT(R(0,1) + 0.1 <= Error_Tolerated);
	JRL_DYNAMICS_ASSERT(R(1,1) - 0.1 <= Error_Tolerated);
	JRL_DYNAMICS_ASSERT(R(2,1) + 0.08 <= Error_Tolerated);

	JRL_DYNAMICS_ASSERT(R(0,2) - 2 <= Error_Tolerated);
	JRL_DYNAMICS_ASSERT(R(1,2) + 0.05 <= Error_Tolerated);
	JRL_DYNAMICS_ASSERT(R(2,2) - 0.5 <= Error_Tolerated);

	std::cout << "err0 = " << (p(0) + 3.75) << std::endl;
	std::cout << "err1 = " << (p(1) - 0.15) << std::endl;
	std::cout << "err2 = " << (p(2) + 2.2) << std::endl;

	JRL_DYNAMICS_ASSERT(p(0) + 3.75 <= Error_Tolerated);
	JRL_DYNAMICS_ASSERT(p(1) - 0.15 <= Error_Tolerated);
	JRL_DYNAMICS_ASSERT(p(2) + 2.2 <= Error_Tolerated);
	std::cout << "Test XInverse has succeeded." << std::endl;
}

GENERATE_TEST()
