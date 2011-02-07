/* 
Test of the operator * by L.S
*/

#include "Spatial.h"
#include <jrl/mal/matrixabstractlayer.hh>

#include "CommonTools.h"
#include "assertion.hh"

using namespace dynamicsJRLJapan;

/* 
1.2 pluckertransformtranspose * force ---> force
*/
void run_test()
{
	double Error_Tolerated = 0.0000001;
	vector3d p1,f1,n1;
	p1(0)= 0.5;    
	p1(1)= 1;    
	p1(2)= 2.5;
	f1(0)= 0;    
	f1(1)= 1;    
	f1(2)= 3;
	n1(0)= 6;
	n1(1)= 8;
	n1(2)= 12;

	Spatial::Force F1(f1,n1);

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

	Spatial::PluckerTransformTranspose X1(R1,p1);

	Spatial::Force F;
	F = X1*F1;
	vector3d f,n0;
	f = F.f();
	n0 = F.n0();

	std::cout << "err0 = " << (f(0) - 5.9) << std::endl;
	std::cout << "err1 = " << (f(1) + 0.05) << std::endl;
	std::cout << "err2 = " << (f(2) - 1.42) << std::endl;
	std::cout << "err3 = " << (n0(0) - 27.745) << std::endl;
	std::cout << "err4 = " << (n0(1) - 5.24) << std::endl;
	std::cout << "err5 = " << (n0(2) - 11.435) << std::endl;

	JRL_DYNAMICS_ASSERT(f(0) - 5.9 <= Error_Tolerated);
	JRL_DYNAMICS_ASSERT(f(1) + 0.05 <= Error_Tolerated);
    JRL_DYNAMICS_ASSERT(f(2) - 1.42 <= Error_Tolerated);
	JRL_DYNAMICS_ASSERT(n0(0) - 27.745 <= Error_Tolerated);
	JRL_DYNAMICS_ASSERT(n0(1) - 5.24 <= Error_Tolerated);
	JRL_DYNAMICS_ASSERT(n0(2) - 11.435 <= Error_Tolerated);
	std::cout << "Test XTByf has succeeded." << std::endl;
}

GENERATE_TEST()
