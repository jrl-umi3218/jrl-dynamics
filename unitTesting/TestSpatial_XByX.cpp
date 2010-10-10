/* 
Test of the operator * by L.S
*/

#include "Spatial.h"
#include <MatrixAbstractLayer/MatrixAbstractLayer.h>

#include "CommonTools.h"
#include "assertion.hh"

using namespace dynamicsJRLJapan;

/* 
1.1 pluckertransform1 * pluckertransform2 ---> pluckertransform
*/
void run_test()
{
	double Error_Tolerated = 0.0000001;
	vector3d p1,p2;
	p1(0)= 0.5;    
	p1(1)= 1;    
	p1(2)= 2.5;
	p2(0)= 2;
	p2(1)= 3.5;
	p2(2)= 5;

	matrix3d R1,R2;
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

	{
	R2(0,0)=0.15;
	R2(1,0)=-0.05;
	R2(2,0)=0;

	R2(0,1)=-0.05;
	R2(1,1)=0.3;
	R2(2,1)=0;

	R2(0,2)=0;
	R2(1,2)=0;
	R2(2,2)=0.4;
	}

	Spatial::PluckerTransform X1(R1,p1);
	Spatial::PluckerTransform X2(R2,p2);

	Spatial::PluckerTransform X;
	X = X1*X2;
	matrix3d R;
	vector3d p;
	R = X.R();
	p = X.p();

	std::cout << "R = " << R << std::endl;
	std::cout << "p = " << p << std::endl;

	std::cout << "Rerr0 = " << (R(0,0) - 0.15) << std::endl;
	std::cout << "Rerr1 = " << (R(1,0) + 0.02) << std::endl;
	std::cout << "Rerr2 = " << (R(2,0) - 0.3025) << std::endl;
	std::cout << "Rerr3 = " << (R(0,1) + 0.475) << std::endl;
	std::cout << "Rerr4 = " << (R(1,1) - 0.035) << std::endl;
	std::cout << "Rerr5 = " << (R(2,1) + 0.115) << std::endl;
	std::cout << "Rerr3 = " << (R(0,2) - 0.8) << std::endl;
	std::cout << "Rerr4 = " << (R(1,2) + 0.032) << std::endl;
	std::cout << "Rerr5 = " << (R(2,2) - 0.2) << std::endl;

	JRL_DYNAMICS_ASSERT(R(0,0) - 0.15 <= Error_Tolerated);
	JRL_DYNAMICS_ASSERT(R(1,0) + 0.02 <= Error_Tolerated);
	JRL_DYNAMICS_ASSERT(R(2,0) - 0.3025 <= Error_Tolerated);

	JRL_DYNAMICS_ASSERT(R(0,1) + 0.475 <= Error_Tolerated);
	JRL_DYNAMICS_ASSERT(R(1,1) - 0.035 <= Error_Tolerated);
	JRL_DYNAMICS_ASSERT(R(2,1) + 0.115 <= Error_Tolerated);

	JRL_DYNAMICS_ASSERT(R(0,2) - 0.8 <= Error_Tolerated);
	JRL_DYNAMICS_ASSERT(R(1,2) + 0.032 <= Error_Tolerated);
	JRL_DYNAMICS_ASSERT(R(2,2) - 0.2 <= Error_Tolerated);

	std::cout << "perr0 = " << (p(0) - 2.025) << std::endl;
	std::cout << "perr1 = " << (p(1) - 3.775) << std::endl;
	std::cout << "perr2 = " << (p(2) - 6) << std::endl;

	JRL_DYNAMICS_ASSERT(p(0) - 2.025 <= Error_Tolerated);
	JRL_DYNAMICS_ASSERT(p(1) - 3.775 <= Error_Tolerated);
	JRL_DYNAMICS_ASSERT(p(2) - 6 <= Error_Tolerated);
	std::cout << "Test XByX has succeeded." << std::endl;
}

GENERATE_TEST()