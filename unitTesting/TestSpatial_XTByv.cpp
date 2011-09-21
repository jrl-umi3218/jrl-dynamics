/* 
Test of the operator * by L.S
*/

#include "Spatial.h"
#include <jrl/mal/matrixabstractlayer.hh>

#include "CommonTools.h"
#include "assertion.hh"

using namespace dynamicsJRLJapan;

/* 
1.1 pluckertransformtranspose * velocity ---> velocity
*/
void run_test()
{
	double Error_Tolerated = 0.0000001;
	vector3d p1,v1lin,v1ang;
	p1(0)= 0.5;    
	p1(1)= 1;    
	p1(2)= 2.5;
	v1lin(0)= 0;    
	v1lin(1)= 1;    
	v1lin(2)= 3;
	v1ang(0)= 6;
	v1ang(1)= 8;
	v1ang(2)= 12;

	Spatial::Velocity v1(v1lin,v1ang);

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

	Spatial::Velocity v;
	v = X1*v1;
	vector3d vlin,vang;
	vlin = v.v0();
	vang = v.w();
    
	std::cout << "err0 = " << (vlin(0) - 45.26) << std::endl;
	std::cout << "err1 = " << (vlin(1) - 56.77) << std::endl;
	std::cout << "err2 = " << (vlin(2) + 29.18) << std::endl;
	std::cout << "err3 = " << (vang(0) - 26.2) << std::endl;
	std::cout << "err4 = " << (vang(1) + 8.8) << std::endl;
	std::cout << "err5 = " << (vang(2) - 17.36) << std::endl;

	JRL_DYNAMICS_ASSERT(vlin(0) - 45.26 <= Error_Tolerated);
	JRL_DYNAMICS_ASSERT(vlin(1) - 56.77 <= Error_Tolerated);
    JRL_DYNAMICS_ASSERT(vlin(2) + 29.18 <= Error_Tolerated);
	JRL_DYNAMICS_ASSERT(vang(0) - 26.2 <= Error_Tolerated);
	JRL_DYNAMICS_ASSERT(vang(1) + 8.8 <= Error_Tolerated);
	JRL_DYNAMICS_ASSERT(vang(2) - 17.36 <= Error_Tolerated);
	std::cout << "Test XTByv has succeeded." << std::endl;
}

GENERATE_TEST()
