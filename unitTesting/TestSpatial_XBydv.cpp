/* 
Test of the operator * by L.S
*/

#include "Spatial.h"
#include <jrl/mal/matrixabstractlayer.hh>

#include "CommonTools.h"
#include "assertion.hh"

using namespace dynamicsJRLJapan;

/* 
1.3 pluckertransform * acceleration ---> acceleration
*/
void run_test()
{
	vector3d p1,dv1lin,dv1ang;
	p1(0)= 0.5;    
	p1(1)= 1;    
	p1(2)= 2.5;
	dv1lin(0)= 0;    
	dv1lin(1)= 1;    
	dv1lin(2)= 3;
	dv1ang(0)= 6;
	dv1ang(1)= 8;
	dv1ang(2)= 12;

	Spatial::Acceleration dv1(dv1lin,dv1ang);

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

	Spatial::Acceleration dv;
	dv = X1*dv1;
	vector3d dvlin,dvang;
	dvlin = dv.dv0();
	dvang = dv.dw();

	JRL_DYNAMICS_ASSERT(dvlin(0) == 26);
	JRL_DYNAMICS_ASSERT(dvlin(1) == -2);
    JRL_DYNAMICS_ASSERT(dvlin(2) == 18.9);
	JRL_DYNAMICS_ASSERT(dvang(0) == 15);
	JRL_DYNAMICS_ASSERT(dvang(1) == -0.76);
	JRL_DYNAMICS_ASSERT(dvang(2) == 17.6);
	std::cout << "Test XBydv has succeeded." << std::endl;
}

GENERATE_TEST()
