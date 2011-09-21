/* 
Test of the operator * by L.S
*/

#include "Spatial.h"
#include <jrl/mal/matrixabstractlayer.hh>

#include "CommonTools.h"
#include "assertion.hh"

using namespace dynamicsJRLJapan;

/* 
1.2 pluckertransform1 * velocity ---> velocity
*/
void run_test()
{
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

	Spatial::PluckerTransform X1(R1,p1);

	Spatial::Velocity v;
	v = X1*v1;
	vector3d vlin,vang;
	vlin = v.v0();
	vang = v.w();

	JRL_DYNAMICS_ASSERT(vlin(0) == 26);
	JRL_DYNAMICS_ASSERT(vlin(1) == -2);
    JRL_DYNAMICS_ASSERT(vlin(2) == 18.9);
	JRL_DYNAMICS_ASSERT(vang(0) == 15);
	JRL_DYNAMICS_ASSERT(vang(1) == -0.76);
	JRL_DYNAMICS_ASSERT(vang(2) == 17.6);
	std::cout << "Test XByv has succeeded." << std::endl;
}

GENERATE_TEST()
