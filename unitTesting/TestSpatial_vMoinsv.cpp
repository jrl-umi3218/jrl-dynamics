/* 
Test of the operator - by L.S
*/

#include "Spatial.h"
#include <jrl/mal/matrixabstractlayer.hh>

#include "CommonTools.h"
#include "assertion.hh"

using namespace dynamicsJRLJapan;

/* 
2.1 velocity1 - velocity2 ---> velocity
*/
void run_test()
{
	vector3d v1lin,v2lin,v1ang,v2ang;
	v1lin(0)= 0;    
	v1lin(1)= 1;    
	v1lin(2)= 3;
	v1ang(0)= 6;
	v1ang(1)= 8;
	v1ang(2)= 12;

	Spatial::Velocity v1(v1lin,v1ang);

	v2lin(0)= 1;    
	v2lin(1)= 4;    
	v2lin(2)= 6;
	v2ang(0)= 7;
	v2ang(1)= 9;
	v2ang(2)= 11;

	Spatial::Velocity v2(v2lin,v2ang);

	Spatial::Velocity v;
	v = v2-v1;
	vector3d vlin,vang;
	vlin = v.v0();
	vang = v.w();
	JRL_DYNAMICS_ASSERT(vlin(0) == 1);
	JRL_DYNAMICS_ASSERT(vlin(1) == 3);
    JRL_DYNAMICS_ASSERT(vlin(2) == 3); 
	JRL_DYNAMICS_ASSERT(vang(0) == 1);
	JRL_DYNAMICS_ASSERT(vang(1) == 1);
	JRL_DYNAMICS_ASSERT(vang(2) == -1);
	std::cout << "Test vMoinsv has succeeded." << std::endl;
}

GENERATE_TEST()
