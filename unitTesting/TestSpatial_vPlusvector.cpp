/* 
Test of the operator + by L.S
*/

#include "Spatial.h"
#include <jrl/mal/matrixabstractlayer.hh>

#include "CommonTools.h"
#include "assertion.hh"

using namespace dynamicsJRLJapan;

/* 
1.2 velocity1 + vectorN ---> velocity
*/
void run_test()
{
	vector3d v1lin,v1ang;
	v1lin(0)= 0;    
	v1lin(1)= 1;    
	v1lin(2)= 3;
	v1ang(0)= 6;
	v1ang(1)= 8;
	v1ang(2)= 12;

	Spatial::Velocity v1(v1lin,v1ang);
	MAL_VECTOR_DIM(a, double, 6);
	
	a(0)= 1;    
	a(1)= 4;    
	a(2)= 6;
	a(3)= 7;
	a(4)= 9;
	a(5)= 11;

	Spatial::Velocity v;
	v = v1+a;
	vector3d vlin,vang;
	vlin = v.v0();
	vang = v.w();
	JRL_DYNAMICS_ASSERT(vlin(0) == 1);
	JRL_DYNAMICS_ASSERT(vlin(1) == 5);
    JRL_DYNAMICS_ASSERT(vlin(2) == 9); 
	JRL_DYNAMICS_ASSERT(vang(0) == 13);
	JRL_DYNAMICS_ASSERT(vang(1) == 17);
	JRL_DYNAMICS_ASSERT(vang(2) == 23);
	std::cout << "Test vPlusvector has succeeded." << std::endl;
}

GENERATE_TEST()
