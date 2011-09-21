/* 
Test of the operator = by L.S
*/

#include "Spatial.h"
#include <jrl/mal/matrixabstractlayer.hh>

#include "CommonTools.h"
#include "assertion.hh"

using namespace dynamicsJRLJapan;

/* 
5.1 velocity = vectorN ---> velocity (this)
*/
void run_test()
{
	MAL_VECTOR_DIM(a, double, 6);
	
	a(0)= 1;    
	a(1)= 4;    
	a(2)= 6;
	a(3)= 7;
	a(4)= 9;
	a(5)= 11;

	Spatial::Velocity v;
	v = a;
	vector3d vlin,vang;
	vlin = v.v0();
	vang = v.w();
	JRL_DYNAMICS_ASSERT(vlin(0) == 1);
	JRL_DYNAMICS_ASSERT(vlin(1) == 4);
    JRL_DYNAMICS_ASSERT(vlin(2) == 6); 
	JRL_DYNAMICS_ASSERT(vang(0) == 7);
	JRL_DYNAMICS_ASSERT(vang(1) == 9);
	JRL_DYNAMICS_ASSERT(vang(2) == 11);
	std::cout << "Test vEgalvector has succeeded." << std::endl;
}

GENERATE_TEST()
