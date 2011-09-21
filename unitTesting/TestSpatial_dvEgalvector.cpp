/* 
Test of the operator = by L.S
*/

#include "Spatial.h"
#include <jrl/mal/matrixabstractlayer.hh>

#include "CommonTools.h"
#include "assertion.hh"

using namespace dynamicsJRLJapan;

/* 
3.1 acceleration = vectorN ---> acceleration (this)
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

	Spatial::Acceleration dv;
	dv = a;
	vector3d dvlin,dvang;
	dvlin = dv.dv0();
	dvang = dv.dw();
	JRL_DYNAMICS_ASSERT(dvlin(0) == 1);
	JRL_DYNAMICS_ASSERT(dvlin(1) == 4);
    JRL_DYNAMICS_ASSERT(dvlin(2) == 6); 
	JRL_DYNAMICS_ASSERT(dvang(0) == 7);
	JRL_DYNAMICS_ASSERT(dvang(1) == 9);
	JRL_DYNAMICS_ASSERT(dvang(2) == 11);
	std::cout << "Test dvEgalvector has succeeded." << std::endl;
}

GENERATE_TEST()
