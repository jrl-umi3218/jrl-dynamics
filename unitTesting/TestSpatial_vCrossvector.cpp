/* 
Test of the operator ^ by L.S
*/
#include "Spatial.h"
#include <MatrixAbstractLayer/MatrixAbstractLayer.h>

#include "CommonTools.h"
#include "assertion.hh"

using namespace dynamicsJRLJapan;

/* 
4.1 velocity ^ vectorN ---> vectorN
*/
void run_test()
{
	vectorN h;
	MAL_VECTOR_RESIZE(h,6);
	h(0)= 0;    
	h(1)= 1;    
	h(2)= 3;
	h(3)= 6;
	h(4)= 8;
	h(5)= 12;

	vector3d v1lin,v1ang;
	v1lin(0)= 1;    
	v1lin(1)= 4;    
	v1lin(2)= 6;
	v1ang(0)= 7;
	v1ang(1)= 9;
	v1ang(2)= 11;

	Spatial::Velocity v1(v1lin,v1ang);

	vectorN v;
	v = v1^h;

	std::cout << "v = " << v << std::endl;

	JRL_DYNAMICS_ASSERT(v(0) == 16);
	JRL_DYNAMICS_ASSERT(v(1) == 3);
    JRL_DYNAMICS_ASSERT(v(2) == -9); 
	JRL_DYNAMICS_ASSERT(v(3) == 20);
	JRL_DYNAMICS_ASSERT(v(4) == -18);
	JRL_DYNAMICS_ASSERT(v(5) == 2);
	std::cout << "Test vCrossvector has succeeded." << std::endl;
}

GENERATE_TEST()
