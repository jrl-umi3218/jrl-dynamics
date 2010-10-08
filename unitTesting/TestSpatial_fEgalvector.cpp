/* 
Test of the operator = by L.S
*/

#include "Spatial.h"
#include <MatrixAbstractLayer/MatrixAbstractLayer.h>

#include "CommonTools.h"
#include "assertion.hh"

using namespace dynamicsJRLJapan;

/* 
4.1 force = vectorN ---> force (this)
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

	Spatial::Force v;
	v = a;
	vector3d f,n0;
	f = v.f();
	n0 = v.n0();
	JRL_DYNAMICS_ASSERT(f(0) == 1);
	JRL_DYNAMICS_ASSERT(f(1) == 4);
    JRL_DYNAMICS_ASSERT(f(2) == 6); 
	JRL_DYNAMICS_ASSERT(n0(0) == 7);
	JRL_DYNAMICS_ASSERT(n0(1) == 9);
	JRL_DYNAMICS_ASSERT(n0(2) == 11);
	std::cout << "Test fEgalvector has succeeded." << std::endl;
}

GENERATE_TEST()