/* 
Test of the operator + by L.S
*/


#include "Spatial.h"
#include <MatrixAbstractLayer/MatrixAbstractLayer.h>

#include "CommonTools.h"
#include "assertion.hh"

using namespace dynamicsJRLJapan;

/* 
1.1 force1 + force2 ---> force
*/
void run_test()
{
	vector3d f1,f2,n1,n2;
	f1(0)= 0;    
	f1(1)= 1;    
	f1(2)= 3;
	n1(0)= 6;
	n1(1)= 8;
	n1(2)= 12;

	Spatial::Force v1(f1,n1);

	f2(0)= 1;    
	f2(1)= 4;    
	f2(2)= 6;
	n2(0)= 7;
	n2(1)= 9;
	n2(2)= 11;

	Spatial::Force v2(f2,n2);

	Spatial::Force v;
	v = v1+v2;
	vector3d f,n0;
	f = v.f();
	n0 = v.n0();
	JRL_DYNAMICS_ASSERT(f(0) == 1);
	JRL_DYNAMICS_ASSERT(f(1) == 5);
    JRL_DYNAMICS_ASSERT(f(2) == 9); 
	JRL_DYNAMICS_ASSERT(n0(0) == 13);
	JRL_DYNAMICS_ASSERT(n0(1) == 17);
	JRL_DYNAMICS_ASSERT(n0(2) == 23);
	std::cout << "Test fPlusf has succeeded." << std::endl;
}

GENERATE_TEST()
