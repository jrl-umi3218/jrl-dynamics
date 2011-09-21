/* 
Test of the operator * by L.S
*/

#include "Spatial.h"
#include <jrl/mal/matrixabstractlayer.hh>

#include "CommonTools.h"
#include "assertion.hh"

using namespace dynamicsJRLJapan;

/* 
3.1 force1 * double ---> force
*/
void run_test()
{
	vector3d f1,n1;
	f1(0)= 0;    
	f1(1)= 1;    
	f1(2)= 3;
	n1(0)= 6;
	n1(1)= 8;
	n1(2)= 12;

	Spatial::Force v1(f1,n1);

	double a = 3;

	Spatial::Force v;
	v = v1*a;
	vector3d f,n0;
	f = v.f();
	n0 = v.n0();
	JRL_DYNAMICS_ASSERT(f(0) == 0);
	JRL_DYNAMICS_ASSERT(f(1) == 3);
    JRL_DYNAMICS_ASSERT(f(2) == 9); 
	JRL_DYNAMICS_ASSERT(n0(0) == 18);
	JRL_DYNAMICS_ASSERT(n0(1) == 24);
	JRL_DYNAMICS_ASSERT(n0(2) == 36);
	std::cout << "Test fBydouble has succeeded." << std::endl;
}

GENERATE_TEST()
