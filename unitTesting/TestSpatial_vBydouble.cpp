/* 
Test of the operator * by L.S
*/

#include "Spatial.h"
#include <jrl/mal/matrixabstractlayer.hh>

#include "CommonTools.h"
#include "assertion.hh"

using namespace dynamicsJRLJapan;

/* 
3.1 velocity1 * double ---> velocity
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

	double a = 3;

	Spatial::Velocity v;
	v = v1*a;
	vector3d vlin,vang;
	vlin = v.v0();
	vang = v.w();
	JRL_DYNAMICS_ASSERT(vlin(0) == 0);
	JRL_DYNAMICS_ASSERT(vlin(1) == 3);
    JRL_DYNAMICS_ASSERT(vlin(2) == 9); 
	JRL_DYNAMICS_ASSERT(vang(0) == 18);
	JRL_DYNAMICS_ASSERT(vang(1) == 24);
	JRL_DYNAMICS_ASSERT(vang(2) == 36);
	std::cout << "Test vBydouble has succeeded." << std::endl;
}

GENERATE_TEST()
