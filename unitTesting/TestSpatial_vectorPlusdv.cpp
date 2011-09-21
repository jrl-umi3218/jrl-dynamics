/* 
Test of the operator + by L.S
*/

#include "Spatial.h"
#include <jrl/mal/matrixabstractlayer.hh>

#include "CommonTools.h"
#include "assertion.hh"

using namespace dynamicsJRLJapan;

/* 
1.3 vectorN + acceleration ---> acceleration
*/
void run_test()
{
	vector3d dv1lin,dv1ang;
	dv1lin(0)= 0;    
	dv1lin(1)= 1;    
	dv1lin(2)= 3;
	dv1ang(0)= 6;
	dv1ang(1)= 8;
	dv1ang(2)= 12;

	Spatial::Acceleration dv1(dv1lin,dv1ang);

	MAL_VECTOR_DIM(a, double, 6);
	a(0)= 1;    
	a(1)= 4;    
	a(2)= 6;
	a(3)= 7;
	a(4)= 9;
	a(5)= 11;

	Spatial::Acceleration dv;
	dv = a+dv1;
	vector3d dvlin,dvang;
	dvlin = dv.dv0();
	dvang = dv.dw();
	JRL_DYNAMICS_ASSERT(dvlin(0) == 1);
	JRL_DYNAMICS_ASSERT(dvlin(1) == 5);
    JRL_DYNAMICS_ASSERT(dvlin(2) == 9); 
	JRL_DYNAMICS_ASSERT(dvang(0) == 13);
	JRL_DYNAMICS_ASSERT(dvang(1) == 17);
	JRL_DYNAMICS_ASSERT(dvang(2) == 23);
	std::cout << "Test vectorPlusdv has succeeded." << std::endl;
}

GENERATE_TEST()
