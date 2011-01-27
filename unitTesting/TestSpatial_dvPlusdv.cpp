/* 
Test of the operator + by L.S
*/

#include "Spatial.h"
#include <MatrixAbstractLayer/MatrixAbstractLayer.h>

#include "CommonTools.h"
#include "assertion.hh"

using namespace dynamicsJRLJapan;

/* 
1.1 acceleration1 + acceleration2 ---> acceleration
*/
void run_test()
{
	vector3d dv1lin,dv2lin,dv1ang,dv2ang;
	dv1lin(0)= 0;    
	dv1lin(1)= 1;    
	dv1lin(2)= 3;
	dv1ang(0)= 6;
	dv1ang(1)= 8;
	dv1ang(2)= 12;

	Spatial::Acceleration dv1(dv1lin,dv1ang);

	dv2lin(0)= 1;    
	dv2lin(1)= 4;    
	dv2lin(2)= 6;
	dv2ang(0)= 7;
	dv2ang(1)= 9;
	dv2ang(2)= 11;

	Spatial::Acceleration dv2(dv2lin,dv2ang);

	Spatial::Acceleration dv;
	dv = dv1+dv2;
	vector3d dvlin,dvang;
	dvlin = dv.dv0();
	dvang = dv.dw();

	JRL_DYNAMICS_ASSERT(dvlin(0) == 1);
	JRL_DYNAMICS_ASSERT(dvlin(1) == 5);
    JRL_DYNAMICS_ASSERT(dvlin(2) == 9); 
	JRL_DYNAMICS_ASSERT(dvang(0) == 13);
	JRL_DYNAMICS_ASSERT(dvang(1) == 17);
	JRL_DYNAMICS_ASSERT(dvang(2) == 23);
	std::cout << "Test dvPlusdv has succeeded." << std::endl;
}

GENERATE_TEST()