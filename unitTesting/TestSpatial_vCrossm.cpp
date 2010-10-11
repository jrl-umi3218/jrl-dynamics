/* 
Test of the operator ^ by L.S
*/
#include "Spatial.h"
#include <MatrixAbstractLayer/MatrixAbstractLayer.h>

#include "CommonTools.h"
#include "assertion.hh"

using namespace dynamicsJRLJapan;

/* 
4.2 velocity ^ momentum ---> Force
*/
void run_test()
{
	vector3d hlin,hang;
	hlin(0)= 0;    
	hlin(1)= 1;    
	hlin(2)= 3;
	hang(0)= 6;
	hang(1)= 8;
	hang(2)= 12;

	

	vector3d vlin,vang;
	vlin(0)= 1;    
	vlin(1)= 4;    
	vlin(2)= 6;
	vang(0)= 7;
	vang(1)= 9;
	vang(2)= 11;

	Spatial::Velocity v(vlin,vang);

	Spatial::Momentum h(hlin,hang);
	Spatial::Force f;
	f = v^h;
	vector3d flin,fang;
	flin = f.f();
	fang = f.n0();

	JRL_DYNAMICS_ASSERT(flin(0) == 16);
	JRL_DYNAMICS_ASSERT(flin(1) == -21);
    JRL_DYNAMICS_ASSERT(flin(2) == 7); 
	JRL_DYNAMICS_ASSERT(fang(0) == 26);
	JRL_DYNAMICS_ASSERT(fang(1) == -21);
	JRL_DYNAMICS_ASSERT(fang(2) == 3);
	std::cout << "Test vCrossm has succeeded." << std::endl;
}

GENERATE_TEST()