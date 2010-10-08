/* 
Test of the operator * by L.S
*/

#include "Spatial.h"
#include <MatrixAbstractLayer/MatrixAbstractLayer.h>

#include "CommonTools.h"
#include "assertion.hh"

using namespace dynamicsJRLJapan;

/* 
2.2 inertia1 * acceleration ---> force
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

	matrix3d lI1;
	vector3d lh1;
	double lm1;

	{
	lI1(0,0)=0.5;
	lI1(1,0)=-0.1;
	lI1(2,0)=2;

	lI1(0,1)=-1.5;
	lI1(1,1)=0.1;
	lI1(2,1)=-0.05;

	lI1(0,2)=2;
	lI1(1,2)=-0.08;
	lI1(2,2)=0.5;
	}

	lh1(0)= 0.5;    
	lh1(1)= 1;    
	lh1(2)= 2.5;

	lm1 = 0.1;

	Spatial::Inertia I1(lI1,lh1,lm1);

	Spatial::Force f;
	f = I1*dv1;
	vector3d f0,n0;
	f0 = f.f();
	n0 = f.n0();

	JRL_DYNAMICS_ASSERT(f0(0) == 8);
	JRL_DYNAMICS_ASSERT(f0(1) == -8.9);
    JRL_DYNAMICS_ASSERT(f0(2) == 2.3); 
	JRL_DYNAMICS_ASSERT(n0(0) == 15.5);
	JRL_DYNAMICS_ASSERT(n0(1) == -2.26);
	JRL_DYNAMICS_ASSERT(n0(2) == 18.1);
	std::cout << "Test iBydv has succeeded." << std::endl;
}

GENERATE_TEST()