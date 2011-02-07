/* 
Test of the operator * by L.S
*/

#include "Spatial.h"
#include <jrl/mal/matrixabstractlayer.hh>

#include "CommonTools.h"
#include "assertion.hh"

using namespace dynamicsJRLJapan;

/* 
2.1 inertia1 * velocity ---> momentum
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

	Spatial::Momentum m;
	m = I1*v1;
	vector3d mv,mw;
	mv = m.v();
	mw = m.w();

	JRL_DYNAMICS_ASSERT(mv(0) == 8);
	JRL_DYNAMICS_ASSERT(mv(1) == -8.9);
    JRL_DYNAMICS_ASSERT(mv(2) == 2.3); 
	JRL_DYNAMICS_ASSERT(mw(0) == 15.5);
	JRL_DYNAMICS_ASSERT(mw(1) == -2.26);
	JRL_DYNAMICS_ASSERT(mw(2) == 18.1);
	std::cout << "Test iByv has succeeded." << std::endl;
}

GENERATE_TEST()
