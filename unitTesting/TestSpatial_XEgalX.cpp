/* 
Test of the operator = by L.S
*/

#include "Spatial.h"
#include <MatrixAbstractLayer/MatrixAbstractLayer.h>

#include "CommonTools.h"
#include "assertion.hh"

using namespace dynamicsJRLJapan;

/* 
2.1 pluckertransform1 = pluckertransform2 (this)
*/
void run_test()
{
	vector3d p1;
	p1(0)= 0.5;    
	p1(1)= 1;    
	p1(2)= 2.5;

	matrix3d R1;
	{
	R1(0,0)=0.5;
	R1(1,0)=-0.1;
	R1(2,0)=2;

	R1(0,1)=-1.5;
	R1(1,1)=0.1;
	R1(2,1)=-0.05;

	R1(0,2)=2;
	R1(1,2)=-0.08;
	R1(2,2)=0.5;
	}

	Spatial::PluckerTransform X1(R1,p1);

	Spatial::PluckerTransform X;
	X = X1;
	matrix3d R;
	vector3d p;
	R = X.R();
	p = X.p();

	JRL_DYNAMICS_ASSERT(R(0,0) == 0.5);
	JRL_DYNAMICS_ASSERT(R(1,0) == -0.1);
	JRL_DYNAMICS_ASSERT(R(2,0) == 2);

	JRL_DYNAMICS_ASSERT(R(0,1) == -1.5);
	JRL_DYNAMICS_ASSERT(R(1,1) == 0.1);
	JRL_DYNAMICS_ASSERT(R(2,1) == -0.05);

	JRL_DYNAMICS_ASSERT(R(0,2) == 2);
	JRL_DYNAMICS_ASSERT(R(1,2) == -0.08);
	JRL_DYNAMICS_ASSERT(R(2,2) == 0.5);

	
	JRL_DYNAMICS_ASSERT(p(0) == 0.5);
	JRL_DYNAMICS_ASSERT(p(1) == 1);
	JRL_DYNAMICS_ASSERT(p(2) == 2.5);
	std::cout << "Test XEgalX has succeeded." << std::endl;
}

GENERATE_TEST()