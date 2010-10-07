/* 
Test of the operator * by L.S
*/
#include <cassert> //FIXME: maybe assert.h

#include <MatrixAbstractLayer/MatrixAbstractLayer.h>

#include "Spatial.h"
#include "Body.h"
#include "CommonTools.h"

using namespace dynamicsJRLJapan;

/* 
3.1 velocity1 * double ---> velocity
*/
int main()
{
	vector3d v1lin,v1ang;
	v1lin(0)= 0;    
	v1lin(1)= 1;    
	v1lin(2)= 3;
	v1ang(0)= 6;
	v1ang(1)= 8;
	v1ang(2)= 12;

	Spatial::Velocity v1;
	v1 = Spatial::Velocity(v1lin,v1ang);

	double a = 3;
	
	Spatial::Velocity v;
	v = a*v1;
	vector3d vlin,vang;
	vlin = v.v0();
	vang = v.w();
	assert(vlin(0) = 0);
	assert(vlin(1) = 3);
    assert(vlin(2) = 9); 
	assert(vang(0) = 18);
	assert(vang(1) = 24);
	assert(vang(2) = 36);
	std::cout << "Test has succeeded." << std::endl;
}
