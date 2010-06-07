/* \file Spatial vector algebra
   Copyright (c) 2010
   Olivier Stasse
   
 */
#include "Spatial.h"

using namespace dynamicsJRLJapan::Spatial;

template<class T> void mult(T &c,
			    const T &a,
			    const T &b)
{
  // Rotation part.
  MAL_S3x3_C_eq_A_by_B(c.R, a.R, b.R);
  
  // Translation part.
  matrix3d aRT;
  MAL_S3x3_TRANSPOSE(aRT,b.R);
  MAL_S3x3_C_eq_A_by_B(c.p, aRT, a.p);
  c.p = c.p + b.p;
}

Velocity::Velocity()
{
  MAL_S3_VECTOR_FILL(m_v0,0.0);
  MAL_S3_VECTOR_FILL(m_w,0.0);
}

Velocity::Velocity(vector3d lv0,
		   vector3d lw)
  : m_v0(lv0), m_w(lw)
{
}

Velocity Velocity::operator+(Velocity &a)
{
  return Velocity(m_v0 + a.m_v0, m_w + a.m_w);
}

Velocity Velocity::operator-(Velocity &a)
{
  return Velocity(m_v0 - a.m_v0, m_w - a.m_w);
}

Acceleration::Acceleration()
{
  MAL_S3_VECTOR_FILL(m_dv0,0.0);
  MAL_S3_VECTOR_FILL(m_dw,0.0);
}

Acceleration::Acceleration(vector3d ldv0,
			   vector3d ldw)
  : m_dv0(ldv0), m_dw(ldw) 
{
}

Acceleration Acceleration::operator+(Acceleration &a)
{
  return Acceleration(m_dv0 + a.m_dv0, m_dw + a.m_dw);
}
    
Acceleration Acceleration::operator-(Acceleration &a)
{
  return Acceleration(m_dv0 - a.m_dv0, m_dw - a.m_dw);
}

cAcceleration::cAcceleration()
{
  MAL_S3_VECTOR_FILL(m_dv0,0.0);
  MAL_S3_VECTOR_FILL(m_dw,0.0);
}
    
cAcceleration::cAcceleration(vector3d ldv0,
			     vector3d ldw)
  : m_dv0(ldv0),m_dw(ldw)
{
}

cAcceleration cAcceleration::operator+(cAcceleration &a)
{
  return cAcceleration(m_dv0 + a.m_dv0, m_dw + a.m_dw);
}
    
cAcceleration cAcceleration::operator-(cAcceleration &a)
{
  return cAcceleration(m_dv0 - a.m_dv0, m_dw - a.m_dw);
}

Force::Force()
{
  MAL_S3_VECTOR_FILL(m_f,0.0);
  MAL_S3_VECTOR_FILL(m_n0,0.0);
}
    
Force::Force(vector3d lf, vector3d ln0)
  : m_f(lf), m_n0(ln0) 
{}

Force Force::operator+(Force &a)
{
  return Force(m_f + a.m_f, m_n0 + a.m_n0);
}
Force Force::operator-(Force &a)
{
  return Force(m_f - a.m_f, m_n0 - a.m_n0);
}

Motion::Motion()
{
  MAL_S3_VECTOR_FILL(m_p,0.0);
  MAL_S3_VECTOR_FILL(m_theta,0.0);
}

Motion::Motion(vector3d lp,
	       vector3d ltheta): 
  m_p(lp), m_theta(ltheta) 
{}

Motion Motion::operator+(Motion &a)
{
  return Motion(m_p + a.m_p, m_theta + a.m_theta);
}
Motion Motion::operator-(Motion &a)
{
  return Motion(m_p - a.m_p, m_theta - a.m_theta);
}

Inertia::Inertia(matrix3d lI,
		 vector3d lh,
		 double lm)
  : m_I(lI), m_h(lh),m_m(lm)
{
}

void Inertia::addInertia(Inertia &c,
			 const Inertia &a,
			 const Inertia &b) const
{
  c.m_m = a.m_m + b.m_m;
  c.m_h = a.m_h + b.m_h;
  c.m_I = a.m_I + b.m_I;
}
  
Inertia Inertia::operator+(const Inertia &a)
{
  Inertia c;
  c.m_m = a.m_m + m_m;
  c.m_h = a.m_h + m_h;
  c.m_I = a.m_I + m_I;
  return c;
}
  
Velocity Velocity::operator*(double ad)
{
  Velocity c;
  c.m_v0 = m_v0 * ad;
  c.m_w = m_w * ad;
  return c;
}

  
Velocity operator*(double ad, Velocity &a)
{
  Velocity c;
  c = a * ad;
  return c;
}

Velocity operator*(Inertia & sI, Velocity &v)
{
  Velocity c;
  vector3d NE_tmp,NE_tmp2;
  // Angular velocity
  MAL_S3x3_C_eq_A_by_B(NE_tmp , sI.I(), v.w());
  MAL_S3_VECTOR_CROSS_PRODUCT(NE_tmp2, sI.h(), v.v0());
  c.w(NE_tmp2 + NE_tmp);
      
  // Linear velocity
  NE_tmp = v.v0() * sI.m();
  MAL_S3_VECTOR_CROSS_PRODUCT(NE_tmp2, sI.h(), v.w());
  c.v0(NE_tmp2 + NE_tmp);
      
  return c;
}

  
/*! Plucker Transform */
PluckerTransform::PluckerTransform()
{};

PluckerTransform::PluckerTransform(matrix3d lR,
				   vector3d lp):
  m_R(lR), m_p(lp) {}
  
PluckerTransform  PluckerTransform::operator*(const PluckerTransform &a)
{
  PluckerTransform c;
  // Rotation
  MAL_S3x3_C_eq_A_by_B(c.R, R, a.R);
  // position
  matrix3d aRT;
  MAL_S3x3_TRANSPOSE(aRT,a.R);
  c.p = p + aRT * a.p;
}

Velocity  operator*( PluckerTransform &X, const Velocity &v)
{
  Velocity c;
  // Computes the angular velocity
  vector3d NE_tmp,NE_tmp2;
  MAL_S3_VECTOR_CROSS_PRODUCT(NE_tmp,X.p,v.w);
  NE_tmp2=v.v0-NE_tmp;
    
  MAL_S3x3_C_eq_A_by_B(c.w,X.R,NE_tmp2);
    
  // Computes the linear velocity
  MAL_S3x3_C_eq_A_by_B(c.v,X.R,v.w);
  return c;
}
  
Force  PluckerTransform::operator*( const Force &f)
{
  Force c;
  // Computes the angular velocity
  vector3d NE_tmp,NE_tmp2;
  MAL_S3_VECTOR_CROSS_PRODUCT(NE_tmp,p,f.f);
  NE_tmp2=f.n0-NE_tmp;
    
  MAL_S3x3_C_eq_A_by_B(c.n0,R,NE_tmp2);
    
  // Computes the linear velocity
  MAL_S3x3_C_eq_A_by_B(c.f,R,f.f);
  return c;
};

void PluckerTransform::inverse(const PluckerTransform &a)
{
  MAL_S3x3_TRANSPOSE(R,a.R);
  MAL_S3x3_C_eq_A_by_B(p,a.R,-a.p);
}
