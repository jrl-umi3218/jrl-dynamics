/* \file Spatial vector algebra
   Copyright (c) 2010
   Olivier Stasse
   
*/

//FXME: verifier les auto affectations.
//if (&a == this) return this;

#include "Spatial.h"

using namespace dynamicsJRLJapan::Spatial;


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

Velocity Velocity::operator+(vectorN &a)
{
  Velocity av;
  if (MAL_VECTOR_SIZE(a)==6)
    {
      for(unsigned int i=0;i<3;i++)
	av.m_v0(i) = m_v0(i) + a(i);

      for(unsigned int i=0;i<3;i++)
	av.m_w(i) = m_w(i) + a(i+3);
    }
  return av;
}

Velocity operator+(vectorN &a, Velocity &b)
{
  Velocity c;
  if (MAL_VECTOR_SIZE(a)==6)
    {
      vector3d bv0 = b.v0();
      vector3d bw = b.w();
      vector3d cv0, cw;
      for(unsigned int i=0;i<3;i++)
	cv0(i) = bv0(i) + a(i);

      c.v0(cv0);
      for(unsigned int i=0;i<3;i++)
	cw(i) = bw(i) + a(i+3);
      
      c.w(cw);
    }
  return c;
}

//----------> Adding the operator = to associate 2 equivalent quantities by L.S
Velocity* Velocity::operator=(const vectorN &a)
{
  if (MAL_VECTOR_SIZE(a)==6)
    {
      for(unsigned int i=0;i<3;i++)
	m_v0(i) = a(i);

      for(unsigned int i=0;i<3;i++)
	m_w(i) = a(i+3);
    }
  return this;
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

Acceleration Acceleration::operator+(vectorN &a)
{
  Acceleration av;
  if (MAL_VECTOR_SIZE(a)==6)
    {
      for(unsigned int i=0;i<3;i++)
	av.m_dv0(i) = m_dv0(i) + a(i);

      for(unsigned int i=0;i<3;i++)
	av.m_dw(i) = m_dw(i) + a(i+3);
    }
  return av;
}

//----------> Adding the operator = to associate 2 equivalent quantities by L.S
Acceleration* Acceleration::operator=(vectorN &a)
{
  if (MAL_VECTOR_SIZE(a)==6)
    {
      for(unsigned int i=0;i<3;i++)
	m_dv0(i) = a(i);

      for(unsigned int i=0;i<3;i++)
	m_dw(i) = a(i+3);
    }
  return this;
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

//----------> Adding the operator = to associate 2 equivalent quantities by L.S
Force* Force::operator=(vectorN &a)
{
  if (MAL_VECTOR_SIZE(a)==6)
    {
      for(unsigned int i=0;i<3;i++)
		m_f(i) = a(i);

      for(unsigned int i=0;i<3;i++)
		m_n0(i) = a(i+3);
    }
  return this;
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

Inertia::Inertia()
{
  MAL_S3x3_MATRIX_SET_IDENTITY(m_I);
  MAL_S3_VECTOR_FILL(m_h,0.0);
  m_m = 0.0;
}

Inertia::Inertia(matrix3d lI,
		 vector3d lh,
		 double lm)
  : m_I(lI), m_h(lh),m_m(lm)
{
}

void Inertia::addInertia(Inertia &c,
			 Inertia &a,
			 Inertia &b) const
{
  c.m_m = a.m_m + b.m_m;
  c.m_h = a.m_h + b.m_h;
  c.m_I = a.m_I + b.m_I;
}
  
Inertia Inertia::operator+( Inertia &a)
{
  Inertia c;
  c.m_m = a.m_m + m_m;
  c.m_h = a.m_h + m_h;
  c.m_I = a.m_I + m_I;
  return c;
}

Momentum::Momentum()
{
  MAL_S3_VECTOR_FILL(m_v,0.0);
  MAL_S3_VECTOR_FILL(m_w,0.0);
}

////----------> correct the formula by L.S(substraction in lf linear expression instead of addition)
Momentum Inertia::operator*(Velocity &v)
{
  Momentum c;
  vector3d NE_tmp,NE_tmp2;
  // Angular acceleration
  NE_tmp = m_I * v.w();
  MAL_S3_VECTOR_CROSS_PRODUCT(NE_tmp2, m_h, v.v0());
  vector3d lw = NE_tmp2 + NE_tmp;
  c.w(lw);
      
  // Linear acceleration
  NE_tmp = v.v0() * m_m;
  MAL_S3_VECTOR_CROSS_PRODUCT(NE_tmp2, m_h, v.w());
  vector3d lv = NE_tmp2 - NE_tmp;
  c.v(lv);
      
  return c;
}
  
////----------> correct the formula by L.S(substraction in lf linear expression instead of addition)
Force Inertia::operator*(Acceleration &a)
{
  Force c;
  vector3d NE_tmp,NE_tmp2;
  // Angular acceleration
  NE_tmp = m_I * a.dw();
  MAL_S3_VECTOR_CROSS_PRODUCT(NE_tmp2, m_h, a.dv0());
  vector3d ln0 = NE_tmp2 + NE_tmp;
  c.n0(ln0);
      
  // Linear acceleration
  NE_tmp = a.dv0() * m_m;
  MAL_S3_VECTOR_CROSS_PRODUCT(NE_tmp2, m_h, a.dw());
  vector3d lf = NE_tmp2 - NE_tmp;
  c.f(lf);
      
  return c;
}

Force Force::operator*(double ad)
{
  Force c;
  c.m_f = m_f * ad;
  c.m_n0 = m_n0 * ad;
  return c;
}
  
Velocity Velocity::operator*(double ad)
{
  Velocity c;
  c.m_v0 = m_v0 * ad;
  c.m_w = m_w * ad;
  return c;
}

vectorN Velocity::operator^(vectorN &a)
{
  vectorN c;
  if (MAL_VECTOR_SIZE(a)==6)
    {
      // c x w
      c(3) =               -m_w(2)*a(4) + m_w(1)*a(5);
      c(4) = m_w(2)* a(3)                -m_w(0)*a(5);
      c(5) =-m_w(1)* a(3)  +m_w(0)*a(4);
      // v0 x m + w x m0
      c(0) =                -m_v0(2)*a(4) + m_v0(1)*a(5)               -m_w(2)*a(1) + m_w(1)*a(2);
      c(1) = m_v0(2)* a(3)                 -m_v0(0)*a(5)+m_w(2)* a(0)                -m_w(0)*a(2);
      c(2) =-m_v0(1)* a(3)  +m_v0(0)*a(4)               -m_w(1)* a(0)  +m_w(0)*a(1) ;

    }
  return c;
}

Force Velocity::operator^(Momentum &a)
{
  vector3d dn0,df;
  vector3d aw  = a.w();
  vector3d av0 = a.v();

  // c x w
  dn0(0) =               -m_w(2)*aw(1) + m_w(1)*aw(2);
  dn0(1) = m_w(2)* aw(0)                -m_w(0)*aw(2);
  dn0(2) =-m_w(1)* aw(0) +m_w(0)*aw(1);

  // v0 x m + w x m0
  df(0) =               -m_v0(2)*aw(1)+ m_v0(1)*aw(2)                 -m_w(2)*av0(1) + m_w(1)*av0(2);
  df(1) = m_v0(2)* aw(0)               -m_v0(0)*aw(2)+m_w(2)* av0(0)                 - m_w(0)*av0(2);
  df(2) =-m_v0(1)* aw(0)+m_v0(0)*aw(1)               -m_w(1)* av0(0)  +m_w(0)*av0(1) ;
  
  Spatial::Force c(df,dn0);
  
  return c;
} 
 
Velocity operator*(double ad, Velocity &a)
{
  Velocity c;
  c = a * ad;
  return c;
}

Momentum operator*(Inertia & sI, Velocity &v)
{
  Momentum c;
  vector3d NE_tmp,NE_tmp2;
  // Angular velocity
  NE_tmp = sI.I() * v.w();
  MAL_S3_VECTOR_CROSS_PRODUCT(NE_tmp2, sI.h(), v.v0());
  vector3d hm = NE_tmp2 + NE_tmp;
  c.w(hm);
      
  // Linear velocity
  NE_tmp = v.v0() * sI.m();
  MAL_S3_VECTOR_CROSS_PRODUCT(NE_tmp2, sI.h(), v.w());
  vector3d lv = NE_tmp2 - NE_tmp;
  c.v(lv);
      
  return c;
}

  
/*! Plucker Transform */
PluckerTransform::PluckerTransform()
{}

PluckerTransform::PluckerTransform(matrix3d lR,
				   vector3d lp):
  m_R(lR), m_p(lp) {}

 
PluckerTransform  PluckerTransform::operator*(PluckerTransform &a)
{
  PluckerTransform c;
  // Rotation
  MAL_S3x3_C_eq_A_by_B(c.m_R, m_R, a.m_R);
  // position
  matrix3d aRT;
  MAL_S3x3_TRANSPOSE_A_in_At(a.m_R,aRT);
  c.m_p = a.m_p + aRT * m_p;
  return c;
}

  
Force  PluckerTransform::operator*( Force &f)
{
  Force c;
  // Computes the angular velocity
  vector3d NE_tmp,NE_tmp2,NE_tmp3;
  MAL_S3_VECTOR_CROSS_PRODUCT(NE_tmp,m_p,f.f());
  NE_tmp2=f.n0()-NE_tmp;
    
  MAL_S3x3_C_eq_A_by_B(NE_tmp,m_R,NE_tmp2);
  c.n0(NE_tmp);
    
  // Computes the linear velocity
  NE_tmp3 = f.f();
  MAL_S3x3_C_eq_A_by_B(NE_tmp,m_R,NE_tmp3);
  c.f(NE_tmp);
  return c;
};

void PluckerTransform::inverse( PluckerTransform &a)
{
  MAL_S3x3_TRANSPOSE_A_in_At(a.m_R,m_R);
  vector3d NE_tmp;
  NE_tmp = a.p()* -1.0;
  MAL_S3x3_C_eq_A_by_B(m_p,a.m_R,NE_tmp);
}

//----------> correcting the formulas by L.S (inverting angular and linear expressions)
Velocity PluckerTransform::operator*( Velocity &v)
{
  Velocity c;
  // Computes the linear velocity
  vector3d NE_tmp,NE_tmp2;
  MAL_S3_VECTOR_CROSS_PRODUCT(NE_tmp,m_p,v.w());
  NE_tmp2=v.v0()-NE_tmp;
    
  c.v0(MAL_S3x3_RET_A_by_B(m_R,NE_tmp2));
    
  // Computes the angular velocity
  c.w(MAL_S3x3_RET_A_by_B(m_R,v.w()));
  return c;
}
//----------> correcting the formulas by L.S (inverting angular and linear expressions)
Acceleration PluckerTransform::operator*( Acceleration &v)
{
  Acceleration c;
  // Computes the linear velocity
  vector3d NE_tmp,NE_tmp2;
  MAL_S3_VECTOR_CROSS_PRODUCT(NE_tmp,m_p,v.dw());
  NE_tmp2=v.dv0()-NE_tmp;
    
  c.dv0(MAL_S3x3_RET_A_by_B(m_R,NE_tmp2));
    
  // Computes the angularvelocity
  c.dw(MAL_S3x3_RET_A_by_B(m_R,v.dw()));
  return c;
}

//----------> Adding the operator = to associate 2 equivalent quantities by L.S
PluckerTransform* PluckerTransform::operator=( const PluckerTransform &a)
{
	if (&a == this) return this;
	m_R = a.m_R;
	m_p = a.m_p;
}

//----------> To develop the Transpose of a Plucker Transform by L.S 
void PluckerTransform::transpose( PluckerTransform &a)
{
  MAL_S3x3_TRANSPOSE_A_in_At(a.m_R,m_R);
  vector3d NE_tmp;
  NE_tmp = a.p()* -1.0;
  MAL_S3x3_C_eq_A_by_B(m_p,a.m_R,NE_tmp);
}