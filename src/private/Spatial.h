/* \file Spatial vector algebra
   Copyright (c) 2010
   Olivier Stasse
   
*/
#ifndef _SPATIAL_ALGEBRA_H_
#define _SPATIAL_ALGEBRA_H_

#include "MatrixAbstractLayer/MatrixAbstractLayer.h"
#include "robotDynamics/jrlBody.h"

namespace dynamicsJRLJapan
{
  namespace Spatial
  {
    class Velocity;
    class Acceleration;
    class cAcceleration;
    class Inertia;
    class PluckerTransform;

    class Velocity
    {
    public:
      Velocity();
      Velocity(vector3d lv0,vector3d lw);       
      Velocity operator+(Velocity &a);
      Velocity operator-(Velocity &a);
      Velocity operator*(double ad);

      friend Velocity operator*(double ad, Velocity &a);
      friend Velocity operator*(Inertia & ,Velocity &);
      friend Velocity operator*(PluckerTransform &c, Velocity &v);
      
      const vector3d& v0()
      { return m_v0;}
      void v0(const vector3d &lv0)
      { m_v0 = lv0;}

      const vector3d& w()
      { return m_w;};
      void w(const vector3d& lw)
      { m_w = lw;};

    private:
      vector3d m_v0,m_w;
    };

    Velocity operator*(Inertia &, Velocity &);

    class Acceleration
    {
    public:
      Acceleration();
      Acceleration(vector3d ldv0, vector3d ldw);
      Acceleration operator+(Acceleration &a);
      Acceleration operator-(Acceleration &a);

      const vector3d& dv0()
      { return m_dv0;}
      const vector3d& dw()
      { return m_dw;};

    private:
      vector3d m_dv0, m_dw;
    };

    class cAcceleration
    {
    public:
      cAcceleration();
      cAcceleration(vector3d ldv0, vector3d ldw);
      cAcceleration operator+(cAcceleration &a);
      cAcceleration operator-(cAcceleration &a);

      const vector3d& dv0()
      { return m_dv0;}
      const vector3d& dw()
      { return m_dw;};

    private:  
      vector3d m_dv0,m_dw;
    };

    class Force
    {
    public:
      Force();
      Force(vector3d lf, vector3d ln0);
      Force operator+(Force &a);
      Force operator-(Force &a);
      
      const vector3d & f()
      {return m_f;};
      const vector3d & n0()
      {return m_n0;};
      
    private:  

      vector3d m_f, m_n0;
    };
    
    class Motion
    {
    public:
      Motion();
      Motion(vector3d lp, vector3d ltheta);
      Motion operator+(Motion &a);
      Motion operator-(Motion &a);

      const vector3d & p();
      const vector3d & theta();
    private:  
      vector3d m_p, m_theta;
    };

    class Inertia
    {
    public:
      Inertia();
      Inertia(matrix3d lI,   vector3d lh, double lm);
      void addInertia(Inertia &c,
		      const Inertia &a,
		      const Inertia &b) const;
	
      Inertia operator+(const Inertia &a);
      friend Velocity operator*(Inertia & ,Velocity &);

      const matrix3d & I()
      { return m_I;};
      const vector3d & h()
      { return m_h;};
      double m()
      { return m_m;};

    private:
      matrix3d m_I;
      vector3d m_h;
      double m_m;
    };

    
    class PluckerTransform
    {
    public:
      PluckerTransform();
      PluckerTransform(matrix3d lR, vector3d lp) ;
      PluckerTransform operator*(const PluckerTransform &a);
      Velocity operator*(const Velocity &a);
      void inverse(const PluckerTransform &a);

      const matrix3d & R()
      {return m_R;}
      const vector3d & p()
      {return m_p;}

    private:
      matrix3d m_R;
      vector3d m_p;

    public:
      friend Velocity operator*(const Velocity &v);
      friend Velocity operator*(PluckerTransform &c, Velocity &v);

    };
  };
};
#endif /* _SPATIAL_ALGEBRA_H_ */
