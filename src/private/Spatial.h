/*
 * Copyright 2010,
 *
 * Olivier Stasse,
 *
 *
 * JRL/LAAS, CNRS/AIST
 *
 * This file is part of dynamicsJRLJapan.
 * dynamicsJRLJapan is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * dynamicsJRLJapan is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Lesser Public License for more details.
 * You should have received a copy of the GNU Lesser General Public License
 * along with dynamicsJRLJapan.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  Research carried out within the scope of the Associated
 *  International Laboratory: Joint Japanese-French Robotics
 *  Laboratory (JRL)
 *
 */

/* \file Spatial vector algebra */
#ifndef _SPATIAL_ALGEBRA_H_
#define _SPATIAL_ALGEBRA_H_

#include "jrl/dynamics/dll.hh"
#include "jrl/mal/matrixabstractlayer.hh"
#include "abstract-robot-dynamics/body.hh"

namespace dynamicsJRLJapan
{
  namespace Spatial
  {
    class Velocity;
    class Acceleration;
    class cAcceleration;
    class Inertia;
    class Momentum;
    class Motion;
    class Force;
    class PluckerTransform;

    class DYN_JRL_JAPAN_EXPORT Velocity
    {
    public:
      Velocity();
      Velocity(vector3d lv0,vector3d lw);
      Velocity operator+(Velocity &a);
      Velocity operator+(vectorN &a);
      Velocity operator-(Velocity &a);
      Velocity operator*(double ad);
	  Velocity* operator=(const vectorN &a);
      vectorN  operator^(vectorN &a);
      Force operator^(Momentum &a);

      friend DYN_JRL_JAPAN_EXPORT Velocity operator*(double ad, Velocity &a);
      friend DYN_JRL_JAPAN_EXPORT Velocity operator+(vectorN & ,Velocity &);

      vector3d v0()
      { return m_v0;}
      void v0(const vector3d &lv0)
      { m_v0 = lv0;}

      vector3d w()
      { return m_w;};
      void w(const vector3d& lw)
      { m_w = lw;};

    private:
      vector3d m_v0,m_w;
    };


    inline Velocity operator*(double ad, Velocity &a)
	{
		Velocity c;
		c = a * ad;
		return c;
	}

    DYN_JRL_JAPAN_EXPORT Velocity operator+(vectorN &, Velocity &);

    class DYN_JRL_JAPAN_EXPORT Acceleration
    {
    public:
      Acceleration();
      Acceleration(vector3d ldv0, vector3d ldw);
      Acceleration operator+(Acceleration &a);
      Acceleration operator-(Acceleration &a);
      Acceleration operator+(vectorN &a);
      Acceleration* operator=(vectorN &a);

      friend DYN_JRL_JAPAN_EXPORT Acceleration operator+(vectorN & ,Acceleration &);
      vector3d dv0()
      { return m_dv0;}
      void dv0(const vector3d &lv0)
      { m_dv0 = lv0;}

      vector3d dw()
      { return m_dw;};
      void dw(const vector3d &ldw)
      { m_dw = ldw; };

    private:
      vector3d m_dv0, m_dw;
    };

    DYN_JRL_JAPAN_EXPORT Acceleration operator+(vectorN &, Acceleration &);

    class DYN_JRL_JAPAN_EXPORT cAcceleration
    {
    public:
      cAcceleration();
      cAcceleration(vector3d ldv0, vector3d ldw);
      cAcceleration operator+(cAcceleration &a);
      cAcceleration operator-(cAcceleration &a);

      vector3d dv0()
      { return m_dv0;}
      vector3d dw()
      { return m_dw;};

    private:
      vector3d m_dv0,m_dw;
    };

    class DYN_JRL_JAPAN_EXPORT Force
    {
    public:
      Force();
      Force(vector3d lf, vector3d ln0);
      Force operator+(Force &a);
      Force operator-(Force &a);
      Force* operator=(vectorN &a);
      Force operator*(double ad);

      vector3d f()
      {return m_f;};
      vector3d n0()
      {return m_n0;};
      void f(vector3d &lf)
      {m_f = lf;}
      void n0(vector3d &ln0)
      {m_n0 = ln0;}

    private:

      vector3d m_f, m_n0;
    };


    class DYN_JRL_JAPAN_EXPORT Motion
    {
    public:
      Motion();
      Motion(vector3d lp, vector3d ltheta);
      Motion operator+(Motion &a);
      Motion operator-(Motion &a);

      vector3d p()
      { return m_p; }
      vector3d theta()
      { return m_theta; }

    private:
      vector3d m_p, m_theta;
    };

    class DYN_JRL_JAPAN_EXPORT Momentum
    {
    public:
      Momentum();
      Momentum(vector3d v, vector3d w);

      vector3d v()
      { return m_v;}
      vector3d w()
      { return m_w;}
      void v(vector3d &lv)
      { m_v = lv;}
      void w(vector3d &lw)
      { m_w = lw;}

    private:
      vector3d m_v, m_w;
    };

    class DYN_JRL_JAPAN_EXPORT Inertia
    {
    public:
      Inertia();
      Inertia(matrix3d lI,   vector3d lh, double lm);
      void addInertia(Inertia &c,
		      Inertia &a,
		      Inertia &b) const;

      Inertia operator+(Inertia &a);
      Momentum operator*(Velocity &);
      Force operator*(Acceleration &);

      matrix3d  I()
      { return m_I;};
      vector3d  h()
      { return m_h;};
      double m()
      { return m_m;};


     private:
      matrix3d m_I;
      vector3d m_h;
      double m_m;
    };

    class DYN_JRL_JAPAN_EXPORT PluckerTransform
    {
    public:
      PluckerTransform();
      PluckerTransform(matrix3d lR, vector3d lp) ;
      PluckerTransform operator*(PluckerTransform &a);
      Velocity operator*(Velocity &a);
      Acceleration operator*(Acceleration &a);
      Force operator*(Force &f);
      void inverse(PluckerTransform &a);
	  void transpose( PluckerTransform &a);

	  PluckerTransform* operator=( const PluckerTransform &a);

      matrix3d R()
      {return m_R;}
      vector3d p()
      {return m_p;}

    private:
      matrix3d m_R;
      vector3d m_p;

    };

  }
}


#endif /* _SPATIAL_ALGEBRA_H_ */
