/*
 * Copyright 2005, 2006, 2007, 2008, 2009, 2010,
 *
 * Jean-Remy Chardonnet
 * Abderrahmane Kheddar
 * Florent Lamiraux
 * Olivier Stasse
 * Ramzi Sellouati
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

/* Computation of the dynamical aspect for the body
   of a model.

*/

#include "DynamicBodyPrivate.h"

using namespace dynamicsJRLJapan;

DynamicBodyPrivate::DynamicBodyPrivate():Body()
{
  sister =-1;
  child=-1;

  q =0.0;
  dq = 0.0;
  ddq = 0.0;

  MAL_S3x3_MATRIX_SET_IDENTITY(R);
  MAL_S4x4_MATRIX_SET_IDENTITY(m_transformation);

  a[0] = a[1] = a[2] = 0;
  b[0] = b[1] = b[2] = 0;
  w_c[0] = w_c[1] = w_c[2] = 0;
  w_a[0] = w_a[1] = w_a[2] = 0;
  p[0] = p[1] = p[2] = 0;
  v0[0] = v0[1] = v0[2] = 0;
  dv[0] = dv[1] = dv[2] = 0;
  w[0] = w[1] = w[2] = 0;
  dw[0] = dw[1] = dw[2] = 0;  

  /*L.S new variables for the spatial notations*/

  MAL_VECTOR_RESIZE(sq,0);
  MAL_VECTOR_RESIZE(sdq,0);
  MAL_VECTOR_RESIZE(sddq,0);
  MAL_VECTOR_RESIZE(stau,1);

  MAL_VECTOR_FILL(sq,0);
  MAL_VECTOR_FILL(sdq,0);
  MAL_VECTOR_FILL(sddq,0);
  MAL_VECTOR_FILL(stau,0);
}

DynamicBodyPrivate::~DynamicBodyPrivate()
{

}

DynamicBodyPrivate & DynamicBodyPrivate::operator=(const DynamicBodyPrivate & r)
{
  *((Body *)this)= *((Body *)&r);
  //  (Body)*this= (Body)r;

  // JointPrivate value.
  q = r.q;
  dq = r.dq;
  ddq = r.ddq;

  // Inertia related.
  R = r.R;
  a = r.a;
  b = r.b;
  w_c = r.w_c;
  p = r.p;
  v0 = r.v0;
  dv = r.dv;
  w = r.w;
  dw = r.dw;

  sister = r.sister;
  child = r.child;

  P = r.P;
  L = r.L;

  /*L.S new position-velocity-acceleration variables for the spatial notations*/
  sq = r.sq;
  sdq = r.sdq;
  sddq = r.sddq;

  return *this;
}


DynamicBodyPrivate & DynamicBodyPrivate::operator=(const Body & r)
{
  *((Body *)this) =r ;
  this->localCenterOfMass(r.localCenterOfMass());
  return *this;
}

