/* Computation of the dynamical aspect for the body 
   of a model.
   OS: Almost all the modifications are related to the computation
   of the Resolved Momentum Control.

   OS: Updated the names of the contributors, the documentation
   and added a sample file for WalkPlugin
   OS (21/12/2006): removed any reference to non-homogeneous matrix
   library.

   Copyright (c) 2005-2006, 
   Jean Remy Chardonnet,
   Abderrahmane Kheddar,
   Olivier Stasse,
   Ramzi Sellouati
   
   JRL-Japan, CNRS/AIST

   All rights reserved.

   Please refers to file License.txt for details on the license.
*/

#include "dynamicsJRLJapan/DynamicBody.h"

using namespace dynamicsJRLJapan;

DynamicBody::DynamicBody():Body()
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
  c[0] = c[1] = c[2] = 0;
  w_c[0] = w_c[1] = w_c[2] = 0;
  w_a[0] = w_a[1] = w_a[2] = 0;
  p[0] = p[1] = p[2] = 0;
  v0[0] = v0[1] = v0[2] = 0;
  dv[0] = dv[1] = dv[2] = 0;
  w[0] = w[1] = w[2] = 0;
  dw[0] = dw[1] = dw[2] = 0;
  sv[0] = sv[1] = sv[2] = 0;
  sw[0] = sw[1] = sw[2] = 0;
  cv[0] = cv[1] = cv[2] = 0;
  cw[0] = cw[1] = cw[2] = 0;
  hhv[0] = hhv[1] = hhv[2] = 0;
  hhw[0] = hhw[1] = hhw[2] = 0;
  pph[0] = pph[1] = pph[2] = 0;
  ppb[0] = ppb[1] = ppb[2] = 0;
  
  
  MAL_S3x3_MATRIX_SET_IDENTITY(pastR);
  pastp[0] = pastp[1] = pastp[2] = 0;
  pastv0[0] = pastv0[1] = pastv0[2] = 0;
  pastw[0] = pastw[1] = pastw[2] = 0;
  pastq = pastdq = 0;
}

DynamicBody::~DynamicBody()
{

}

DynamicBody & DynamicBody::operator=(const DynamicBody & r)
{
  *((Body *)this)= *((Body *)&r);
  //  (Body)*this= (Body)r;

  // Joint value.
  q = r.q;
  dq = r.dq;
  ddq = r.ddq;

  // Inertia related.
  R = r.R;
  a = r.a;
  b = r.b;
  c = r.c;
  w_c = r.w_c;
  p = r.p;
  v0 = r.v0;
  dv = r.dv;
  w = r.w;
  dw = r.dw;
  sv = r.sv;
  sw = r.sw;
  cv = r.cv;
  cw = r.cw;
  hhv = r.hhv;
  hhw = r.hhw;
  pph = r.pph;
  ppb = r.ppb;
  
  pastR = r.pastR;
  pastp = r.pastp;
  pastv0 = r.pastv0;
  pastw = r.pastw;
  pastq =r.pastq;
  pastdq =r.pastdq;
  
  sister = r.sister;
  child = r.child;
  
  P = r.P;
  L = r.L;
  m_tildem = r.m_tildem;
  m_tildem_sister = r.m_tildem_sister;

  m_tildec = r.m_tildec;
  m_tildec_sister = r.m_tildec_sister;
  m_tildeI = r.m_tildeI;
  m_tildeI_sister = r.m_tildeI_sister;
  m_Dsister = r.m_Dsister;

  m_RMC_m = r.m_RMC_m;
  m_RMC_h = r.m_RMC_h;
  
  return *this;
}


DynamicBody & DynamicBody::operator=(const Body & r)
{
  *((Body *)this) =r ;
  this->c = r.getPositionCoM();
  return *this;
}

