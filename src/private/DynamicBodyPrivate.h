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

#ifndef DYNAMICBODYPRIVATE_H
#define DYNAMICBODYPRIVATE_H

#include <string>

#include "Body.h"

#include "MatrixAbstractLayer/MatrixAbstractLayer.h"
#include "robotDynamics/jrlBody.h"


namespace dynamicsJRLJapan
{

  /*! Fundamental class to store the dynamical information for one body. 
    @ingroup forwardynamics
  */
  class DynamicBodyPrivate : public Body
  {
    
  public:
    
    /*! Force vector applied to the body. */
    vector3d m_Force;
    /*! Torque vector applied to the body.  */
    vector3d m_Torque;

    matrix3d Riip1,Riip1t;
    
    /*! This relationship does make sense only if we are considering the
      relationship between this body and its mother in a given oriented
      graph.
      q, dq, ddq = articulation position, velocity and acceleration2
      u = torque
      uu, dd = intermediate variables */
    double q, dq, ddq, u, uu, dd, gr, Ir;
    
    /*! Information coded as matrices:
      
    - \a R = current body's orientation,
    - \a R_static = static body's orientation (related to its mother).
    */
    matrix3d  R,R_static;
    

    /*! Here are the physical parameters
      (as defined in Kajita's book page 46 figure 2.20)
      / addition with M.W. Spong Book. 
      - \a a  = rotation vector,
      - \a b  = translation vector,
      - \a w_c = center of mass in reference frame,

      - \a p  = position,

      - \a v0 = linear velocity in reference frame,
      - \a dv = linear acceleration global frame,
      - \a ldv = linear acceleration local frame,
      - \a ldv_c = linear acceleration of center of mass in local frame,

      - \a w  = angular velocity (Global reference frame),
      - \a lw = angular velocity (Local reference frame)

      - \a dw = angular acceleration (Global reference frame),
      - \a ldw = angular acceleration (Local reference frame),

      - \a w_a = axis for revolute joint in the global reference frame.
    */
    vector3d a, b, w_c, p,
      v0,  dv,  
      w, lw,  
      dw, ldw,
      w_a, ldv,
      ldv_c;

    /*! Transformation
      (It is redundant, but required to implement CjrlJoint::currentTransformation)
     */
    matrix4d m_transformation;

    /*! Linear and angular momentums. */
    vector3d P,L;

    int sister;
    int child;
    
    /*! Default Constructor. */
    DynamicBodyPrivate();

    /*! Default Destructor. */
    virtual ~DynamicBodyPrivate();

    /*! \name Assignment operator 
      @{
     */
    /*! From a dynamical body. */
    DynamicBodyPrivate & operator=(const DynamicBodyPrivate & r);
    /*! From a body */
    DynamicBodyPrivate & operator=(const Body & r);
    /*! @} */
    
    

  };

};
#endif /* DYNAMICBODYPRIVATE_H */
