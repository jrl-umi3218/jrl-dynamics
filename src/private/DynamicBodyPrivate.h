/* Computation of the dynamical aspect for the body 
   of a model. 
   OS: Almost all the modifications are related to the computation
   of the Resolved Momentum Control.
   
   OS: Updated the names of the contributors, the documentation
   and added a sample file for WalkPlugin
   OS (21/12/2006): removed any reference to non-homogeneous matrix
   library.


   Copyright (c) 2005-2006, 
   @author Jean Remy Chardonnet,  Francois Keith, Abderrahmane Kheddar,  Olivier Stasse,  Ramzi Sellouati, Florent Lamiraux
   
   JRL-Japan, CNRS/AIST

   All rights reserved.

   Please refers to file License.txt for details on the license.
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

    vector3d dv_c;
    matrix3d Riip1;
    
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
      (as defined in Kajita's book page 46 figure 2.20):
      - \a a  = rotation vector,
      - \a b  = translation vector,
      - \a w_c = center of mass in reference frame,
      - \a p  = position,
      - \a v0 = linear velocity in reference frame,
      - \a dv = linear acceleration,
      - \a w  = angular velocity,
      - \a dw = angular acceleration,
      - \a w_a = axis for revolute joint in the world reference frame.
    */
    vector3d a, b, w_c, p,
      v0,  dv,  w,  dw,
      w_a;

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
