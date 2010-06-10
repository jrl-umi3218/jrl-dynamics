/*
 *   Copyright (c) 2006, 2007, 2008, 2009 CNRS-AIST 
 *
 *   Research carried out within the scope of the Associated
 *   International Laboratory: Joint Japanese-French Robotics
 *   Laboratory (JRL)
 *
 *   Author: Olivier Stasse and Florent Lamiraux
 *
 *   Please refers to file License.txt for details on the license.
 *
 */

#ifndef DYNAMICSJRLJAPAN_DYNAMICBODY_H
#define DYNAMICSJRLJAPAN_DYNAMICBODY_H

#include "boost/shared_ptr.hpp"
#include "robotDynamics/jrlBody.h"
#include "dynamicsJRLJapan/dll.h"

/*
  Forward declaration
*/
class CjrlJoint;

namespace dynamicsJRLJapan {

  /** \ingroup userclasses
     \brief This class implements a body

     See CjrlJoint for documentation.
  */
  class DYN_JRL_JAPAN_EXPORT DynamicBody : virtual public CjrlBody
  {
  public:

    boost::shared_ptr<CjrlBody> m_privateObj;

    /**
       \name Constructor and destructor
     */
    virtual ~DynamicBody() {};

    DynamicBody();

    DynamicBody(const DynamicBody& inBody);

    /**
       @}
    */
    /**
       \brief Get position of center of mass in joint local reference frame.
    */
    virtual const vector3d& localCenterOfMass() const;

    /**
       \brief Set postion of center of mass in joint reference frame.
    */
    virtual void localCenterOfMass(const vector3d& inlocalCenterOfMass);

    /**
       \brief Get Intertia matrix expressed in joint local reference frame.
    */
    virtual const matrix3d& inertiaMatrix() const;

    /**
       \brief Set inertia matrix.
    */
    virtual void inertiaMatrix(const matrix3d& inInertiaMatrix);
    
    /**
    \brief Get mass.
     */
    virtual double mass() const;

    /**
    \brief Set mass.
     */
    virtual void mass(double inMass);

    /**
       \brief Get const pointer to the joint the body is attached to.
    */
    virtual const CjrlJoint* joint() const;
    
  };
};

#endif /* DYNAMICSJRLJAPAN_DYNAMICBODY_H */
