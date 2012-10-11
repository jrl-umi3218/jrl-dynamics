/*
 * Copyright 2006, 2007, 2008, 2009, 2010,
 *
 * Oussama Kanoun
 * Fumio Kanehiro
 * Francois Keith
 * Florent Lamiraux
 * Olivier Stasse
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
 */

#ifndef DYNAMICSJRLJAPAN_DYNAMICBODY_H
#define DYNAMICSJRLJAPAN_DYNAMICBODY_H

#include "boost/shared_ptr.hpp"
#include "abstract-robot-dynamics/traits/default-pointer.hh"
#include "abstract-robot-dynamics/body.hh"
#include "jrl/dynamics/dll.hh"

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
}

#endif /* DYNAMICSJRLJAPAN_DYNAMICBODY_H */
