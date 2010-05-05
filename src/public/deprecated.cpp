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

#include "MatrixAbstractLayer/MatrixAbstractLayer.h"
#include "dynamicsJRLJapan/dynamicRobot.h"

// using namespace dynamicsJRLJapan;

namespace jrlDelegate
{

    void dynamicRobot::computeJacobianCenterOfMass()
    {
        assert ( m_DR != 0 );
        m_DR->computeJacobianCenterOfMass();
    }

    const matrixNxP& dynamicRobot::jacobianCenterOfMass() const
    {
        assert ( m_DR != 0 );
        return m_DR->jacobianCenterOfMass();
    }

}
