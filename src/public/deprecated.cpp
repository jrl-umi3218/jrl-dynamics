/*
 * Copyright 2010,
 *
 * Oussama Kanoun,
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

#include "jrl/mal/matrixabstractlayer.hh"
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
