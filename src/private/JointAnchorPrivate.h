/*
 * Copyright 2010,
 *
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
 *
 */

/*! Class to implement a Joint object. */

#ifndef JOINT_ANCHOR_PRIVATE_H
#define JOINT_ANCHOR_PRIVATE_H

#include <vector>

#include "jrl/mal/matrixabstractlayer.hh"
#include "abstract-robot-dynamics/joint.hh"
#include <JointPrivate.h>

using namespace std;

namespace dynamicsJRLJapan
{
    class DynamicBodyPrivate;
    class JointAnchorPrivate : public JointPrivate
    {
    public:

      JointAnchorPrivate(const matrix4d &inInitialPosition);
      virtual ~JointAnchorPrivate();

      /*! \brief Computes speed in joint and global reference frame. */
      bool updateVelocity(const vectorN& inRobotConfigVector,
			  const vectorN& inRobotSpeedVector);

      /*! \brief Computes speed in joint and global reference frame. */
      bool updateAcceleration(const vectorN& inRobotConfigVector,
			      const vectorN& inRobotSpeedVector,
			      const vectorN& inRobotAccelerationVector);


      /*! \brief Here the number of DOFs is 0. */
	  unsigned int numberDof() const
	  { return  0; }
     /* virtual unsigned int numberDof()  
      {	
			m_nbDofs = 0;
			return m_nbDofs;
	  }*/

	      /*! \brief Returns the free modes of the  joint. 
	Currently this will return an empty matrix.
      */
      const virtual matrixNxP & pcalc(const vectorN & qi);

      /*! \brief Returns the derivative of the free modes of the  joint. 
	Currently this will return an empty matrix.
      */
      const virtual matrixNxP & pdcalc(const vectorN & qi);

    };

}

#endif /* JOINT_ANCHOR_PRIVATE */
