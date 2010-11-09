/*
 * Copyright 2010,
 *
 * Oussama Kanoun
 * Francois Keith
 * Olivier Stasse
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
/* \file This class implements the behavior of a revolute joint.
*/

#ifndef JOINT_ROTATION_PRIVATE_H
#define JOINT_ROTATION_PRIVATE_H

#include <vector>

#include "jrl/mal/matrixabstractlayer.hh"
#include "abstract-robot-dynamics/joint.hh"
#include <JointPrivate.h>

using namespace std;

namespace dynamicsJRLJapan
{
    class DynamicBodyPrivate;

    /*! \brief This class computes quantities such as position, velocity, acceleration,
      force and torques specific to revolute joints and its associated body.
     */
    class JointRotationPrivate : public JointPrivate
    {
    public:
      /*! \brief Constructor when the initial position is not known.
	This evaluation will done after and by doing a normalization.
       */
      JointRotationPrivate();

      /*! \brief Constructor for copy.*/
      JointRotationPrivate(const JointRotationPrivate &a);

      /*! \brief Constructor when the initial position is known.
       Do not do any normalization. */
      JointRotationPrivate(const matrix4d &inInitialPosition);

      /*! Default destructor */
      virtual ~JointRotationPrivate();

      /*! \brief Compute position and orientation for state vector given inDofVector. */
      bool updateTransformation(const vectorN & inDofVector);

      /*! \brief Computes speed in joint and global reference frame. */
      bool updateVelocity(const vectorN& inRobotConfigVector,
			  const vectorN& inRobotSpeedVector);

      /*! \brief Computes acceleration in joint and global reference frame. */
      bool updateAcceleration(const vectorN& inRobotConfigVector,
			      const vectorN& inRobotSpeedVector,
			      const vectorN& inRobotAccelerationVector);

      /*! \brief Here the number of DOFs is one. */
      virtual unsigned int numberDof()
      { return 1;};

    };

}

#endif /* JOINT_ROTATION_PRIVATE_*/
