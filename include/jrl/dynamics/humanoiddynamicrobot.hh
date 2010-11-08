/*
 * Copyright 2009, 2010,
 *
 * Oussama Kanoun
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
 *
 */

#ifndef JRL_HUMANOID_DYNAMIC_ROBOT_NA_H_
#define JRL_HUMANOID_DYNAMIC_ROBOT_NA_H_

#include <cassert>
#include <abstract-robot-dynamics/humanoid-dynamic-robot.hh>
#include <jrl/dynamics/dynamicsfactory.hh>

//#include "dynamicsJRLJapan/deprecated.h"
#include "jrl/dynamics/dynamicrobot.hh"

namespace jrlDelegate {

  /** \ingroup userclasses
     \brief Template to implement a non abstract class describing a humanoid robot with dynamics.
     This template takes a class implementing the methods of the template
     CjrlRobotDynamicsObjectConstructor.
  */
  class humanoidDynamicRobot : public CjrlHumanoidDynamicRobot,
    public virtual dynamicRobot
    {
    private:
      CjrlHumanoidDynamicRobot *m_HDR;

    public:

      humanoidDynamicRobot(CjrlRobotDynamicsObjectFactory * inObjectFactory)
	{
	  m_HDR = inObjectFactory->createHumanoidDynamicRobot();
	  setDynamicRobot(m_HDR);
	}

      humanoidDynamicRobot(humanoidDynamicRobot *inHDRNA )
	{
	  m_HDR = inHDRNA;
	  setDynamicRobot(m_HDR);
	}

      humanoidDynamicRobot()
	{
	  m_HDR=0;
	  setDynamicRobot(m_HDR);
	}
      /**
	 \brief Destructor
      */
      virtual ~humanoidDynamicRobot()
	{
	  // m_HDR should be deleted through DynamicRobot.
	}

      /**
	 \name Joints specific to humanoid robots
      */

      /**
	 \brief Set the pointer to the waist.
      */
      virtual void waist(CjrlJoint* inWaist)
      {
	assert(m_HDR != 0);
	m_HDR->waist(inWaist);

      }

      /**
	 \brief Get a pointer to the waist.
      */
      virtual CjrlJoint* waist()
      {
	assert(m_HDR != 0);
	return m_HDR->waist();

      }

      /**
	 \brief Set the pointer to the chest.

	 \note for some humanoid robots, the waist and the chest are the same joints.
      */
      virtual void chest(CjrlJoint* inChest)
      {
	assert(m_HDR != 0);
	m_HDR->chest(inChest);
      }

      /**
	 \brief Get a pointer to the chest.

	 \note for some humanoid robots, the waist and the chest are the same joints.
      */
      virtual CjrlJoint* chest()
      {
	assert(m_HDR != 0);
	return m_HDR->chest();
      }

      /**
	 \brief Set the pointer to the left wrist joint.
      */
      virtual void leftWrist(CjrlJoint* inLeftWrist)
      {
	assert(m_HDR != 0);
	m_HDR->leftWrist(inLeftWrist);
      }

      /**
	 \brief Get a pointer to the left wrist.
      */
      virtual CjrlJoint* leftWrist()
      {
	assert(m_HDR != 0);
	return m_HDR->leftWrist();
      }

      /**
	 \brief Set the pointer to the right wrist joint.
      */
      virtual void rightWrist(CjrlJoint* inRightWrist)
      {
	assert(m_HDR != 0);
	m_HDR->rightWrist(inRightWrist);
      }

      /**
	 \brief Get a pointer to the right wrist.
      */
      virtual CjrlJoint* rightWrist()
      {
	assert(m_HDR != 0);
	return m_HDR->rightWrist();
      }

      /**
	 \brief Set the pointer to the right hand
      */
      virtual void rightHand(CjrlHand* inRightHand)
      {
	assert(m_HDR != 0);
	m_HDR->rightHand(inRightHand);
      }

      /**
	 \brief Get a pointer to the right hand
      */
      virtual CjrlHand* rightHand()
      {
	assert(m_HDR != 0);
	return m_HDR->rightHand();
      }

      /**
	 \brief Set the pointer to the left hand
      */
      virtual void leftHand(CjrlHand* inLeftHand)
      {
	assert(m_HDR != 0);
	m_HDR->leftHand(inLeftHand);
      }

      /**
	 \brief Get a pointer to the left hand
      */
      virtual CjrlHand* leftHand()
      {
	assert(m_HDR != 0);
	return m_HDR->leftHand();

      }

      /**
	 \brief Get the hand clench value.
	 This is a scalar value ranging between 0 and 1 which
	 describes the hand clench (0 for open and 1 for closed hand)
      */
      virtual double getHandClench(CjrlHand* inHand)
      {
	assert(m_HDR != 0);
	return m_HDR->getHandClench(inHand);
      }

      /**
	 \brief Set the hand clench value. This is a scalar value
	 ranging between 0 and 1 which describes the hand clench
	 (0 for open and 1 for closed hand)
	 \return false if parameter 2 is out of range
      */
      virtual bool setHandClench(CjrlHand* inHand, double inClenchingValue)
      {
	assert(m_HDR != 0);
	return m_HDR->setHandClench(inHand,inClenchingValue);
      }

      /**
	 \brief Set the pointer to the left ankle joint.
      */
      virtual void leftAnkle(CjrlJoint* inLeftAnkle)
      {
	assert(m_HDR != 0);
	m_HDR->leftAnkle(inLeftAnkle);
      }

      /**
	 \brief Get a pointer to the left ankle.
      */
      virtual CjrlJoint* leftAnkle()
      {
	assert(m_HDR != 0);
	return m_HDR->leftAnkle();
      }

      /**
	 \brief Set the pointer to the right ankle joint.
      */
      virtual void rightAnkle(CjrlJoint* inRightAnkle)
      {
	assert(m_HDR != 0);
	m_HDR->rightAnkle(inRightAnkle);
      }

      /**
	 \brief Get a pointer to the right ankle.
      */
      virtual CjrlJoint* rightAnkle()
      {
	assert(m_HDR != 0);
	return m_HDR->rightAnkle();
      }

      /**
	 \brief Set the pointer to the left foot joint.
      */
      virtual void leftFoot(CjrlFoot* inLeftFoot)
      {
	assert(m_HDR != 0);
	return m_HDR->leftFoot(inLeftFoot);
      }

      /**
	 \brief Get a pointer to the left foot.
      */
      virtual CjrlFoot* leftFoot()
      {
	assert(m_HDR != 0);
	return m_HDR->leftFoot();
      }

      /**
	 \brief Set the pointer to the right foot joint.
      */
      virtual void rightFoot(CjrlFoot* inRightFoot)
      {
	assert(m_HDR != 0);
	return m_HDR->rightFoot(inRightFoot);
      }

      /**
	 \brief Get a pointer to the right foot.
      */
      virtual CjrlFoot* rightFoot()
      {
	assert(m_HDR != 0);
	return m_HDR->rightFoot();
      }

      /**
	 \brief Set gaze joint

	 \note  For most humanoid robots, the gaze joint is the head.
      */
      virtual void gazeJoint(CjrlJoint* inGazeJoint)
      {
	assert(m_HDR != 0);
	m_HDR->gazeJoint(inGazeJoint);
      }

      /**
	 \brief Get gaze joint
      */
      virtual CjrlJoint* gazeJoint()
      {
	assert(m_HDR != 0);
	return m_HDR->gazeJoint();
      }

      /**
	 \brief Set the gaze orientation and position in the local frame of the gaze joint.
	 \return inOrigin a point on the gaze straight line,
	 \return inDirection the direction of the gaze joint.
      */
      virtual void gaze(const vector3d& inDirection, const vector3d& inOrigin)
      {
	assert(m_HDR != 0);
	m_HDR->gaze(inDirection,inOrigin);

      }

      /**
	 \brief Get a point on the gaze straight line
      */
      virtual const vector3d& gazeOrigin() const
      {
	assert(m_HDR != 0);
	return m_HDR->gazeOrigin();
      }

      /**
	 \brief Get the direction of gaze
      */
      virtual const vector3d& gazeDirection() const
      {
	assert(m_HDR != 0);
	return m_HDR->gazeDirection();
      }

      /**
	 \@}
      */

      /**
	 \name Zero momentum point
      */

      /**
	 \brief return the coordinates of the Zero Momentum Point.
      */
      virtual const vector3d& zeroMomentumPoint() const
      {
	assert(m_HDR != 0);
	return m_HDR->zeroMomentumPoint();
      }


      friend int dynamicsJRLJapan::parseOpenHRPVRMLFile(CjrlHumanoidDynamicRobot &ajrlHumanoidDynamicRobot,
							std::string &OpenHRPVRMLFile,
							std::string &MapJointToRankFileName,
							std::string &SpecificitiesFileName,
							std::vector<BodyGeometricalData> &VectorOfURLs,
							bool ReadGeometryInformation);

      /*! @} */

      /**
	 @}
      */

    };

};

#endif
