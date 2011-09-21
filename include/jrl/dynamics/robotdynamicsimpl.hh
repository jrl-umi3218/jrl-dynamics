/*
 * Copyright 2007, 2008, 2009, 2010,
 *
 * Fumio Kanehiro
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

#ifndef ROBOT_DYNAMICS_IMPL_H
#define ROBOT_DYNAMICS_IMPL_H

#include <jrl/mal/matrixabstractlayer.hh>

#include "jrl/dynamics/joint.hh"
#include "jrl/dynamics/dynamicrobot.hh"
#include "jrl/dynamics/dynamicbody.hh"
#include "jrl/dynamics/humanoiddynamicrobot.hh"
#include "jrl/dynamics/foot.hh"
#include "jrl/dynamics/hand.hh"

typedef jrlDelegate::dynamicRobot CimplDynamicRobot;
typedef jrlDelegate::humanoidDynamicRobot CimplHumanoidDynamicRobot;
typedef dynamicsJRLJapan::JointFreeflyer CimplJointFreeFlyer;
typedef dynamicsJRLJapan::JointAnchor CimplJointAnchor;
typedef dynamicsJRLJapan::JointRotation CimplJointRotation;
typedef dynamicsJRLJapan::JointTranslation CimplJointTranslation;
typedef dynamicsJRLJapan::Joint CimplJoint;
typedef dynamicsJRLJapan::DynamicBody CimplBody;
typedef dynamicsJRLJapan::Hand CimplHand;
typedef dynamicsJRLJapan::ObjectFactory CimplObjectFactory;

#endif
