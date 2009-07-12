#ifndef ROBOT_DYNAMICS_IMPL_H
#define ROBOT_DYNAMICS_IMPL_H

#include "Joint.h"
#include "DynamicBody.h"
#include "dynamicsJRLJapan/dynamicRobot.h"
#include "dynamicsJRLJapan/humanoidDynamicRobot.h"
#include "Hand.h"

typedef CjrlDynamicRobotNA CimplDynamicRobot;
typedef CjrlHumanoidDynamicRobotNA CimplHumanoidDynamicRobot;
typedef dynamicsJRLJapan::JointFreeflyer CimplJointFreeFlyer;
typedef dynamicsJRLJapan::JointAnchor CimplJointAnchor;
typedef dynamicsJRLJapan::JointRotation CimplJointRotation;
typedef dynamicsJRLJapan::JointTranslation CimplJointTranslation;
typedef dynamicsJRLJapan::Joint CimplJoint;
typedef dynamicsJRLJapan::DynamicBody CimplBody;
typedef dynamicsJRLJapan::Hand CimplHand;

#endif
