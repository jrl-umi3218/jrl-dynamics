#ifndef ROBOT_DYNAMICS_IMPL_H
#define ROBOT_DYNAMICS_IMPL_H

#include "dynamicsJRLJapan/Joint.h"
#include "dynamicsJRLJapan/Body.h"
#include "dynamicsJRLJapan/DynamicBody.h"
#include "dynamicsJRLJapan/MultiBody.h"
#include "dynamicsJRLJapan/DynamicMultiBody.h"
#include "dynamicsJRLJapan/HumanoidDynamicMultiBody.h"
#include "dynamicsJRLJapan/Hand.h"

typedef dynamicsJRLJapan::DynamicMultiBody CimplDynamicRobot;
typedef dynamicsJRLJapan::HumanoidDynamicMultiBody CimplHumanoidDynamicRobot;
typedef dynamicsJRLJapan::JointFreeflyer CimplJointFreeFlyer;
typedef dynamicsJRLJapan::JointRotation CimplJointRotation;
typedef dynamicsJRLJapan::JointTranslation CimplJointTranslation;
typedef dynamicsJRLJapan::Joint CimplJoint;
typedef dynamicsJRLJapan::Body CimplBody;
typedef dynamicsJRLJapan::Hand CimplHand;

#endif
