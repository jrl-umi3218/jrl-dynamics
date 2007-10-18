#include "dynamicsJRLJapan/Hand.h"

using namespace dynamicsJRLJapan;

Hand::Hand(CjrlJoint* inWristJoint, const vector3d& inCenterInwristFrame, const vector3d& inOkayAxisInWristFrame, const vector3d& inShowingAxisInWristFrame, const vector3d& inPalmAxisInWristFrame)
{
    attAssociatedWrist = inWristJoint;
    attOkayAxis = inOkayAxisInWristFrame;
    attShowingAxis = inShowingAxisInWristFrame;
    attPalmAxis = inPalmAxisInWristFrame;
    attCenter = inCenterInwristFrame;
    
}

Hand::~Hand()
{
}

CjrlJoint* Hand::associatedWrist()
{
    return attAssociatedWrist;
}

vector3d& Hand::centerInWristFrame()
{
    return attCenter;
}

vector3d& Hand::okayAxisInWristFrame()
{
    return attOkayAxis;
}

vector3d& Hand::showingAxisInWristFrame()
{
    return attShowingAxis;
}

vector3d& Hand::palmAxisInWristFrame()
{
    return attPalmAxis;
}

