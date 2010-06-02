/* Class to implement a Joint object.

   Copyright (c) 2007, 
   @author Olivier Stasse, Florent Lamiraux, Francois Keith
   
   JRL-Japan, CNRS/AIST

   All rights reserved.

   Please refers to file License.txt for details on the license.   
*/

#ifndef JOINT_ANCHOR_PRIVATE_H
#define JOINT_ANCHOR_PRIVATE_H

#include <vector>

#include "MatrixAbstractLayer/MatrixAbstractLayer.h"
#include "robotDynamics/jrlJoint.h"
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
      bool updateTransformation(const vectorN & inDofVector);
    };

};

#endif /* JOINT_ANCHOR_PRIVATE */
