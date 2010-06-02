/* Class to implement a Joint object.

   Copyright (c) 2007, 
   @author Olivier Stasse, Florent Lamiraux, Francois Keith
   
   JRL-Japan, CNRS/AIST

   All rights reserved.

   Please refers to file License.txt for details on the license.   
*/

#ifndef JOINT_ROTATION_PRIVATE_H
#define JOINT_ROTATION_PRIVATE_H

#include <vector>

#include "MatrixAbstractLayer/MatrixAbstractLayer.h"
#include "robotDynamics/jrlJoint.h"
#include <JointPrivate.h>

using namespace std;

namespace dynamicsJRLJapan
{  
    class DynamicBodyPrivate;
    
    class JointRotationPrivate : public JointPrivate
    {
    public:
      /*! \brief Constructor when the initial position is not known.
	This evaluation will done after and by doing a normalization.
       */
      JointRotationPrivate();
      
      /*! \brief Constructor when the initial position is known.
       Do not do any normalization. */
      JointRotationPrivate(const matrix4d &inInitialPosition);

      /*! Default destructor */
      virtual ~JointRotationPrivate();

      /*! Compute position and orientation for state vector given inDofVector. */
      bool updateTransformation(const vectorN & inDofVector);
    };

};

#endif /* JOINT_ROTATION_PRIVATE_*/
