/* Class to implement a Joint object.

   Copyright (c) 2007, 
   @author Olivier Stasse, Florent Lamiraux, Francois Keith
   
   JRL-Japan, CNRS/AIST

   All rights reserved.

   Please refers to file License.txt for details on the license.   
*/

#ifndef JOINT_TRANSLATION_PRIVATE_H
#define JOINT_TRANSLATION_PRIVATE_H

#include <vector>

#include "MatrixAbstractLayer/MatrixAbstractLayer.h"
#include "robotDynamics/jrlJoint.h"
#include <JointPrivate.h>

using namespace std;

namespace dynamicsJRLJapan
{  
    class DynamicBodyPrivate;
    class JointTranslationPrivate : public JointPrivate
    {
    public:
      /*! \brief Translation joint with no know position 
	Does trigger a normalization */
      JointTranslationPrivate();
      /*! \brief Translation joint with know position 
       Does not trigger any normalization*/
      JointTranslationPrivate(const matrix4d &inInitialPosition);
      virtual ~JointTranslationPrivate();

      /*! 
	\brief Compute position and orientation for state vector 
	given inDofVector. 
	\param inDofVector: The current configuration of the robot.
	\return false is the number of dofs is not sufficient. */
      bool updateTransformation(const vectorN & inDofVector);

      /*! \brief Computes speed in joint and global reference frame. */
      bool updateVelocity(const vectorN& inRobotConfigVector,
			  const vectorN& inRobotSpeedVector);

      /*! \brief Here the number of DOFs is 6. */
      unsigned int numberDof() const 
      { return 1;};

    };

};

#endif /* JOINT_TRANSLATION_PRIVATE */
