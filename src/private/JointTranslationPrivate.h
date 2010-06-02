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
      bool updateTransformation(const vectorN & inDofVector);
    };

};

#endif /* JOINT_TRANSLATION_PRIVATE */
