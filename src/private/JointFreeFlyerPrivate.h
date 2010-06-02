/* Class to implement a Joint object.

   Copyright (c) 2007, 
   @author Olivier Stasse, Florent Lamiraux, Francois Keith
   
   JRL-Japan, CNRS/AIST

   All rights reserved.

   Please refers to file License.txt for details on the license.   
*/

#ifndef JOINT_FREE_FLYER_PRIVATE_H
#define JOINT_FREE_FLYER_PRIVATE_H

#include <vector>

#include "JointPrivate.h"
#include "MatrixAbstractLayer/MatrixAbstractLayer.h"
#include "robotDynamics/jrlJoint.h"

using namespace std;

namespace dynamicsJRLJapan
{  
    class DynamicBodyPrivate;
    class JointFreeflyerPrivate : public JointPrivate
    {
    public:
      /*! \brief Constructor when the initial position is not known 
	This call for a renormalization of the object. */
      JointFreeflyerPrivate();

      /*! \brief Constructor when the initial position is known 
	Do not renormalize the object.
       */
      JointFreeflyerPrivate(const matrix4d &inInitialPosition);
      virtual ~JointFreeflyerPrivate();
      bool updateTransformation(const vectorN & inDofVector);

    private:
      /*! Store variables */
      vectorN m_dof6D;
      
    };

};

#endif /* JOINT_FREE_FLYER_PRIVATE*/
