/* @doc Object used to handle hand

   Copyright (c) 2005-2009, 

   @author : 
   Oussama Kanoun
   
   JRL-Japan, CNRS/AIST

   All rights reserved.
   
   Please refers to file License.txt for details on the license.

*/

#ifndef HAND_H
#define HAND_H

#include <MatrixAbstractLayer/MatrixAbstractLayer.h>

#include <robotDynamics/jrlHand.h>
#include "dynamics-config.h"

namespace dynamicsJRLJapan
{
    /**
    \brief This class represents a HRP2 hand with a parallel mechanism.
        A hand has a central point referenced in the wrist joint frame, three axis and a scalar value ranging between 0 and 1 to describe the grasping degree (0 for open and 1 for closed hand)
    */
    class DYN_JRL_JAPAN_EXPORT Hand: public CjrlHand
    {
    public:
    
        /**
        \brief Constructor
        */
        Hand(CjrlJoint* inWristJoint, const vector3d& centerInwristFrame, const vector3d& okayAxisInWristFrame, const vector3d& showingAxisInWristFrame, const vector3d& palmAxisInWristFrame);

	/*!\brief Destructor */
	virtual ~Hand();
    
        /**
            \brief Get the wrist joint to which the hand is attached
        */
        virtual CjrlJoint* associatedWrist();
    
        /**
            \brief Get the center of the hand in the wrist frame
        */
        virtual vector3d& centerInWristFrame();
    
        /**
            \brief Get the axis defined by the thumb being held up in the way an "okay" sign is made. The returned axis is a 3d vector in the wrist frame.
        */
        virtual vector3d& okayAxisInWristFrame();
    
        /**
            \brief Get the axis defined by the forefinger being. The returned axis is a 3d vector in the wrist frame,
        */
        virtual vector3d& showingAxisInWristFrame();
    
        /**
            \brief Get the axis orthogonal to the palm. The returned axis is a 3d vector in the wrist frame pointing to the direction where all fingers can join,.
        */
        virtual vector3d& palmAxisInWristFrame();
    
    
    private:
    
        CjrlJoint* attAssociatedWrist;
    
        vector3d attOkayAxis;
    
        vector3d attShowingAxis;
    
        vector3d attPalmAxis;
    
        vector3d attCenter;
    };


#endif
};
