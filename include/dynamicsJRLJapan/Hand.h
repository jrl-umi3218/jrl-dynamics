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
#include "dynamicsJRLJapan/dll.h"

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
        Hand(const CjrlJoint* inWristJoint);

	/*!\brief Destructor */
	virtual ~Hand();
    
        /**
            \brief Get the wrist joint to which the hand is attached
        */
        virtual const CjrlJoint* associatedWrist();
    
	/**
	   \brief Get the center of the hand

	   \retval outCenter Center of the hand in the frame of the wrist.

	*/
	void getCenter(vector3d& outCenter) const;

	/**
	   \brief Set the center of the hand

	   \param inCenter Center of the hand in the frame of the wrist.

	*/
	void setCenter(const vector3d& inCenter);

	/**
	   \brief Get thumb axis when had is in open position

	   \retval outThumbAxis Axis of the thumb in wrist frame in open position

	*/
	void getThumbAxis(vector3d& outThumbAxis) const;

	/**
	   \brief Set thumb axis in wrist frame when had is in open position

	   \param inThumbAxis Axis of the thumb in wrist frame in open position
	*/
	void setThumbAxis(const vector3d& inThumbAxis);

	/**
	   \brief Get forefinger axis

	   \retval outForeFingerAxis axis of the forefinger in wrist frame 
	   in open position 
	*/
	void getForeFingerAxis(vector3d& outForeFingerAxis) const;

	/**
	   \brief Set forefinger axis

	   \param inForeFingerAxis axis of the forefinger in wrist frame 
	   in open position 
	*/
	void setForeFingerAxis(const vector3d& inForeFingerAxis);

	/**
	   \brief Get palm normal

	   \retval outPalmNormal normal to the palm in the frame of the wrist.
	*/
	void getPalmNormal(vector3d& outPalmNormal) const;

	/**
	   \brief Set palm normal

	   \param inPalmNormal normal to the palm in the frame of the wrist.
	*/
	void setPalmNormal(const vector3d& inPalmNormal);


        /**
            \brief Get the center of the hand in the wrist frame
        */
        virtual vector3d& centerInWristFrame() __attribute__ ((deprecated));
    
        /**
            \brief Get the axis defined by the thumb being held up in the way an "okay" sign is made. The returned axis is a 3d vector in the wrist frame.
        */
        virtual vector3d& okayAxisInWristFrame() __attribute__ ((deprecated));
    
        /**
            \brief Get the axis defined by the forefinger being. The returned axis is a 3d vector in the wrist frame,
        */
        virtual vector3d& showingAxisInWristFrame() __attribute__ ((deprecated));
    
        /**
            \brief Get the axis orthogonal to the palm. The returned axis is a 3d vector in the wrist frame pointing to the direction where all fingers can join,.
        */
        virtual vector3d& palmAxisInWristFrame() __attribute__ ((deprecated));
    
    
    private:
    
        const CjrlJoint* attAssociatedWrist;
    
        vector3d attOkayAxis;
    
        vector3d attShowingAxis;
    
        vector3d attPalmAxis;
    
        vector3d attCenter;
    };


#endif
};
