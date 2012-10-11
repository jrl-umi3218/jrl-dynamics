/* @doc Object used to handle hand
 * Copyright 2005, 2006, 2007, 2008, 2009, 2010,
 *
 * Oussama Kanoun
 * Francois Keith
 * Florent Lamiraux
 * Olivier Stasse
 *
 * JRL/LAAS, CNRS/AIST
 *
 * This file is part of dynamicsJRLJapan.
 * dynamicsJRLJapan is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * dynamicsJRLJapan is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Lesser Public License for more details.
 * You should have received a copy of the GNU Lesser General Public License
 * along with dynamicsJRLJapan.  If not, see <http://www.gnu.org/licenses/>.

*/

#ifndef HAND_H
#define HAND_H

#include "jrl/mal/matrixabstractlayer.hh"

#include "abstract-robot-dynamics/traits/default-pointer.hh"
#include <abstract-robot-dynamics/hand.hh>
//#include "dynamicsJRLJapan/deprecated.h"
#include "jrl/dynamics/dll.hh"

namespace dynamicsJRLJapan
{
    /** \ingroup userclasses
    \brief This class represents a hand.
        A hand has a central point referenced in the wrist joint frame,
	three axis and a scalar value ranging between 0 and 1 to describe the grasping degree (0 for open and 1 for closed hand)
    */
    class DYN_JRL_JAPAN_EXPORT Hand: public CjrlHand
    {
    public:

        /**
        \brief Constructor
        */
        Hand();

	/*!\brief Destructor */
	virtual ~Hand();

        /**
            \brief Get the wrist joint to which the hand is attached
        */
        virtual CjrlJoint* associatedWrist() const;

        /**
            \brief Get the wrist joint to which the hand is attached
        */
        virtual void setAssociatedWrist(CjrlJoint * inWrist );

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

    private:

	CjrlJoint* attAssociatedWrist;

        vector3d attOkayAxis;

        vector3d attShowingAxis;

        vector3d attPalmAxis;

        vector3d attCenter;
    };
}
#endif
