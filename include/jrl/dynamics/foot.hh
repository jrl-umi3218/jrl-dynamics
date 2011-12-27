/* @doc Object used to handle a foot
 *
 * Copyright 2009, 2010,
 *
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

#ifndef _DYN_JRL_JAPAN_FOOT_H_
#define _DYN_JRL_JAPAN_FOOT_H_
#include "jrl/mal/matrixabstractlayer.hh"

#include "abstract-robot-dynamics/foot.hh"
#include "jrl/dynamics/dll.hh"

namespace dynamicsJRLJapan
{
  /*! \ingroup userclasses
    This class represents a foot of a humanoid robot.
    It assumes some geometrical information available.
    They are described in more details in the original
    class jrlFoot. */
  class DYN_JRL_JAPAN_EXPORT Foot: public CjrlFoot
  {
  public:

    /*! \brief Default constructor */
    Foot();
    Foot(const Foot &inFoot);

        /**
    \brief Destructor
     */
    virtual ~Foot();

    /*! Returns associated ankle. */
    virtual CjrlJoint * associatedAnkle() const;

    /*! Returns associated ankle. */
    void setAssociatedAnkle(CjrlJoint * inAssociatedAnkle);

    /**
	\brief Get size of the rectagular sole

	\retval outLength length of the sole (see Figure)
	\retval outWidth width of the sole (see Figure)

    */
    virtual void getSoleSize(double &outLength, double &outWidth) const;

    /**
	\brief Set size of the rectagular sole

	\param inLength length of the sole (see Figure)
	\param inWidth width of the sole (see Figure)

    */
    virtual void setSoleSize(const double &inLength, const double &inWidth);

    /**
       \brief  Get position of the ankle in the foot local coordinate frame

       \retval outCoordinates coordinates of the ankle joint center
    */
    virtual void getAnklePositionInLocalFrame(vector3d& outCoordinates) const;

    /**
       \brief  Set position of the ankle in the foot local coordinate frame

       \param inCoordinates coordinates of the ankle joint center
    */
    virtual void setAnklePositionInLocalFrame(const vector3d& inCoordinates);

  private:
    /*! Store the ankle joint. */
    CjrlJoint * m_Ankle;

    /*! Store sole size. */
    double m_SoleLength, m_SoleWidth;

    /*! Store ankle position in foot frame. */
    vector3d m_AnklePositionInFootFrame;

    /*! Store center position in foot frame. */
    vector3d m_CenterInFootFrame;

  };

}

#endif /* _DYN_JRL_JAPAN_FOOT_H_ */
