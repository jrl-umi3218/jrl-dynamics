/* @doc Object used to handle a foot 

   Copyright (c) 2009, 

   @author : 
   Olivier Stasse.

   JRL-Japan, CNRS/AIST

   All rights reserved.
   
   Please refers to file License.txt for details on the license.

*/

#ifndef _DYN_JRL_JAPAN_FOOT_H_
#define _DYN_JRL_JAPAN_FOOT_H_
#include <MatrixAbstractLayer/MatrixAbstractLayer.h>

#include "robotDynamics/jrlFoot.h"
#include "dynamics-config.h"

namespace dynamicsJRLJapan
{
  /*! This class represents a foot of a humanoid robot.
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
    virtual CjrlJoint * associatedAnkle();

    /*! Returns associated ankle. */
    void setAssociatedAnkle(CjrlJoint * inAssociatedAnkle);
    
    /** 
	\brief Get size of the rectagular sole
	
	\retval outLength length of the sole (see Figure)
	\retval outWidth width of the sole (see Figure)
	
    */
    virtual void soleSize(double &outLength, double &outWidth);

    /*! Associated setter */
    virtual void setSoleSize(double &inLength, double &inWidth);
    
    /**
       \brief  Get position of the ankle in the foot local coordinate frame
       
       \retval outCoordinates coordinates of the ankle joint center
    */
    virtual void anklePositionInLocalFrame(vector3d& outCoordinates);

    /*! Associated setter */
    virtual void setAnklePositionInLocalFrame(vector3d& inCoordinates);
    
    /**
       \brief Get position of the sole center in foot local frame of the foot

       \refval outCoordinates coordinates of the center C of the sole (see Figure) 
    */
    virtual void soleCenterInLocalFrame(vector3d& outCoordinates);

    /*! Associated setter */
    virtual void setSoleCenterInLocalFrame(vector3d& inCoordinates);

    /**
       \brief Get position of the projection of center of local frame in sole plane

       \refval outCoordinates coordinates of the projection H of the center of the local frame in the sole plane (see Figure) 
    */
    virtual void projectionCenterLocalFrameInSole(vector3d& outCoordinates);

    /*! Associated setter */
    virtual void setProjectionCenterLocalFrameInSole(vector3d& inCoordinates);

  private:
    /*! Store the ankle joint. */
    CjrlJoint * m_Ankle;
    
    /*! Store sole size. */
    double m_SoleLength, m_SoleWidth;

    /*! Store ankle position in foot frame. */
    vector3d m_AnklePositionInFootFrame;

    /*! Store center position in foot frame. */
    vector3d m_CenterInFootFrame;

    /*! Store projection of center in sole frame. */
    vector3d m_ProjectionCenterInSoleFrame;
  };
   
};

#endif /* _DYN_JRL_JAPAN_FOOT_H_ */
