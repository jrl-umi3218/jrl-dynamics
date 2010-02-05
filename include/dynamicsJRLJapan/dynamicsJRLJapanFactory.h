/*! \file dynamicsJRLJapanFactory.h Object factory for dynamicsJRLJapan.
   
  
  Solely provides an implementation of AbstractRobotDynamics.
  Please look at the documentation of AbstrctRobotDynamics 
  for more information.

  Copyright (c) 2009
  @author Olivier Stasse
  
  JRL-Japan, CNRS/AIST
  
  All rights reserved.
  
  Please see License.txt for more informations on the license related to this software.
*/

#ifndef _DYNAMICS_JRL_JAPAN_FACTORY_H_
#define _DYNAMICS_JRL_JAPAN_FACTORY_H_

#if defined (WIN32)
#  ifdef dynamicsJRLJapan_EXPORTS 
#    define DYN_JRL_JAPAN_EXPORT __declspec(dllexport)
#  else  
#    define DYN_JRL_JAPAN_EXPORT __declspec(dllimport)
#  endif 
#else
#  define DYN_JRL_JAPAN_EXPORT
#endif

#include <vector>
#include <MatrixAbstractLayer/MatrixAbstractLayer.h>
#include "robotDynamics/jrlHumanoidDynamicRobot.h"
#include "robotDynamics/jrlRobotDynamicsObjectConstructor.h"

namespace dynamicsJRLJapan
{
  /*! Hooks for to create objects. */

  class DYN_JRL_JAPAN_EXPORT ObjectFactory: public CjrlRobotDynamicsObjectFactory
  {
  public:
    CjrlHumanoidDynamicRobot * createHumanoidDynamicRobot();
    
    CjrlDynamicRobot * createDynamicRobot();
    
    CjrlJoint * createJointFreeflyer(const matrix4d& inInitialPosition);
    
    CjrlJoint * createJointRotation(const matrix4d& inInitialPosition);
    
    CjrlJoint * createJointTranslation(const matrix4d& inInitialPosition);
    
    CjrlJoint * createJointAnchor(const matrix4d& inInitialPosition);
    
    CjrlBody * createBody();

    CjrlHand* createHand(const CjrlJoint* inWrist);

    CjrlFoot* createFoot(const CjrlJoint* inAnkle);
  };


  class DYN_JRL_JAPAN_EXPORT BodyGeometricalData
  {
  private:
    matrix3d m_RotationForDisplay;
    std::string m_URL;
  public:
    
    const matrix3d & getRotationForDisplay();
    void setRotationForDisplay(const matrix3d &RotationForDisplay);

    const std::string & getURL();
    void setURL(const std::string &URLtoVRML);
  };
  
  /*! Populate a CjrlHumanoidDynamicRobot instance
    from a OpenHRP vrml file and a file of specificities
    to add semantic information. 
    Right now this will fail it is not a dynamicsJRLJapan instanciated
    object.
    \param OpenHRPVRMLFile Filename which containes the humanoid description
    using OpenHRP format.
    \param MapJointToRankFileName: File describing the joint mapping from
    the VRML ID to the state vector.
    \param FileOfSpecificities Describe which joints are hands, arm...
    and so on.
    \param Geometrical information returns bodies geometry description.
    \retval ajrlHumanoidDynamicRobot The robot built by parsing the file.
    \return Negative value if failed, 0 otherwise.
  */
  DYN_JRL_JAPAN_EXPORT 
    int parseOpenHRPVRMLFile(CjrlHumanoidDynamicRobot &ajrlHumanoidDynamicRobot,
			     std::string &OpenHRPVRMLFile,
			     std::string &MapJointToRankFileName,
			     std::string &FileOfSpecificities);

  DYN_JRL_JAPAN_EXPORT 
    int parseOpenHRPVRMLFile(CjrlHumanoidDynamicRobot &ajrlHumanoidDynamicRobot,
			     std::string &OpenHRPVRMLFile,
			     std::string &MapJointToRankFileName,
			     std::string &FileOfSpecificities,
			     std::vector<BodyGeometricalData> &GeometricalDataonBodies);
  
};
#endif
