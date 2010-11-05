/*
 * Copyright 2009, 2010, 
 *
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
 *
 *  Research carried out within the scope of the Associated
 *  International Laboratory: Joint Japanese-French Robotics
 *  Laboratory (JRL)
 */

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
#include <jrl/mal/matrixabstractlayer.hh>
#include "abstract-robot-dynamics/jrlhumanoiddynamicrobot.hh"
#include "abstract-robot-dynamics/jrlrobotdynamicsobjectconstructor.hh"

#include "geometricdata.hh"

namespace dynamicsJRLJapan
{
  /*! \ingroup userclasses
    \brief Hooks for to create objects. */
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
			     std::vector<BodyGeometricalData> &GeometricalDataonBodies,
			     bool ReadGeometricalInformation=false);
  
};
#endif
