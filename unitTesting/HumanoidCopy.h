/*
 * Copyright 2010, 
 *
 * Olivier Stasse,
 * 
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
 *
 */
/* @doc \file Object to copy two humanoid robots 
   through the abstract interface. */

#ifndef _DYNAMIC_JRL_JAPAN_HUMANOID_COPY_H_
#define _DYNAMIC_JRL_JAPAN_HUMANOID_COPY_H_

#include <map>
#include <jrl/dynamics/dynamicsfactory.hh>

namespace dynamicsJRLJapan
{
  class HumanoidCopy
  {
  public:
    /* Default Constructor */
    HumanoidCopy();

    /*! \brief Default Destructor */
    ~HumanoidCopy();
    
    /*! \brief Perform copy between two humanoid robots. */
    void PerformCopyFromJointsTree(CjrlHumanoidDynamicRobot* InitialHDR,
				   CjrlHumanoidDynamicRobot* NewHDR);

  private:

    /*! \brief Map between joints from the initial humanoid robot,
      and the joints of the new humanoid robot */
    std::map<CjrlJoint *,CjrlJoint *> m_JointsMap;

    void CopyAndInstanciateBody(CjrlJoint *initJoint,
				CjrlJoint *newJoint);

    void CopyLocalFieldsOfJoints(CjrlJoint *initJoint,
				 CjrlJoint *newJoint);

    void CopyEndEffectors(CjrlHumanoidDynamicRobot *aHDR,
			  CjrlHumanoidDynamicRobot *a2HDR);

    void recursiveMultibodyCopy(CjrlJoint *initJoint,
				CjrlJoint *newJoint);

    void CopyFoot(CjrlFoot *InitFoot, CjrlFoot * &NewFoot);

    void CopyHand(CjrlHand *InitHand, CjrlHand * &NewHand);

    void CopySemantic(CjrlHumanoidDynamicRobot* aHDR,
		      CjrlHumanoidDynamicRobot* a2HDR);

    void CopyActuatedJoints(CjrlHumanoidDynamicRobot * aHDR,
			    CjrlHumanoidDynamicRobot * a2HDR);

    dynamicsJRLJapan::ObjectFactory robotDynamicsObjectConstructor;

  };
  
};
#endif

