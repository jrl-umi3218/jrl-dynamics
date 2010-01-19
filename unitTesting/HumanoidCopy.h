/* @doc \file Object to copy two humanoid robots 
   through the abstract interface.

   Copyright (c) 2010
   @author Olivier Stasse
   JRL-Japan, CNRS/AIST
 
   All rights reserved.

   Please refers to file License.txt for details on the license.

*/

#ifndef _DYNAMIC_JRL_JAPAN_HUMANOID_COPY_H_
#define _DYNAMIC_JRL_JAPAN_HUMANOID_COPY_H_

#include <map>
#include <dynamicsJRLJapan/dynamicsJRLJapanFactory.h>

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

