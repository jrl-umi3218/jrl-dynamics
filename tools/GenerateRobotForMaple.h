/* @doc \file Object to generate a file following VRML 1.0 format.

   Copyright (c) 2010
   @author Olivier Stasse
   JRL-Japan, CNRS/AIST
 
   All rights reserved.

   Please refers to file License.txt for details on the license.

*/
#ifndef _DYNAMIC_JRL_JAPAN_ROBOT_FOR_MAPLE_H_
#define _DYNAMIC_JRL_JAPAN_ROBOT_FOR_MAPLE_H_

#include <string>
#include <ostream>
#include <fstream>
#include <vector>
#include <map>

#include <dynamicsJRLJapan/dynamicsJRLJapanFactory.h>


namespace dynamicsJRLJapan
{
  namespace Tools
  {
    class GenerateRobotForMaple
    {
    public:
      /*! \brief Default constructor */
      GenerateRobotForMaple();
      
      /*! \brief Default destructor */
      ~GenerateRobotForMaple();
      
      /*! \brief Generate hard coded robot */
      void GenerateRobot(std::string &RobotName,
			 CjrlHumanoidDynamicRobot *aHDR);
      
      /*! \brief Take the links towards the geometrical information */
      void SetAccessToData(std::vector<BodyGeometricalData> &AccessToData);

      /*! \brief Set path to the model files. */
      void SetPathToModelFiles(std::string &Path);
      

    private:

      CjrlHumanoidDynamicRobot *m_HDR;
      
      std::string m_PathToModelFiles;
      
      void GenerateBody(CjrlJoint *aJoint, 
			std::ostream &os,
			std::string & shifttab,
			unsigned int &gindex);

      void GenerateJoints(std::ostream &os,
			  std::string shifttab,
			  CjrlHumanoidDynamicRobot *aHDR);
      
      void GenerateJoint(CjrlJoint *aJoint, 
			 std::ostream &os,
			 std::string shifttab,
			 unsigned int &gindex);
      
      void GenerateBodies(std::ostream &os,
			  CjrlHumanoidDynamicRobot *aHDR);
      
      void GenerateGPLv2License(std::ostream &os);
      
      void GenerateHeader(std::ostream &os,
			  CjrlHumanoidDynamicRobot *aHDR);

      void GenereateHeader(std::ostream &os,
			   CjrlHumanoidDynamicRobot *aHDR);
      
      std::vector<BodyGeometricalData> m_AccessToData;

      std::map<CjrlJoint *, unsigned int> m_Indexes;
    };
  };
};
#endif /* _DYNAMIC_JRL_JAPAN_FOR_VRML1_H_ */
