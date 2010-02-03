/* @doc \file Object to generate a file following VRML 1.0 format.

   Copyright (c) 2010
   @author Olivier Stasse
   JRL-Japan, CNRS/AIST
 
   All rights reserved.

   Please refers to file License.txt for details on the license.

*/
#ifndef _DYNAMIC_JRL_JAPAN_ROBOT_FOR_VRML1_H_
#define _DYNAMIC_JRL_JAPAN_ROBOT_FOR_VRML1_H_

#include <string>
#include <ostream>
#include <fstream>
#include <vector>

#include <dynamicsJRLJapan/dynamicsJRLJapanFactory.h>


namespace dynamicsJRLJapan
{
  namespace Tools
  {
    class GenerateRobotForVRML2
    {
    public:
      /*! \brief Default constructor */
      GenerateRobotForVRML2();
      
      /*! \brief Default destructor */
      ~GenerateRobotForVRML2();
      
      /*! \brief Generate hard coded robot */
      void GenerateRobot(std::string &RobotName,
			 CjrlHumanoidDynamicRobot *aHDR);
      
      /*! \brief Take the links towards the geometrical information */
      void SetAccessToData(std::vector<BodyGeometricalData> &AccessToData);

      /*! \brief Set path to the model files. */
      void SetPathToModelFiles(std::string &Path);
      

    private:

      void AxisAngle(matrix4d &data, vector3d &axis, double &angle) const;

      void AxisAngle2(matrix4d &data, vector3d &axis, double &angle) const;

      CjrlHumanoidDynamicRobot *m_HDR;
      
      std::string m_PathToModelFiles;
      
      void GenerateBody(CjrlJoint *aJoint, 
			std::ostream &os,
			std::string shifttab,
			unsigned int &gindex);
      
      void GenerateJoint(CjrlJoint *aJoint, 
			 std::ostream &os,
			 std::string shifttab,
			 unsigned int &gindex);
      
      void GenerateBodies(std::ostream &os,
			  std::string shifttab);
      
      void GenerateJoints(std::ostream &os,
			  std::string shifttab);
      
      void CopyGeometricInformation(std::ostream &os,
				    std::string FileName);

      std::vector<BodyGeometricalData> m_AccessToData;

    };
  };
};
#endif /* _DYNAMIC_JRL_JAPAN_FOR_VRML1_H_ */
