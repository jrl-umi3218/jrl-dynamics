/* @doc \file Object to generate a file following VRML 1.0 format.

   Copyright (c) 2010
   @author Olivier Stasse
   JRL-Japan, CNRS/AIST
 
   All rights reserved.

   Please refers to file License.txt for details on the license.

*/
#ifndef _DYNAMIC_JRL_JAPAN_ROBOT_FOR_HPPBUILDER_H_
#define _DYNAMIC_JRL_JAPAN_ROBOT_FOR_HPPBUILDER_H_

#include <string>
#include <ostream>
#include <fstream>
#include <vector>

#include <dynamicsJRLJapan/dynamicsJRLJapanFactory.h>


namespace dynamicsJRLJapan
{
  namespace Tools
  {
    class GenerateRobotForHppBuilder
    {
    public:
      /*! \brief Default constructor */
      GenerateRobotForHppBuilder();
      
      /*! \brief Default destructor */
      ~GenerateRobotForHppBuilder();
      
      /*! \brief Generate hard coded robot */
      void GenerateRobot(std::string &RobotName,
			 CjrlHumanoidDynamicRobot *aHDR);


      void SetAccessToData(std::vector<BodyGeometricalData > &AccessToData);
      
      /*! \brief Take the links towards the geometrical information */
      void SetAccessToData(std::vector<BodyGeometricalData * > &AccessToData);

      /*! \brief Set path to the model files. */
      void SetPathToModelFiles(std::string &Path);
      

    private:

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
      
			 
      void GenerateBodies(std::ostream &os,
			  std::string shifttab,
			  std::string &JointName);

      void CopyGeometricInformation(std::ostream &os,
				    std::string FileName);

      void GenerateTriangles(CjrlJoint *aJoint, 
			     std::ostream &os,
			     std::string shifttab,
			     unsigned int &gindex);

      void GeneratePoints(CjrlJoint *aJoint, 
			  std::ostream &os,
			  std::string shifttab,
			  unsigned int &gindex);

      void GenerateMaterial(CjrlJoint *aJoint, 
			    std::ostream &os,
			    std::string shifttab,
			    unsigned int &gindex);

      void GenerateBodyData(CjrlJoint *aJoint, 
			    std::ostream &os,
			    std::string shifttab,
			    unsigned int &gindex);

      std::vector<BodyGeometricalData> m_AccessToData;
    };
  };
};
#endif /* _DYNAMIC_JRL_JAPAN_FOR_HPPBUILDER_H_ */
