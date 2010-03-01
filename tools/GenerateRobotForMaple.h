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
      /*! \brief Generate Kinematic Data. */
      void GenerateKinematicData(std::string &RobotName,
				 CjrlHumanoidDynamicRobot *aHDR);

      /*! \brief Generate Dynamic Data. */
      void GenerateDynamicData(std::string &RobotName,
			       CjrlHumanoidDynamicRobot *aHDR);

      /*! \brief Generate Dynamic Data. */
      void GenerateContactData(std::string &RobotName,
			       CjrlHumanoidDynamicRobot *aHDR);


      std::string m_PathToModelFiles;

      /*! \name Part related to the dynamic information generation
	@{
      */
      /*! \brief Main entry point */
      void GenerateBodies(std::ostream &os,
			  CjrlHumanoidDynamicRobot *aHDR);

      /*! \brief Generate dynamic information for one body */
      void GenerateBody(CjrlJoint *aJoint, 
			std::ostream &os,
			std::string & shifttab,
			unsigned int &gindex);

      /*! @} */

      /*! /name Part related to the kinematic information generation
	@{
       */
      /*! \brief Main entry point 
       */
      void GenerateJoints(std::ostream &os,
			  std::string shifttab,
			  CjrlHumanoidDynamicRobot *aHDR);
      
      /*! \brief Compute and generate information for one joint */
      void GenerateJoint(CjrlJoint *aJoint, 
			 std::ostream &os,
			 std::string shifttab,
			 unsigned int &gindex);

      /*! \brief Generate file for one joint */
      void GenerateJointFilePart(CjrlJoint *aJoint, 
				 std::ostream &os,
				 unsigned &indexparent, unsigned int &gindex,
				 vector3d &aRealAxis,
				 matrix4d &aTransformation);
      /*! @} */

      /*! \name Part related to the contact point information 
	@{
       */
      void GenerateContactPointFile(std::ostream &os,
				    CjrlHumanoidDynamicRobot *aHDR);
      
      void ComputeContactPointsForOneFoot(CjrlHumanoidDynamicRobot *aHDR,
					  int LeftOrRight,
					  vector4d ContactPoints[4]);

      void GenerateContactPointsForOneFoot(std::ostream &os,
					   CjrlHumanoidDynamicRobot *aHDR,
					   int LeftOrRight,
					   vector4d ContactPoints[4],
					   unsigned int &gindex);

      void ExtractEulerAngles(matrix3d &aRotationMatrix,
			      vector3d &EulerAngles);
      /*! @} */

      void GenerateMapleScript(std::string &RobotName);

      void GenerateGPLv2License(std::ostream &os);
      
      void GenerateHeader(std::ostream &os,
			  CjrlHumanoidDynamicRobot *aHDR);

      void GenereateHeader(std::ostream &os,
			   CjrlHumanoidDynamicRobot *aHDR);

      double FilterPrecision(double x);

      std::vector<BodyGeometricalData> m_AccessToData;

      std::map<CjrlJoint *, unsigned int> m_Indexes;
    };
  };
};
#endif /* _DYNAMIC_JRL_JAPAN_FOR_VRML1_H_ */
