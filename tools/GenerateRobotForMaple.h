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
/* @doc \file Object to generate a file following VRML 1.0 format.*/
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

      /*! \name Getter/Setter on normalisation flag.
	The normalization aligned the axis along the x-axis
	with revolute join. */
      void setNormalization(bool lNormalization);

      bool getNormalization();
	
      
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
      
      void GenerateDummyTag(std::ostream &os,
			    unsigned int gindex,
			    std::string &JointName,
			    unsigned int JointRank);

      void GenerateSupplementaryTags(std::ostream &os,
				     CjrlHumanoidDynamicRobot *aHDR,
				     unsigned int &gindex);

      void GenerateMapleScript(std::string &RobotName);

      void GenerateGPLv2License(std::ostream &os);
      
      void GenerateHeader(std::ostream &os,
			  CjrlHumanoidDynamicRobot *aHDR);

      void GenereateHeader(std::ostream &os,
			   CjrlHumanoidDynamicRobot *aHDR);

      double FilterPrecision(double x);

      std::vector<BodyGeometricalData> m_AccessToData;

      std::map<CjrlJoint *, unsigned int> m_Indexes;

      /*! Extract an unnormalized transformation for the joint aJoint. */
      void UnnormalizedTransformation(CjrlJoint * aJoint,
				      MAL_S4x4_MATRIX(,double) &aTransformation);

      /*! Extract an unnormalized CoM and IG for the body related to joint aJoint. */
      void UnnormalizedComAndInertia(CjrlJoint * aJoint,
				     vector3d &uCom,
				     matrix3d &uIG);

      bool m_NormalizationFlag;
    };
  };
};
#endif /* _DYNAMIC_JRL_JAPAN_FOR_VRML1_H_ */
