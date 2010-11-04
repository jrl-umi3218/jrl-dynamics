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
 /* @doc \file Common Tools for unitary testing */

 #ifndef _DYNAMIC_JRL_JAPAN_COMMON_TOOLS_H_
 #define _DYNAMIC_JRL_JAPAN_COMMON_TOOLS_H_

 #include <string>
 #include <iostream>
 #include <jrl/dynamics/dynamicsfactory.hh>

 namespace dynamicsJRLJapan
 {
   /*! \name Display related functions. 
      @{
    */

   /*! \name Display mathematical objects
     @{ */

   /*! \brief filter precision for display. */
   double filterprecision(double adb);

   /*! \brief Display horizontally in os a 3d vector. */
   void dvd3d(vector3d &av3d, std::ostream &os);

   /*! \brief Display a 3x3 matrix in os with a shift. */
   void dm3d(const matrix3d &todisplay, 
	     std::ostream &os,
	     std::string shifttab);

   /*! \brief Display a 4x4 matrix in os with a shift. */
   void dm4d(const matrix4d &todisplay, 
	     std::ostream &os,
	     std::string shifttab);

   /*! \brief Display a 4x4 matrix in os as a vector. */
   void dm4dv(const matrix4d &todisplay, 
	     std::ostream &os);

   /*! @} */

   /*! \name Display robotic objects
     @{ */

   /*! \brief Display a body of the robot */
   void DisplayBody(CjrlBody *aBody,
		    std::string &shifttab,
		    std::ostream &tcout);

   /*! \brief Display a hand of the robot */
   void DisplayHand(CjrlHand *ajrlHand,
		    std::string &shifttab,
		    std::ostream &tcout);

   /*! \brief Display a foot of the robot */
   void DisplayFoot(CjrlFoot *aFoot,
		    std::string &shifttab,
		    std::ostream &tcout);

   /*! \brief Display actuated */
   void DisplayActuated(CjrlHumanoidDynamicRobot *aHDR,
			std::string &shifttab,
			std::ostream &tcout);

   /*! \brief Display forces */
   void DisplayForces(CjrlHumanoidDynamicRobot *aHDR, 
		      std::string &shifttab, 
		      std::ostream &tcout);

   /*! \brief Display torques */
   void DisplayTorques(CjrlHumanoidDynamicRobot *aHDR, 
		       std::string &shifttab, 
		       std::ostream &tcout);

   /*! \brief Display linear velocity */
   void DisplayLinearVelocity(CjrlHumanoidDynamicRobot * aHDR,
				  std::ostream &tcout);

   /*! \brief Display angular velocity */
   void DisplayAngularVelocity(CjrlHumanoidDynamicRobot * aHDR,
				  std::ostream &tcout);



   /*! \brief Display linear acceleration */
   void DisplayLinearAcceleration(CjrlHumanoidDynamicRobot * aHDR,
				  std::ostream &tcout);

   /*! \brief Display angular acceleration */
   void DisplayAngularAcceleration(CjrlHumanoidDynamicRobot * aHDR,
				  std::ostream &tcout);


  /*! \brief Display full humanoid information */
  void DisplayHumanoid(CjrlHumanoidDynamicRobot *aHDR,
		       std::ostream &tcout);

  /*! \brief Display all the joints of a kinematic subtree */
  void RecursiveDisplayOfJoints(CjrlJoint *aJoint, 
				std::ostream &tcout,
				unsigned int verbosedisplay=0,
				unsigned int ldepth=0);

  /*! \brief Display a matrix following an imposed format. */
  void DisplayMatrix(MAL_MATRIX(,double) &aJ,std::ostream &os);

  /*! \brief Display a matrix following an imposed format. */
  void DisplayMatrix(const MAL_MATRIX(,double) &aJ, std::ostream &os);
  /*!  @} */

  /*! \brief Compare two files for testing, and report in the third one */
  bool CompareTwoFiles(char *RefFileName,
		       char *OurFileName,
		       char *ReportFileName);
  /*! @} */
};
#endif
