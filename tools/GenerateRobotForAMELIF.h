/*
 * Copyright 2010, 
 *
 * Francois Keith
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
/* @doc \file Object to generate a file following AMELIF format. */

#ifndef _DYNAMIC_JRL_JAPAN_ROBOT_FOR_AMELIF_H_
#define _DYNAMIC_JRL_JAPAN_ROBOT_FOR_AMELIF_H_

#include <string>
#include <ostream>
#include <fstream>
#include <vector>

#include <jrl/dynamics/dynamicsfactory.hh>


namespace dynamicsJRLJapan
{
  class GenerateRobotForAMELIF
  {
  public:
    /*! \brief Default constructor */
    GenerateRobotForAMELIF();

    /*! \brief Default destructor */
    ~GenerateRobotForAMELIF();

    /*! \brief Generate hard coded robot */
    void GenerateRobot(std::string &RobotName,
		       CjrlHumanoidDynamicRobot *aHDR);

    /*! \brief Take the links towards the geometrical information */
    void SetAccessToData(std::vector<BodyGeometricalData> &AccessToData);
  private:
    
    CjrlHumanoidDynamicRobot *m_HDR;

    void Header(std::ostream &os);

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
    
    void StaticParameters(CjrlJoint *aJoint,
			  std::ostream &os,
			  std::string &shifttab);
    
    std::vector<BodyGeometricalData> m_AccessToData;
      
  };
}
#endif /* _DYNAMIC_JRL_JAPAN_HARD_CODED_ROBOT_H_ */
