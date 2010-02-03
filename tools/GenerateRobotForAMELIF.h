/* @doc \file Object to generate a file following AMELIF format.

   Copyright (c) 2010
   @author Olivier Stasse
   JRL-Japan, CNRS/AIST
 
   All rights reserved.

   Please refers to file License.txt for details on the license.

*/
#ifndef _DYNAMIC_JRL_JAPAN_ROBOT_FOR_AMELIF_H_
#define _DYNAMIC_JRL_JAPAN_ROBOT_FOR_AMELIF_H_

#include <string>
#include <ostream>
#include <fstream>
#include <vector>

#include <dynamicsJRLJapan/dynamicsJRLJapanFactory.h>


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

    
    std::vector<BodyGeometricalData> m_AccessToData;
      
  };
};
#endif /* _DYNAMIC_JRL_JAPAN_HARD_CODED_ROBOT_H_ */
