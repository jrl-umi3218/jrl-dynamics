
/* @doc \file Object to generate a file following AMELIF format.

   Copyright (c) 2010
   @author Olivier Stasse
   JRL-Japan, CNRS/AIST
 
   All rights reserved.

   Please refers to file License.txt for details on the license.

*/
#include "GenerateRobotForAMELIF.h"

using namespace std;
namespace dynamicsJRLJapan {
  GenerateRobotForAMELIF::GenerateRobotForAMELIF()
  {
  }

  GenerateRobotForAMELIF::~GenerateRobotForAMELIF()
  {
  }

  void GenerateRobotForAMELIF::Header(ostream &os)
  {
    os << "<!DOCTYPE MultiBody SYSTEM \"./AMELIFMultiBody.xsd\">" << endl;
  }


  void GenerateRobotForAMELIF::GenerateJoint(CjrlJoint *aJoint, 
					     ostream &os,
					     string shifttab, unsigned int &gindex)
  {
    // Joint name and type.
    os << shifttab << "<Joint id=\""      
       << aJoint->rankInConfiguration() ;
    if (gindex==0)
      os << "\" type=\"revolute\"";
    else
      os << "\" type=\"revolute\"";
    
    os << " axis=\"x\" " 
       << "innerID=\""<<aJoint->rankInConfiguration() 
       << "\" outerID=\""<<aJoint->rankInConfiguration()+1
       << "\"> " << endl;
  
    // Label
    os << shifttab << "  <Label>JOINT_" 
       << aJoint->rankInConfiguration() 
       << "  </Label>" << endl;
  

    // Position min and max.
    os << shifttab << "  <PositionMin>"
       << aJoint->lowerBound(0) 
       <<"</PositionMin>" << endl;
  
    os << shifttab << "  <PositionMax>"
       << aJoint->upperBound(0)
       << "</PositionMax>" << endl;

    // Close the joint description
    os << shifttab << "</Joint>"<< endl;

    gindex++;

    // Call the sons.
    for(unsigned int i=0;i<aJoint->countChildJoints();i++)
      {
	GenerateJoint(aJoint->childJoint(i),os,shifttab,gindex);
      }

  }

  void GenerateRobotForAMELIF::GenerateBody(CjrlJoint *aJoint, 
					    ostream &os,
					    string shifttab,
					    unsigned int &gindex)
  {
    CjrlBody *aBody= aJoint->linkedBody();
    if (aBody==0)
      return;

    os << shifttab << "<Body id=\"" 
       << aJoint->rankInConfiguration()
       << "\">" << endl;

    // Label
    os << shifttab << "  <Label>LINK_"  
       << aJoint->rankInConfiguration()
       << "</Label>" << endl;

    // CoM
    const  vector3d lcom = aBody->localCenterOfMass();
    os << shifttab << "  <CoM>"  
       << MAL_S3_VECTOR_ACCESS(lcom,0) << " " 
       << MAL_S3_VECTOR_ACCESS(lcom,1) << " "
       << MAL_S3_VECTOR_ACCESS(lcom,2) << "</CoM>" << endl;

    // Inertia
    os << shifttab << "  <Inertia>" ;
    matrix3d Inertia = aBody->inertiaMatrix();

    for(unsigned int i=0;i<3;i++)
      for(unsigned int j=0;j<3;j++)
	os << MAL_S3x3_MATRIX_ACCESS_I_J(Inertia,i,j) << " ";
    os << "</Inertia>" << endl;
    // Geometric file.
    os << shifttab << "  <File>" << m_AccessToData[gindex] << "</File>" << endl;

    gindex++;
    // Close body description.
    os << shifttab << "</Body>" << endl;

    // Call the sons.
    for(unsigned int i=0;i<aJoint->countChildJoints();i++)
      {
	GenerateBody(aJoint->childJoint(i),os,shifttab,gindex);
      }
  
  }

  void GenerateRobotForAMELIF::SetAccessToData(vector<std::string> &AccessToData)
  {
    m_AccessToData = AccessToData;
  }

  void GenerateRobotForAMELIF::GenerateBodies(ostream &os,
					      string shifttab)
  {
    CjrlJoint *RootJoint = m_HDR->rootJoint();
    string lshifttab = shifttab+"  ";
    os << shifttab << "<Bodies>" << endl;
    unsigned int gindex=0;
    GenerateBody(RootJoint,os, lshifttab,gindex);
    os << shifttab << "</Bodies>" << endl;
  }

  void GenerateRobotForAMELIF::GenerateJoints(ostream &os,
					      string shifttab)
  {
    CjrlJoint *RootJoint = m_HDR->rootJoint();
    string lshifttab = shifttab+"  ";
    os << shifttab << "<Joints>" << endl;
    unsigned int gindex=0;
    GenerateJoint(RootJoint,os, lshifttab,gindex);
    os << shifttab << "</Joints>" << endl;
  }

  void GenerateRobotForAMELIF::GenerateRobot(std::string &RobotName,
					     CjrlHumanoidDynamicRobot *aHDR)
  {
    m_HDR = aHDR;
    ofstream aof;
  
    string FileName = RobotName;
    FileName += ".xml";
  
    aof.open(FileName.c_str(),fstream::out);
    if (!aof.is_open())
      return;

    Header(aof);

    string shifttab;
    aof << "<MultiBody>" << endl;
    shifttab = "  ";
    aof << shifttab << "<Root id=\"0\">" << endl;
    GenerateBodies(aof, shifttab);
    GenerateJoints(aof, shifttab);
    aof << "</MultiBody>" << endl;

    aof.close();
  }
};
