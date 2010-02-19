
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

  void GenerateRobotForAMELIF::ComputeEulerAngles(matrix4d &aTransformation,
						   vector3d &EulerAngles)
  {
    EulerAngles(0)= 0.0;
    EulerAngles(1)= 0.0;
    EulerAngles(2)= 0.0;
    
    EulerAngles(1) = asin(MAL_S4x4_MATRIX_ACCESS_I_J(aTransformation,2,0));
    double r = cos(EulerAngles(1));

    if (fabs(r)<1e-8)
      {
	EulerAngles(2) = atan2(-MAL_S4x4_MATRIX_ACCESS_I_J(aTransformation,0,1),
			       MAL_S4x4_MATRIX_ACCESS_I_J(aTransformation,0,2));
      }
    else
      {
	EulerAngles(0) = atan2(MAL_S4x4_MATRIX_ACCESS_I_J(aTransformation,2,1)/r,
			       MAL_S4x4_MATRIX_ACCESS_I_J(aTransformation,2,2)/r);	

	EulerAngles(2) = atan2(MAL_S4x4_MATRIX_ACCESS_I_J(aTransformation,0,0)/r,
			       MAL_S4x4_MATRIX_ACCESS_I_J(aTransformation,0,1)/r);	
      }
  }

  void GenerateRobotForAMELIF::StaticParameters(CjrlJoint *aJoint,
						ostream &os,
						string &shifttab)
  {
    matrix4d LocalTransformation;
    
    if (aJoint->parentJoint()!=0)
      LocalTransformation = aJoint->initialPosition();
    else
      {
	matrix4d FromParentToWorld = aJoint->parentJoint()->initialPosition();
	matrix4d FromCurrentToWorld = aJoint->initialPosition();	
	matrix4d FromWorldToParent;
	MAL_S4x4_INVERSE(FromParentToWorld,FromWorldToParent,double);
	
	MAL_S4x4_C_eq_A_by_B(LocalTransformation,FromWorldToParent,FromCurrentToWorld);
      }
    
    vector3d EulerAngles;
    ComputeEulerAngles(LocalTransformation,
		       EulerAngles);

    os << "<StaticParameters>"    
       << MAL_S4x4_MATRIX_ACCESS_I_J(LocalTransformation,0,3) << " "
       << MAL_S4x4_MATRIX_ACCESS_I_J(LocalTransformation,1,3) << " "
       << MAL_S4x4_MATRIX_ACCESS_I_J(LocalTransformation,2,3) << " "
       << EulerAngles(0) << " " 
       << EulerAngles(1) << " " 
       << EulerAngles(2)
       << "</StaticParameters>";
    
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
    
    os << " axis=\"x\" " ;
    if (aJoint->rankInConfiguration() ==0)
      os << "innerID=\"0";
    else if (aJoint->parentJoint()!=0)
      os << "innerID=\""<<aJoint->parentJoint()->rankInConfiguration();
    
    os << "\" outerID=\""<<aJoint->rankInConfiguration()+1
       << "\"> " << endl;

    // Static parameters
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
    os << shifttab << "  <File>" << m_AccessToData[gindex].getURL() << "</File>" << endl;

    gindex++;
    // Close body description.
    os << shifttab << "</Body>" << endl;

    // Call the sons.
    for(unsigned int i=0;i<aJoint->countChildJoints();i++)
      {
	GenerateBody(aJoint->childJoint(i),os,shifttab,gindex);
      }
  
  }

  void GenerateRobotForAMELIF::SetAccessToData(vector<BodyGeometricalData> &AccessToData)
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
