
/* @doc \file Object to generate a file following AMELIF format.

   Copyright (c) 2010
   @author Olivier Stasse
   JRL-Japan, CNRS/AIST
 
   All rights reserved.

   Please refers to file License.txt for details on the license.

*/
#include "GenerateRobotForAMELIF.h"

using namespace std;

namespace {
	double sign(double val)
	{
		if(val < 0.0)
			return -1.0;

		return 1.0;
	}
}

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

  void GenerateRobotForAMELIF::ComputeEulerAngles(const matrix4d &R,
	  vector3d &u)
  {
	  double nz_ay = static_cast<double>(R[5] - R[7]);
	  double ax_sz = static_cast<double>(R[6] - R[2]);
	  double sy_nx = static_cast<double>(R[1] - R[3]);

	  double ctheta = 0.5 * (static_cast<double>(R[0] + R[4] + R[8]) - 1.0);
	  double stheta = 0.5 * sqrt(nz_ay*nz_ay + ax_sz*ax_sz + sy_nx*sy_nx);
	  double theta  = atan2(stheta, ctheta);

	  double denom = 1.0 - ctheta;
	  if(denom > 1e-24) {
		  u[0] = -sign(nz_ay) * sqrt(abs((static_cast<double>(R[0]) - ctheta)	/ denom));
		  u[1] = -sign(ax_sz) * sqrt(abs((static_cast<double>(R[4]) - ctheta)	/ denom));
		  u[2] = -sign(sy_nx) * sqrt(abs((static_cast<double>(R[8]) - ctheta) / denom));
	  }
	  else {
		  theta = 0.0;
		  u[0] = 1.0;
		  u[1] = 0.0;
		  u[2] = 0.0;
	  }
	  u *= theta;
  }


  void GenerateRobotForAMELIF::StaticParameters(CjrlJoint *aJoint,
						ostream &os,
						string &shifttab)
  {
    matrix4d FromCurrentToParent(0);
    
    if (aJoint->parentJoint()==0)
      FromCurrentToParent = aJoint->initialPosition();
    else
      {
	matrix4d FromParentToWorld = aJoint->parentJoint()->initialPosition();
	matrix4d FromCurrentToWorld = aJoint->initialPosition();	
	matrix4d FromWorldToParent;
	MAL_S4x4_INVERSE(FromParentToWorld,FromWorldToParent,double);
	
	MAL_S4x4_C_eq_A_by_B(FromCurrentToParent,FromCurrentToWorld,FromWorldToParent);
      }
    vector3d EulerAngles;
    ComputeEulerAngles(FromCurrentToParent, EulerAngles);

    os << shifttab
       << "  <StaticParameters> "
       << MAL_S4x4_MATRIX_ACCESS_I_J(FromCurrentToParent,0,3) << " "
       << MAL_S4x4_MATRIX_ACCESS_I_J(FromCurrentToParent,1,3) << " "
       << MAL_S4x4_MATRIX_ACCESS_I_J(FromCurrentToParent,2,3) << " "
       << EulerAngles(0) * 180.0/M_PI << " " 
       << EulerAngles(1) * 180.0/M_PI << " " 
       << EulerAngles(2) * 180.0/M_PI
       << "</StaticParameters>"
	   << std::endl;
  }
  
  void GenerateRobotForAMELIF::GenerateJoint(CjrlJoint *aJoint, 
					     ostream &os,
					     string shifttab, unsigned int &gindex)
  {
	os.precision(15);
	if (aJoint->rankInConfiguration() == 0)
	{
		gindex++;
		// Call the sons.
		for(unsigned int i=0;i<aJoint->countChildJoints();i++)
			GenerateJoint(aJoint->childJoint(i),os,shifttab,gindex);
		return;
	}

    // Joint name and type.
    os << shifttab << "<Joint id=\""      
       << aJoint->rankInConfiguration() ;
    if (gindex==0)
      os << "\" type=\"revolute\"";
    else
      os << "\" type=\"revolute\"";
    
    os << " axis=\"x\" " ;

    if (aJoint->parentJoint()!=0)
      os << "innerId=\""<<aJoint->parentJoint()->rankInConfiguration();
    
    os << "\" outerId=\""<<aJoint->rankInConfiguration()
       << "\"> " << endl;

    // Label
    os << shifttab << "  <Label>JOINT_" 
       << aJoint->rankInConfiguration() 
       << "  </Label>" << endl;
  
    // Static parameters
    StaticParameters(aJoint, os, shifttab);
  
    // Position min and max.
    os << shifttab << "  <PositionMin>"
       << (aJoint->lowerBound(0)  * 180.0/M_PI)
       <<"</PositionMin>" << endl;
  
    os << shifttab << "  <PositionMax>"
       << (aJoint->upperBound(0) * 180.0/M_PI)
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
	os.precision(15);
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

	// Mass
	const double mass = aBody->mass();
    os << shifttab << "  <Mass>"<< mass <<"</Mass>" << endl;

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
    aof << shifttab << "<Root id=\"0\" />" << endl;
    GenerateBodies(aof, shifttab);
    GenerateJoints(aof, shifttab);
    aof << "</MultiBody>" << endl;

    aof.close();
  }
};
