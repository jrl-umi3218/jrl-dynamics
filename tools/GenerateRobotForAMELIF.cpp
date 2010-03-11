
/* @doc \file Object to generate a file following AMELIF format.

   Copyright (c) 2010
   @author Olivier Stasse
   JRL-Japan, CNRS/AIST
 
   All rights reserved.

   Please refers to file License.txt for details on the license.

*/
#include "GenerateRobotForAMELIF.h"
#include <cmath>

using namespace std;

namespace {
	double FilterPrecision(double x)
	{
		x *= 1e6;
		x =floor(x+0.5);
		x *= 1e-6;
		return x;
	}

	void ComputeEulerAngles(const matrix3d &aRotationMatrix, vector3d &EulerAngles)
	{
		double r11 = MAL_S3x3_MATRIX_ACCESS_I_J(aRotationMatrix,0,0);
		double r12 = MAL_S3x3_MATRIX_ACCESS_I_J(aRotationMatrix,0,1);
		double r13 = MAL_S3x3_MATRIX_ACCESS_I_J(aRotationMatrix,0,2);

		double r21 = MAL_S3x3_MATRIX_ACCESS_I_J(aRotationMatrix,1,0);

		double r31 = MAL_S3x3_MATRIX_ACCESS_I_J(aRotationMatrix,2,0);
		double r32 = MAL_S3x3_MATRIX_ACCESS_I_J(aRotationMatrix,2,1);
		double r33 = MAL_S3x3_MATRIX_ACCESS_I_J(aRotationMatrix,2,2);


		if (fabs(fabs(r31)-1.0)>1e-8)
		{

			double Y1 = -asin(r31);
			double Y2 = M_PI - Y1;
			Y2 = fmod(Y2,M_PI);
			double c0_1 = cos(Y1);
			double c0_2 = cos(Y2);

			double X1 = atan2(r32/c0_1,r33/c0_1);
			double X2 = atan2(r32/c0_2,r33/c0_2);

			double c0;
			if (fabs(X1)<fabs(X2))
			{
				EulerAngles(1) = Y1;
				EulerAngles(0) = X1;
				c0 = c0_1;
			}
			else 
			{
				EulerAngles(1) = Y2;
				EulerAngles(0) = X2;
				c0 = c0_2;
			}

			EulerAngles(2) = atan2(r21/c0,r11/c0);

		}
		else
		{
			EulerAngles(2) = 0;
			double d = atan2(r12,r13);
			d = fmod(d,M_PI);
			if (fabs(r31+1.0)<1e-8)
			{
				EulerAngles(1) = M_PI/2;
				EulerAngles(0) = d;
			}
			else
			{
				EulerAngles(1) = -M_PI/2;
				EulerAngles(0) = -d;
			}
		}
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

  void GenerateRobotForAMELIF::StaticParameters(CjrlJoint *aJoint,
						ostream &os,
						string &shifttab)
  {
    MAL_S4x4_MATRIX(aTransformation,double);
    MAL_S4x4_MATRIX(FinalTransformation,double);
    aTransformation = aJoint->currentTransformation();
    unsigned int indexparent=0;

    /* Compute transformation in local coordinates */
    CjrlJoint *parentJoint = aJoint->parentJoint();
    if (parentJoint!=0)
      {
	MAL_S4x4_MATRIX(parentTransformation,double);
	parentTransformation = parentJoint->currentTransformation();

	MAL_S4x4_MATRIX(invParentTransformation,double);
	MAL_S4x4_INVERSE(parentTransformation,invParentTransformation,double);
	MAL_S4x4_C_eq_A_by_B(FinalTransformation,invParentTransformation,aTransformation);
      }
    else
      FinalTransformation = aTransformation;

    /* Project rotation axis */

    matrix3d aRotation;
    vector3d anAxis, EulerAngles;
    for(unsigned int i=0;i<3;i++)
      {
	for(unsigned int j=0;j<3;j++)
	  MAL_S3x3_MATRIX_ACCESS_I_J(aRotation,i,j) = 
	    MAL_S4x4_MATRIX_ACCESS_I_J(FinalTransformation,i,j);
      }

    ComputeEulerAngles(aRotation, EulerAngles);

    os << shifttab
       << "  <StaticParameters> "
       << MAL_S4x4_MATRIX_ACCESS_I_J(FinalTransformation,0,3) << " "
       << MAL_S4x4_MATRIX_ACCESS_I_J(FinalTransformation,1,3) << " "
       << MAL_S4x4_MATRIX_ACCESS_I_J(FinalTransformation,2,3) << " "
       << FilterPrecision(EulerAngles(0) * 180.0/M_PI) << " " 
       << FilterPrecision(EulerAngles(1) * 180.0/M_PI) << " " 
       << FilterPrecision(EulerAngles(2) * 180.0/M_PI)
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
       << FilterPrecision(aJoint->lowerBound(0)  * 180.0/M_PI)
       <<"</PositionMin>" << endl;
  
    os << shifttab << "  <PositionMax>"
       << FilterPrecision(aJoint->upperBound(0) * 180.0/M_PI)
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
