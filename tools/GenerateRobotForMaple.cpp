/* @doc \file Object to generate a file following VRML 1.0 format.

   Copyright (c) 2010
   @author Olivier Stasse
   JRL-Japan, CNRS/AIST
 
   All rights reserved.

   Please refers to file License.txt for details on the license.

*/
/* System includes */
#include <iostream>
#include <fstream>

/*! Includes of the framework */
#include "GenerateRobotForMaple.h"

#define ODEBUG3(x) \
  cout << x << endl;

using namespace std;

namespace dynamicsJRLJapan {

  Tools::GenerateRobotForMaple::GenerateRobotForMaple()
  {
  }

  Tools::GenerateRobotForMaple::~GenerateRobotForMaple()
  {
  }

  double Tools::GenerateRobotForMaple::FilterPrecision(double x)
  {
    if (fabs(x)<1e-8)
      return 0.0;

    return x;
  }

  void Tools::GenerateRobotForMaple::GenerateGPLv2License(ostream &aof)
  {
    aof << "# Copyright (C) "<< endl   
	<< "# This program is free software; you can redistribute it and/or modify it" << endl
	<< "# under the terms of the GNU General Public License version 2 as published" << endl
	<< "# by the Free Software Foundation." << endl
	<< "#" << endl
	<< "# This program is distributed in the hope that it will be useful, but" << endl
	<< "# WITHOUT ANY WARRANTY; without even the implied warranty of" << endl
	<< "# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General" << endl
	<< "# Public License for more details." << endl
	<< "# " << endl
	<< "# You should have received a copy of the GNU General Public License along" << endl
	<< "# with this program; if not, write to the Free Software Foundation, Inc.," << endl
	<< "# 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA." << endl;
  }

  void Tools::GenerateRobotForMaple::GenerateHeader(ostream &aof,
						    CjrlHumanoidDynamicRobot *aHDR)
  {

    aof << "# Number of Degree of freedom" << endl;
    aof << "NDDL := " << aHDR->numberDof() << ":"<< endl <<endl;

    aof << "# Number of bodies" << endl;
    aof << "NSOL := " << aHDR->numberDof()-6+2 << ":" << endl <<endl;
    
    aof << "# Definitions of coordinates and generalized speeds " << endl;
    aof << "q := vector(NDDL):" << endl;
    aof << "qdot := vector(NDDL):" << endl << endl;
    
    aof << "#Cardan rotation: we begin with rotation around x, and after around y and" << endl;
    aof << "#finally around z." << endl << endl;

  }

  /*! \brief Generate hard coded robot */
  void Tools::GenerateRobotForMaple::GenerateRobot(std::string &RobotName,
						    CjrlHumanoidDynamicRobot *aHDR)
  {
    GenerateKinematicData(RobotName,aHDR);
    GenerateDynamicData(RobotName,aHDR);
    GenerateContactData(RobotName,aHDR);
  }


  void Tools::GenerateRobotForMaple::GenerateKinematicData(std::string &RobotName,
							   CjrlHumanoidDynamicRobot *aHDR)

  {

    ofstream aof;
    string FileName = RobotName + "KinematicData.maple";
    aof.open((char *)FileName.c_str(),ofstream::out);

    if(!aof.is_open())
      return;

    GenerateGPLv2License(aof);
    GenerateHeader(aof,aHDR);

    string empty("");
    GenerateJoints(aof,empty,aHDR);

    aof.close();

    
  }

  void Tools::GenerateRobotForMaple::GenerateDynamicData(std::string &RobotName,
							 CjrlHumanoidDynamicRobot *aHDR)
  {

    ofstream aof;
    string FileName = RobotName + "DynamicData.maple";
    aof.open((char *)FileName.c_str(),ofstream::out);

    if(!aof.is_open())
      return;

    GenerateGPLv2License(aof);

    GenerateBodies(aof,aHDR);

    aof.close();

    
  }

  void Tools::GenerateRobotForMaple::GenerateContactData(std::string &RobotName,
							 CjrlHumanoidDynamicRobot *aHDR)
  {

    ofstream aof;
    string FileName = RobotName + "AdditionalData.maple";
    aof.open((char *)FileName.c_str(),ofstream::out);

    if(!aof.is_open())
      return;

    GenerateGPLv2License(aof);

    GenerateContactPointFile(aof,aHDR);

    aof.close();

    
  }

  void Tools::GenerateRobotForMaple::GenerateJoints(ostream &os,
						     string shifttab,
						     CjrlHumanoidDynamicRobot *aHDR)
  {
    CjrlJoint *RootJoint = aHDR->rootJoint();
    unsigned int gindex=1;


    
    os << "# Frame 1 : rotation absolute frame (x direction to front of robot, "   << endl;
    os << "# y vertically upwards, z to the right of robot) to frame defined by " << endl;
    os << "# (x direction to front of robot, y to the left of robot, z vertically " << endl;
    os << "# upwards)" << endl;
    os << "ref_"<<gindex << " :=0:" << endl;
    os << "Rx_"<< gindex << " := -Pi/2:" << endl;
    os << "Ry_"<< gindex << " := 0:" << endl;
    os << "Rz_"<< gindex << " := 0:" << endl;
    os << "Tx_"<< gindex << " := 0:" << endl;
    os << "Ty_"<< gindex << " := 0:" << endl;
    os << "Tz_"<< gindex << " := 0:" << endl << endl;
    
    gindex++;
    m_Indexes[RootJoint] = gindex;    
    unsigned int IndexBase = aHDR->numberDof()-5;
    os << "# Frame "<< gindex << endl;
    os << "ref_"<<gindex << " := "<< gindex-1 << ":"<< endl;
    os << "Rx_"<< gindex << " := q["<< IndexBase+3<< "]:" << endl;
    os << "Ry_"<< gindex << " :=-q["<< IndexBase+5<< "]:" << endl;
    os << "Rz_"<< gindex << " := q["<< IndexBase+4<< "]:" << endl;
    os << "Tx_"<< gindex << " := q["<< IndexBase<< "]:" << endl;
    os << "Ty_"<< gindex << " :=-q["<< IndexBase+2<< "]:" << endl;
    os << "Tz_"<< gindex << " := q["<< IndexBase+1<< "]:" << endl << endl;
    
    gindex++;
    for(unsigned int i=0;i<RootJoint->countChildJoints();i++)
      {
	GenerateJoint(RootJoint->childJoint(i),os,shifttab,gindex);
      }

  }
  
  /*! \brief Take the links towards the geometrical information */
  void Tools::GenerateRobotForMaple::SetAccessToData(std::vector<BodyGeometricalData> &AccessToData)
  {
    m_AccessToData = AccessToData;
  }

  void Tools::GenerateRobotForMaple::GenerateJointFilePart(CjrlJoint *aJoint, 
							   ostream &os,
							   unsigned &indexparent, unsigned int &gindex,
							   vector3d &aRealAxis,
							   matrix4d &aTransformation)
  {
    /* Generate file */
    os << "ref_"<<gindex << " := "<< indexparent << ":" << endl;
    if (fabs(fabs(aRealAxis(0))-1.0)<1e-8)
      os << "Rx_"<< gindex << " := q["<< aJoint->rankInConfiguration()-5 << "]:"  << endl;
    else 
      os << "Rx_"<< gindex << " := 0:" << endl;

    if (fabs(fabs(aRealAxis(1))-1.0)<1e-8)
      os << "Ry_"<< gindex << " := q["<< aJoint->rankInConfiguration()-5 << "]:"  << endl;
    else 
      os << "Ry_"<< gindex << " := 0:" << endl;

    if (fabs(fabs(aRealAxis(2))-1.0)<1e-8)
      os << "Rz_"<< gindex << " := q["<< aJoint->rankInConfiguration()-5 << "]:"   << endl;
    else 
      os << "Rz_"<< gindex << " := 0:" << endl;

    os << "Tx_"<< gindex << " := " 
       << FilterPrecision(MAL_S4x4_MATRIX_ACCESS_I_J(aTransformation,0,3)) 
       <<  ":" << endl;
    os << "Ty_"<< gindex << " := " 
       << FilterPrecision(MAL_S4x4_MATRIX_ACCESS_I_J(aTransformation,1,3)) 
       <<  ":" << endl;
    os << "Tz_"<< gindex << " := " 
       << FilterPrecision(MAL_S4x4_MATRIX_ACCESS_I_J(aTransformation,2,3)) 
       << ":" << endl;
    os << endl;
    gindex++;
						       }
  void Tools::GenerateRobotForMaple::GenerateJoint(CjrlJoint *aJoint, 
						    ostream &os,
						    string shifttab, unsigned int &gindex)
  {

    m_Indexes[aJoint] = gindex;
    os << "# Frame "<< gindex << endl;
    
    MAL_S4x4_MATRIX(aTransformation,double);
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
	indexparent = m_Indexes[parentJoint];
	aTransformation = MAL_S4x4_RET_A_by_B(invParentTransformation,aTransformation);
      }

    /* Project rotation axis */

    matrix3d aRotation;
    vector3d anAxis, aRealAxis;
    for(unsigned int i=0;i<3;i++)
      {
	for(unsigned int j=0;j<3;j++)
	  MAL_S3x3_MATRIX_ACCESS_I_J(aRotation,i,j) = 
	    MAL_S4x4_MATRIX_ACCESS_I_J(aTransformation,i,j);
      }
    anAxis(0) = 1.0; anAxis(1) = 0.0; anAxis(2) = 0.0;
    MAL_S3x3_C_eq_A_by_B(aRealAxis,aRotation,anAxis);
    
    GenerateJointFilePart(aJoint, os, indexparent,gindex,
			  aRealAxis,aTransformation);
    // Call the sons.
    for(unsigned int i=0;i<aJoint->countChildJoints();i++)
      {
	GenerateJoint(aJoint->childJoint(i),os,shifttab,gindex);
      }

  }  

  void Tools::GenerateRobotForMaple::GenerateBody(CjrlJoint *aJoint,
						  ostream &os,
						  string &shifttab,
						  unsigned int &gindex)
  {

    CjrlBody *aBody = aJoint->linkedBody();
    os << "# Body " << gindex<< " Rank:" << aJoint->rankInConfiguration() << endl;
    os << "m_" << gindex << " := "<<  aBody->mass() << ":"<<endl;
    
    vector3d aCom=aBody->localCenterOfMass();
    os << "G_" << gindex 
       << " := vector(["
       << aCom(0) << " , "
       << aCom(1) << " , " 
       << aCom(2) << "]):" << endl;
    
    matrix3d IG=aBody->inertiaMatrix();
    os << "IG_" << gindex 
       << " := matrix([[" 
       << MAL_S3x3_MATRIX_ACCESS_I_J(IG,0,0) << " , " 
       << MAL_S3x3_MATRIX_ACCESS_I_J(IG,0,1) << " , " 
       << MAL_S3x3_MATRIX_ACCESS_I_J(IG,0,2) << " ],[" 
       << MAL_S3x3_MATRIX_ACCESS_I_J(IG,1,0) << " , " 
       << MAL_S3x3_MATRIX_ACCESS_I_J(IG,1,1) << " , " 
       << MAL_S3x3_MATRIX_ACCESS_I_J(IG,1,2) << " ],[" 
       << MAL_S3x3_MATRIX_ACCESS_I_J(IG,2,0) << " , " 
       << MAL_S3x3_MATRIX_ACCESS_I_J(IG,2,1) << " , " 
       << MAL_S3x3_MATRIX_ACCESS_I_J(IG,2,2) << " ]]):" 
       << endl << endl;


    gindex++;
    // Call the sons.
    for(unsigned int i=0;i<aJoint->countChildJoints();i++)
      {
	GenerateBody(aJoint->childJoint(i),os,shifttab,gindex);
      }
    
  }

  void Tools::GenerateRobotForMaple::GenerateBodies(ostream &os,
						    CjrlHumanoidDynamicRobot *aHDR)
  {
    os << endl;
    os << "Gravity := vector([0, -9.81, 0]):"  << endl << endl;
    unsigned int gindex=1;
    CjrlJoint *RootJoint = aHDR->rootJoint();
    string empty("");
    
    os << "# Ghost Body " << endl;
    os << "m_1 := 0.0:" << endl;
    os << "G_1 := vector([0.0, 0.0, 0.0]):"<< endl;
    os << "IG_1 := matrix([[0.0, 0.0, 0.0],[0.0, 0.0, 0.0],[0.0, 0.0, 0.0]]):"<< endl << endl;
    gindex++;

    GenerateBody(RootJoint,os,empty,gindex);

  }

  void Tools::GenerateRobotForMaple::ComputeContactPointsForOneFoot(CjrlHumanoidDynamicRobot *aHDR,
								    int LeftOrRight,
								    vector4d ContactPoints[4])
  {
    CjrlFoot *aFoot=0;
    if (LeftOrRight>0)
      aFoot = aHDR->leftFoot();
    else
      aFoot = aHDR->rightFoot();

    if (aFoot==0)
      return;

    matrix4d AnkleTransformation;
    const CjrlJoint *Ankle = aFoot->associatedAnkle();
    if (Ankle==0)
      return;
    
    AnkleTransformation = Ankle->currentTransformation();

    vector3d AnklePositionInLocalFrame;
    aFoot->getAnklePositionInLocalFrame(AnklePositionInLocalFrame);
    vector3d SoleCenterInLocalFrame;
    aFoot->getSoleCenterInLocalFrame(SoleCenterInLocalFrame);

    vector3d SoleCenterInAnkleFrame;
    SoleCenterInAnkleFrame = SoleCenterInLocalFrame - AnklePositionInLocalFrame;

    vector4d SoleCenterInAnkleFrame4d;
    for(unsigned int i=0;i<3;i++)
      SoleCenterInAnkleFrame4d(i) = SoleCenterInAnkleFrame(i);

    vector4d SoleCenterInWorldFrame;
    MAL_S4x4_C_eq_A_by_B(SoleCenterInWorldFrame,
			 AnkleTransformation,
			 SoleCenterInAnkleFrame4d);
    double outLength, outWidth;
    aFoot->getSoleSize(outLength,outWidth);
    ODEBUG3("Size of the foot: " << outLength << " " << outWidth) ;
    ODEBUG3("AnkleTransformation: " << AnkleTransformation);
    ODEBUG3("SoleCenterInWorldFrame: " << SoleCenterInWorldFrame);

    double LocalShift[4][2];
    LocalShift[0][0] = outLength/2.0; LocalShift[0][1] = outWidth/2.0;
    LocalShift[1][0] = outLength/2.0; LocalShift[1][1] = -outWidth/2.0;
    LocalShift[2][0] = -outLength/2.0; LocalShift[2][1] = outWidth/2.0;
    LocalShift[3][0] = -outLength/2.0; LocalShift[3][1] = -outWidth/2.0;

    for(unsigned int j=0;j<4;j++)
      {
	ContactPoints[j](0)=SoleCenterInWorldFrame(0)+LocalShift[j][0];
	ContactPoints[j](1)=SoleCenterInWorldFrame(1)+LocalShift[j][1];
	ContactPoints[j](2)=SoleCenterInWorldFrame(2);
	ContactPoints[j](3)=1.0;
      }
  }

  void Tools::GenerateRobotForMaple::GenerateContactPointsForOneFoot(std::ostream &os,
								     CjrlHumanoidDynamicRobot *aHDR,
								     int LeftOrRight,
								     vector4d ContactPoints[4],
								     unsigned int &gindex)
  {
    std::string FootSide;
    unsigned int FootRank;
    CjrlFoot *aFoot=0;
    if (LeftOrRight>0)
      {
	aFoot = aHDR->leftFoot();
	FootSide = "Left";
      }
    else
      {
	aFoot = aHDR->rightFoot();
	FootSide = "Right";
      }

    if (aFoot==0)
      return;

    const CjrlJoint *Ankle = aFoot->associatedAnkle();
    if (Ankle==0)
      return;

    FootRank = Ankle->rankInConfiguration()-4;
    for(unsigned int j=0;j<4;j++)
      {
	os << "# Tag "<< gindex << " : " << FootSide << endl;
	os << "reftag_" << gindex << " := " << FootRank << ":" << endl;
	os << "tag_"<< gindex << " := " << "vector(["
	   << ContactPoints[j][0] << ","
	   << ContactPoints[j][1] << ","
	   << ContactPoints[j][2] << "]):"<< endl << endl;
	gindex++;
      }
    
  }
  
  void Tools::GenerateRobotForMaple::GenerateContactPointFile(std::ostream &os,
							      CjrlHumanoidDynamicRobot *aHDR)
  {
    GenerateGPLv2License(os);
    os << endl;
    os << "# Definition of contact points " << endl << endl;
    os << "# Contact tags " << endl;
    os << "points_contact := [ 1, 2, 3, 4, 5, 6, 7, 8]:" << endl << endl;
    os << "# Number of contacts" << endl;
    os << "NCONT := 8:" <<endl << endl;
   
    os << "# Number of tags" << endl;
    os << "NTAG := 8:" << endl <<  endl;

    unsigned int gindex = 1;

    vector4d ContactPoints[4];

    /* Compute contact points on the left */
    ComputeContactPointsForOneFoot(aHDR, -1, ContactPoints);
    GenerateContactPointsForOneFoot(os, aHDR,
				    -1, ContactPoints,gindex);
    
    /* Compute contact points on the right */
    ComputeContactPointsForOneFoot(aHDR, 1, ContactPoints);
    GenerateContactPointsForOneFoot(os, aHDR,
				    1, ContactPoints,gindex);

  }
}

