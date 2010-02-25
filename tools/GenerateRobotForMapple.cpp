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
#include "GenerateRobotForMapple.h"

using namespace std;

namespace dynamicsJRLJapan {

  Tools::GenerateRobotForMapple::GenerateRobotForMapple()
  {
  }

  Tools::GenerateRobotForMapple::~GenerateRobotForMapple()
  {
  }

  void Tools::GenerateRobotForMapple::AxisAngle2(matrix4d &data,
					       vector3d &axis,
					       double &angle) const
  {

    MAL_S3_VECTOR_FILL(axis,0.0);
    angle = 0;
    
    double Qxx = MAL_S4x4_MATRIX_ACCESS_I_J(data,0,0);
    double Qxy = MAL_S4x4_MATRIX_ACCESS_I_J(data,0,1);
    double Qxz = MAL_S4x4_MATRIX_ACCESS_I_J(data,0,2);
    double Qyx = MAL_S4x4_MATRIX_ACCESS_I_J(data,1,0);
    double Qyy = MAL_S4x4_MATRIX_ACCESS_I_J(data,1,1);
    double Qyz = MAL_S4x4_MATRIX_ACCESS_I_J(data,1,2);
    double Qzx = MAL_S4x4_MATRIX_ACCESS_I_J(data,2,0);
    double Qzy = MAL_S4x4_MATRIX_ACCESS_I_J(data,2,1);
    double Qzz = MAL_S4x4_MATRIX_ACCESS_I_J(data,2,2);

    MAL_S3_VECTOR_ACCESS(axis,0) = Qzy - Qyz;
    MAL_S3_VECTOR_ACCESS(axis,1) = Qxz - Qzx;
    MAL_S3_VECTOR_ACCESS(axis,2) = Qyx - Qxy;
    double lr =sqrt(MAL_S3_VECTOR_ACCESS(axis,2)*MAL_S3_VECTOR_ACCESS(axis,2) 
		    + MAL_S3_VECTOR_ACCESS(axis,1)*MAL_S3_VECTOR_ACCESS(axis,1));
    double r = sqrt(MAL_S3_VECTOR_ACCESS(axis,0)*MAL_S3_VECTOR_ACCESS(axis,0)
	     + lr*lr);
    if (r==0.0)
      {
	if ((Qxx==-1.0) && (Qyy==-1.0))
	  {
	    MAL_S3_VECTOR_ACCESS(axis,0)= 0.0;
	    MAL_S3_VECTOR_ACCESS(axis,1)= 0.0;
	    MAL_S3_VECTOR_ACCESS(axis,2)= 1.0;
	    angle=M_PI;
	  }
	else if ((Qxx==-1.0) && (Qzz==-1.0))
	  {
	    MAL_S3_VECTOR_ACCESS(axis,0)= 0.0;
	    MAL_S3_VECTOR_ACCESS(axis,1)= 1.0;
	    MAL_S3_VECTOR_ACCESS(axis,2)= 0.0;
	    angle=M_PI;
	  }
	else if ((Qyy==-1.0) && (Qzz==-1.0))
	  {
	    MAL_S3_VECTOR_ACCESS(axis,0)= 1.0;
	    MAL_S3_VECTOR_ACCESS(axis,1)= 0.0;
	    MAL_S3_VECTOR_ACCESS(axis,2)= 0.0;
	    angle=M_PI;
	  }
	else if ((Qyy==1.0) && (Qzz==1.0))
	  {
	    MAL_S3_VECTOR_ACCESS(axis,0)= 1.0;
	    MAL_S3_VECTOR_ACCESS(axis,1)= 0.0;
	    MAL_S3_VECTOR_ACCESS(axis,2)= 0.0;
	    angle=0;
	  }
	else 
	  {
	    cout.precision(10);
	    cout<< "Pb. " 
		<< fabs(Qxx)-1.0 << " "
		<< fabs(Qyy)-1.0 << " "
		<< fabs(Qzz)-1.0 << " "
		<< endl;
	  }
	return;
      }
    else
      {
	MAL_S3_VECTOR_ACCESS(axis,0) /=r;
	MAL_S3_VECTOR_ACCESS(axis,1) /=r;
	MAL_S3_VECTOR_ACCESS(axis,2) /=r;
	angle=0.0;
      }
    double t = Qxx + Qyy +Qzz;

    angle =atan2(r,t-1.0);
  }
  
  void Tools::GenerateRobotForMapple::AxisAngle(matrix4d &data,
					       vector3d &axis,
					       double &angle) const
  {
    MAL_S3_VECTOR_FILL(axis,0.0);
    angle = 0;
    double x,y,z;
    //    cout << "data:" << data << endl;
    
    double  d0 = MAL_S4x4_MATRIX_ACCESS_I_J(data,0,0);
    double d01 = MAL_S4x4_MATRIX_ACCESS_I_J(data,0,1);
    double d02 = MAL_S4x4_MATRIX_ACCESS_I_J(data,0,2);
    double d10 = MAL_S4x4_MATRIX_ACCESS_I_J(data,1,0);
    double  d1 = MAL_S4x4_MATRIX_ACCESS_I_J(data,1,1);
    double d12 = MAL_S4x4_MATRIX_ACCESS_I_J(data,1,2);
    double d20 = MAL_S4x4_MATRIX_ACCESS_I_J(data,2,0);
    double d21 = MAL_S4x4_MATRIX_ACCESS_I_J(data,2,1);
    double  d2 = MAL_S4x4_MATRIX_ACCESS_I_J(data,2,2);

    double epsilon = 0.01; // margin to allow for rounding errors
    double epsilon2 = 0.1; // margin to distinguish between 0 and 180 degrees
    
    if ((fabs(d01 - d10)<epsilon) &&
	(fabs(d02 - d20)<epsilon) &&
	(fabs(d12 - d21)<epsilon))
      {
	if ((fabs(d01 - d10)<epsilon2) &&
	    (fabs(d02 - d20)<epsilon2) &&
	    (fabs(d12 - d21)<epsilon2))
	  {
	    MAL_S3_VECTOR_ACCESS(axis,0)= 1.0;MAL_S3_VECTOR_ACCESS(axis,1)= 0.0;MAL_S3_VECTOR_ACCESS(axis,2)= 0.0;
	    angle=0.0;
	    return;
	  }
	angle = M_PI;
	double xx = (d0+1)/2;
	double yy = (d1+1)/2;
	double zz = (d2+1)/2;
	double xy = (d01+d10)/4;
	double xz = (d02+d20)/4;
	double yz = (d12+d21)/4;
	if ((xx > yy) && (xx > zz)) { // m[0][0] is the largest diagonal term
	  if (xx< epsilon) {
	    x = 0;
	    y = 0.7071;
	    z = 0.7071;
	  } else {
	    x = sqrt(xx);
	    y = xy/x;
	    z = xz/x;
	  }
	} else if (yy > zz) { // m[1][1] is the largest diagonal term
	  if (yy< epsilon) {
	    x = 0.7071;
	    y = 0;
	    z = 0.7071;
	  } else {
	    y = sqrt(yy);
	    x = xy/y;
	    z = yz/y;
	  }	
	} else { // m[2][2] is the largest diagonal term so base result on this
	  if (zz< epsilon) {
	    x = 0.7071;
	    y = 0.7071;
	    z = 0;
	  } else {
	    z = sqrt(zz);
	    x = xz/z;
	    y = yz/z;
	  }
	}
	MAL_S3_VECTOR_ACCESS(axis,0) = x;
	MAL_S3_VECTOR_ACCESS(axis,1) = y;
	MAL_S3_VECTOR_ACCESS(axis,2) = z;
	return;
      }
    double s = sqrt( (d21 - d12)*(d21 -d12 ) + 
		     (d02 - d20)*(d02 - d20) +
		     (d10 - d01)*(d10 - d01));
    if (fabs(s)<0.001) s =1.0;
    double r = (d0+d1+d2-1.0)*0.5 ;
    //    cout << "r:" << r << endl;
    angle = acos(r);
    //angle = 0;
    s = 2*sin(angle);
    //    cout << "angle:" << angle<<endl;
    MAL_S3_VECTOR_ACCESS(axis,0) = (d21 - d12)/s;
    MAL_S3_VECTOR_ACCESS(axis,1) = (d02 - d20)/s;
    MAL_S3_VECTOR_ACCESS(axis,2) = (d10 - d01)/s;

    return;
  }

  void Tools::GenerateRobotForMapple::GenerateGPLv2License(ostream &aof)
  {
    aof << "# Copyright (C) "<<    
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
    aof << "NDDL := " << aHDR->numberDof() << endl;
    
    aof << "# Definitions of coordinates and generalized speeds " << endl;
    aof << "q := vector(NDDL):" << endl;
    aof << "qdot := vector(NDDL):" << endl;
    
    aof << "#Cardan rotation: we begin with rotation around x, and after around y and" << endl;
    aof << "#finally around z." << endl;

    

  }
  /*! \brief Generate hard coded robot */
  void Tools::GenerateRobotForMapple::GenerateRobot(std::string &RobotName,
			    CjrlHumanoidDynamicRobot *aHDR)
  {
    m_HDR = aHDR;
    ofstream aof;
    string FileName = RobotName + "Data.maple";
    aof.open((char *)FileName.c_str(),ofstream::out);

    if(!aof.is_open())
      return;

    GenerateGPLv2License(aof);
    GenerateHeader(aof,aHDR);

    string empty("");
    GenerateJoints(aof,empty);

    aof.close();
  }


  void Tools::GenerateRobotForMapple::GenerateJoints(ostream &os,
						     string shifttab,
						     CjrlHumanoidDynamicRobot *aHDR)
  {
    CjrlJoint *RootJoint = m_HDR->rootJoint();
    unsigned int gindex=1;

    m_Indexes[RootJoint] = gindex;
    GenerateJoint(RootJoint,os,shifttab,gindex);
    
    IndexDeBase = aHDR->numberDof()+1;
    os << "#Frame "<< gindex << endl;
    os << "Rx_"<< gindex << " := q["<< IndexBase+3<< "]:" << endl;
    os << "Ry_"<< gindex << " :=-q["<< IndexBase+5<< "]:" << endl;
    os << "Rz_"<< gindex << " := q["<< IndexBase+4<< "]:" << endl;
    os << "Tx_"<< gindex << " := q["<< IndexBase<< "]:" << endl;
    os << "Ty_"<< gindex << " :=-q["<< IndexBase+2<< "]:" << endl;
    os << "Tz_"<< gindex << " := q["<< IndexBase+1<< "]:" << endl;

    os << shifttab << "children [" << endl;
    GenerateBody(RootJoint,os, lshifttab,gindex);
    os << shifttab << "]" << endl;

  }
  
  /*! \brief Take the links towards the geometrical information */
  void Tools::GenerateRobotForMapple::SetAccessToData(std::vector<BodyGeometricalData> &AccessToData)
  {
    m_AccessToData = AccessToData;
  }

  void Tools::GenerateRobotForMapple::GenerateJoint(CjrlJoint *aJoint, 
					     ostream &os,
					     string shifttab, unsigned int &gindex)
  {

    m_Indexes[aJoint] = gindex;
    os << "#Frame "<< gindex << endl;
    
    MAL_S4x4_MATRIX(aTransformation,double);
    aTransformation = aJoint->currentTransformation();
    unsigned int indexparent=0;

    jrlJoint *parentJoint = aJoint->parentJoint();
    if (parentJoint!=0)
      {
	MAL_S4x4_MATRIX(parentTransformation,double);
	parentTransformation = parentJoint->currentTransformation();

	MAL_S4x4_MATRIX(invParentTransformation,double);
	MAL_S4x4_INVERSE(parentTransformation,invParentTransformation,double);
	indexparent = m_Indexes[parentJoint];
      }
    aTransformation = MAL_S4x4_RET_A_by_B(invParentTransformation,aTransformation);

    matrix3d aRotation;
    vector3d anAxis, aRealAxis;
    for(unsigned int i=0;i<3;i++)
      {
	for(unsigned int j=0;j<3;j++)
	  MAL_S3x3_ACCESS_I_J(aRotation,i,j) = MAL_S4x4_ACCESS_I_J(aTransformation,i,j);
      }
    anAxis(0) = 1.0; anAxis(1) = 0.0; anAxis(2) = 0.0;
    MAL_S3x3_C_eq_A_by_B(aRealAxis,aRotation,anAxis);
    
    os << "ref_"<<gindex << " := "<< indexparent << ":" << endl;
    if (fabs(fabs(aRealAxis(0))-1.0)<1e-8)
      os << "Rx_"<< gindex << " := q["<< aJoint->rankInConfiguration() << "]:" << endl;
    else 
      os << "Rx_"<< gindex << " := 0:" << endl;
  }  


  void Tools::GenerateRobotForMapple::SetPathToModelFiles(std::string &Path)
  {
    m_PathToModelFiles = Path;
  }
  void Tools::GenerateRobotForMapple::CopyGeometricInformation(ostream &os,
							      string FileName)
  {
    string FinalFileName = m_PathToModelFiles+ FileName;
    std::ifstream geometricFile((char *)FinalFileName.c_str(),ifstream::in);
    if (!geometricFile.is_open())
      {
	cerr << "Unable to open " << FinalFileName << endl;
	return;
      }
    // Go through the geometric file
    while(!geometricFile.eof())
      {
	char geometricline[25536];
	geometricFile.getline(geometricline,25536);
	string aline(geometricline);
	os.write(geometricline,aline.length());
      }
    geometricFile.close();

  }

  void Tools::GenerateRobotForMapple::GenerateBody(CjrlJoint *aJoint, 
						  ostream &os,
						  string shifttab,
						  unsigned int &gindex)
  {

    CjrlBody *aBody= aJoint->linkedBody();
    if (aBody==0)
      return;

    GenerateJoint(aJoint,os,shifttab,gindex);

        // Geometric file.
    os << shifttab << "  children [" << endl
       << shifttab << "    Inline {"<< endl
       << shifttab << "      url \"" 
       << m_AccessToData[gindex].getURL() << "\"" << endl
       << shifttab << "    }" << endl;
    
      //    CopyGeometricInformation(os,m_AccessToData[gindex]);
    os << shifttab << "  ]"<< endl;;
    os << shifttab << "}" << endl;  

    gindex++;
    // Call the sons.
    for(unsigned int i=0;i<aJoint->countChildJoints();i++)
      {
	GenerateBody(aJoint->childJoint(i),os,shifttab,gindex);
      }

  }


}

