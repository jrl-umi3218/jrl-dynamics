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
/* @doc \file Object to generate a file following VRML 2.0 format. */
/* System includes */
#include <iostream>
#include <fstream>

/*! Includes of the framework */
#include "GenerateRobotForVRML2.h"

using namespace std;

namespace dynamicsJRLJapan {

  Tools::GenerateRobotForVRML2::GenerateRobotForVRML2()
  {
  }

  Tools::GenerateRobotForVRML2::~GenerateRobotForVRML2()
  {
  }

  void RotationMatrixFromThetaU(vector3d &axis,
				double &angle,
				matrix3d &rmat)
  {
    double ux = axis(0),
      uy = axis(1), uz = axis(2);
    double c = cos(angle), s = sin(angle);

    MAL_S3x3_MATRIX_ACCESS_I_J(rmat,0,0) =
      ux*ux + (1-ux*ux)*c;
    
    MAL_S3x3_MATRIX_ACCESS_I_J(rmat,0,1) =
      ux*uy*(1-c) - uz * s;

    MAL_S3x3_MATRIX_ACCESS_I_J(rmat,0,2) =
      ux*uz*(1-c) + uy * s;

    MAL_S3x3_MATRIX_ACCESS_I_J(rmat,1,0) =
      ux*uy*(1-c) + uz*s;
    
    MAL_S3x3_MATRIX_ACCESS_I_J(rmat,1,1) =
      uy*uy+ (1-uy*uy) * c;

    MAL_S3x3_MATRIX_ACCESS_I_J(rmat,1,2) =
      uy*uz*(1-c) - ux * s;

    MAL_S3x3_MATRIX_ACCESS_I_J(rmat,2,0) =
      ux*uz*(1-c) - uy*s;
    
    MAL_S3x3_MATRIX_ACCESS_I_J(rmat,2,1) =
      uy*uz*(1-c) + ux*s;

    MAL_S3x3_MATRIX_ACCESS_I_J(rmat,2,2) =
      uz*uz+ (1-uz*uz) * c;
    

  }

  void Tools::GenerateRobotForVRML2::AxisAngle2(matrix4d &data,
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
    double r = sqrt(MAL_S3_VECTOR_ACCESS(axis,0)*MAL_S3_VECTOR_ACCESS(axis,0) +
		    MAL_S3_VECTOR_ACCESS(axis,2)*MAL_S3_VECTOR_ACCESS(axis,2) +
		    MAL_S3_VECTOR_ACCESS(axis,1)*MAL_S3_VECTOR_ACCESS(axis,1))/2.0;
    
    double t = (Qxx + Qyy +Qzz-1.0)/2.0;

    angle =atan2(r,t);

    if ((r>1e-8) || (t>0.0))
      {
	double sinca;

	if (angle<1e-8)
	  sinca = 1.0;
	else
	  sinca = r/angle;

	MAL_S3_VECTOR_ACCESS(axis,0) = MAL_S3_VECTOR_ACCESS(axis,0)/(2.0*sinca);
	MAL_S3_VECTOR_ACCESS(axis,1) = MAL_S3_VECTOR_ACCESS(axis,1)/(2.0*sinca);
	MAL_S3_VECTOR_ACCESS(axis,2) = MAL_S3_VECTOR_ACCESS(axis,2)/(2.0*sinca);
      }
    else
      {
	MAL_S3_VECTOR_ACCESS(axis,0) = sqrt((Qxx - t)/(1-t));
	if ((Qzy - Qyz)<0.0)
	  MAL_S3_VECTOR_ACCESS(axis,0) = -MAL_S3_VECTOR_ACCESS(axis,0);
	
	MAL_S3_VECTOR_ACCESS(axis,1) = sqrt((Qyy - t)/(1-t));
	if ((Qxz - Qzx)<0.0)
	  MAL_S3_VECTOR_ACCESS(axis,1) = -MAL_S3_VECTOR_ACCESS(axis,1);

	
	MAL_S3_VECTOR_ACCESS(axis,2) = sqrt((Qzz - t)/(1-t));
	if ((Qyx - Qxy)<0.0)
	  MAL_S3_VECTOR_ACCESS(axis,2) = -MAL_S3_VECTOR_ACCESS(axis,2);

	if ((r==0) && (angle==M_PI))
	  {
	    if ((MAL_S3_VECTOR_ACCESS(axis,2)==0.0) && 
		(Qyx==Qxy))
	      {
		if (MAL_S3_VECTOR_ACCESS(axis,0)!=0)
		  MAL_S3_VECTOR_ACCESS(axis,0) = -MAL_S3_VECTOR_ACCESS(axis,0);
	      }
	    
	    if ((MAL_S3_VECTOR_ACCESS(axis,0)==0.0) && 
		(Qzy==Qyz))
	      {
		if (MAL_S3_VECTOR_ACCESS(axis,1)!=0)
		  MAL_S3_VECTOR_ACCESS(axis,1) = -MAL_S3_VECTOR_ACCESS(axis,1);
	      }

	    if ((MAL_S3_VECTOR_ACCESS(axis,1)==0) && 
		(Qxz==Qzx))
	      {
		if (MAL_S3_VECTOR_ACCESS(axis,2)!=0)
		  MAL_S3_VECTOR_ACCESS(axis,2) = -MAL_S3_VECTOR_ACCESS(axis,2);
	      }
	  }
      }
    {
      matrix3d check;
      RotationMatrixFromThetaU(axis,angle,check);
      
    }
  }
  
  void Tools::GenerateRobotForVRML2::AxisAngle(matrix4d &data,
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

  /*! \brief Generate hard coded robot */
  void Tools::GenerateRobotForVRML2::GenerateRobot(std::string &RobotName,
			    CjrlHumanoidDynamicRobot *aHDR)
  {
    m_HDR = aHDR;
    ofstream aof;
    string FileName = RobotName + "_v1.wrl";
    aof.open((char *)FileName.c_str(),ofstream::out);

    if(!aof.is_open())
      return;

    aof << "#VRML V2.0 utf8" << endl;
    aof << endl;
    string shifttab="";
    GenerateBodies(aof,shifttab);
    aof.close();
  }


  void Tools::GenerateRobotForVRML2::GenerateBodies(ostream &os,
						    string shifttab)
  {
    CjrlJoint *RootJoint = m_HDR->rootJoint();
    string lshifttab  = shifttab+"  ";
    string lshifttab2 = lshifttab+"  ";
    unsigned int gindex=0;

    GenerateJoint(RootJoint,os,shifttab,gindex);
    os << lshifttab << "children [" << endl;
    GenerateBody(RootJoint,os, lshifttab2,gindex);
    os << lshifttab << "]" << endl;
    os << shifttab << "}" << endl;  

  }
  
  /*! \brief Take the links towards the geometrical information */
  void Tools::GenerateRobotForVRML2::SetAccessToData(std::vector<BodyGeometricalData> &AccessToData)
  {
    m_AccessToData = AccessToData;
  }

  void Tools::GenerateRobotForVRML2::GenerateJoint(CjrlJoint *aJoint, 
					     ostream &os,
					     string shifttab, unsigned int &gindex)
  {

    os << shifttab << "Transform { "<< endl;
    
    MAL_S4x4_MATRIX(aTransformation,double);
    aTransformation = aJoint->currentTransformation();

    matrix4d initialTr;
    initialTr = aJoint->initialPosition();
    MAL_S4x4_MATRIX_ACCESS_I_J(initialTr,0,3) = 0.0;
    MAL_S4x4_MATRIX_ACCESS_I_J(initialTr,1,3) = 0.0;
    MAL_S4x4_MATRIX_ACCESS_I_J(initialTr,2,3) = 0.0;

    matrix4d invrot,invInitialTr;
    MAL_S4x4_INVERSE(initialTr,invInitialTr,double);
    
    MAL_S4x4_C_eq_A_by_B(invrot,aTransformation,invInitialTr);

    for(unsigned int i=0;i<4;i++)
      for(unsigned int j=0;j<4;j++)
	MAL_S4x4_MATRIX_ACCESS_I_J(aTransformation,i,j) =
	  MAL_S4x4_MATRIX_ACCESS_I_J(invrot,i,j);

    // Multiply by the local rotation of the joint reference frame
    matrix3d RotationForDisplay3d;
    RotationForDisplay3d = m_AccessToData[gindex].getRotationForDisplay();
    //MAL_S3x3_MATRIX_SET_IDENTITY(RotationForDisplay3d);
    MAL_S4x4_MATRIX(aTransformationForDisplay,double);

    for(unsigned int i=0;i<3;i++)
      for(unsigned int j=0;j<3;j++)
	{
	  MAL_S4x4_MATRIX_ACCESS_I_J(aTransformationForDisplay,i,j) = 0.0;
	  for(unsigned int k=0;k<3;k++)
	    {
	      MAL_S4x4_MATRIX_ACCESS_I_J(aTransformationForDisplay,i,j)+=
		MAL_S4x4_MATRIX_ACCESS_I_J(aTransformation,i,k)*
		MAL_S3x3_MATRIX_ACCESS_I_J(RotationForDisplay3d,k,j);
 	    }
	}

    os << shifttab << "  translation ";
    for(unsigned int i=0;i<3;i++)
      os << MAL_S4x4_MATRIX_ACCESS_I_J(aTransformation,i,3) << " ";
    os << endl;
    os << shifttab << "  rotation " ;
    vector3d laxis;
    double angle;
    AxisAngle2(aTransformationForDisplay,laxis,angle);
    for(unsigned int i=0;i<3;i++)
      os << MAL_S3_VECTOR_ACCESS(laxis,i) << " ";

    os << angle << endl;

  }  


  void Tools::GenerateRobotForVRML2::SetPathToModelFiles(std::string &Path)
  {
    m_PathToModelFiles = Path;
  }
  void Tools::GenerateRobotForVRML2::CopyGeometricInformation(ostream &os,
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

  void Tools::GenerateRobotForVRML2::GenerateBody(CjrlJoint *aJoint, 
						  ostream &os,
						  string shifttab,
						  unsigned int &gindex)
  {

    CjrlBody *aBody= aJoint->linkedBody();
    if (aBody==0)
      return;

    GenerateJoint(aJoint,os,shifttab,gindex);

        // Geometric file.
    os << shifttab << "  children [" << endl;
    const std::vector< std::string > & urls = m_AccessToData[gindex].getURLs();
    for(unsigned i=0; i < urls.size(); ++i)
      {
	os << shifttab << "    Inline {"<< endl
	   << shifttab << "      url \"" 
	   << urls[i] << "\"" << endl
	   << shifttab << "    }" << endl;
      }
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

