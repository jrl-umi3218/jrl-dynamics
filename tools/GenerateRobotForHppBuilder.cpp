/* @doc \file Object to generate a class deriving from dynamicsJRLJapan and HPP.

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
#include "GenerateRobotForHppBuilder.h"

using namespace std;

namespace dynamicsJRLJapan {

  Tools::GenerateRobotForHppBuilder::GenerateRobotForHppBuilder()
  {
    m_Verbosity=0;
  }

  Tools::GenerateRobotForHppBuilder::~GenerateRobotForHppBuilder()
  {
  }


  /*! \brief Generate hard coded robot */
  void Tools::GenerateRobotForHppBuilder::GenerateRobot(std::string &RobotName,
							CjrlHumanoidDynamicRobot *aHDR)
  {
    m_HDR = aHDR;
    ofstream aof;
    string FileName = RobotName + ".cpp";
    aof.open((char *)FileName.c_str(),ofstream::out);

    if(!aof.is_open())
      return;

    string shifttab="";
    GenerateBodies(aof,shifttab);
    aof.close();
  }


  void Tools::GenerateRobotForHppBuilder::GenerateBodies(ostream &os,
							 string shifttab)
  {
    CjrlJoint *RootJoint = m_HDR->rootJoint();
    string lshifttab  = shifttab+"  ";
    string lshifttab2 = lshifttab+"  ";
    unsigned int gindex=0;

    GenerateBody(RootJoint,os, lshifttab2,gindex);
    
  }
  
  /*! \brief Take the links towards the geometrical information */
  void Tools::GenerateRobotForHppBuilder::SetAccessToData(std::vector<BodyGeometricalData> &AccessToData)
  {
    m_AccessToData = AccessToData;
  }

  void Tools::GenerateRobotForHppBuilder::GenerateJoint(CjrlJoint *aJoint, 
							ostream &os,
							string shifttab, unsigned int &gindex)
  {

    string JointName;
    unsigned int aric = aJoint->rankInConfiguration();
    JointName = "RANK_" + aric;
    const matrix4d anInitMat = aJoint->initialPosition();
    if (aJoint->numberDof()==6)
      {
	os << "createFreeFlyer(std::string(\"RANK_" 
	   << aric
	   << "\")," << endl;
	os << "        fillMat4(1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1));"<< endl;
      }
    else if (aJoint->numberDof()==1)
      {
	os << "createRotation(std::string(\"RANK_" 
	   << aJoint->rankInConfiguration()
	   << "\")," << endl;
	os << "\tfillMat4(";
	for(unsigned int i=0;i<4;i++)
	  for(unsigned int j=0;j<4;j++)
	    {
	      os << MAL_S4x4_MATRIX_ACCESS_I_J(anInitMat,i,j);
	      if ((i!=3) || (j!=3))
		os << ",  ";
	    }
	os << "));" << endl;

	os << "hppJoint_->bounds(0," << aJoint->lowerBound(0)<< " ," << aJoint->upperBound(0) << ");" << endl;
	os << "hppJoint_->isBounded(0, true);" << endl;
	os << "hppJoint_->velocityBounds(0," 
	   << aJoint->lowerVelocityBound(0) << " , " 
	   << aJoint->upperVelocityBound(0) << ");" << endl;
	CjrlJoint *ParentJoint = aJoint->parentJoint();
	if (ParentJoint!=0)
	  {
	    os << "addChildJoint(std::string(\"RANK_" << ParentJoint->rankInConfiguration() 
	       << "\"),std::string(\"RANK_" << aric << "\"));" << endl; 
	  }
      }

    
    // Set root joint if needed.
    if (aJoint->numberDof()==6)
      {
	os << "setRootJoint(std::string(\"RANK_"
	   << aJoint->rankInConfiguration()
	   << "\"));" << endl;
      }
    
  }  


  void Tools::GenerateRobotForHppBuilder::SetPathToModelFiles(std::string &Path)
  {
    m_PathToModelFiles = Path;
  }
  void Tools::GenerateRobotForHppBuilder::CopyGeometricInformation(ostream &os,
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

  void Tools::GenerateRobotForHppBuilder::GenerateTriangles(CjrlJoint *aJoint, 
							    ostream &os,
							    string shifttab,
							    unsigned int &gindex)
  {
    
    // Specify triangles.
    unsigned int ajric = aJoint->rankInConfiguration();
    os << "unsigned int itab" << ajric << "[]={";
    const std::vector< Geometry::Shape  > & Shapes = m_AccessToData[gindex].getShapes();
    unsigned long int lNbFaces = 0;
    unsigned long int lCorrectionFaces =0;
    for(unsigned int iShape=0;
	iShape< Shapes.size();
	iShape++)
      {
	const Geometry::IndexedFaceSet & aIFS = Shapes[iShape].getIndexedFaceSet();
	const std::vector<Geometry::polygonIndex> & polygonIndex =  aIFS.coordIndex;
	if (m_Verbosity>2)
	  cout << "Number of polygons:" << aIFS.coordIndex.size() << endl;
	for(unsigned int iPolygon=0;
	    iPolygon<polygonIndex.size();
	    iPolygon++)
	  {
	    const std::vector<int> lIndex = polygonIndex[iPolygon];

	    if (lIndex.size()==0)
	      lCorrectionFaces++;

	    for(unsigned int i=0;i<lIndex.size();i++)
	      {
		os << lIndex[i] << " , " ;
	      }
	    os << endl;
	  }
	lNbFaces+=  Shapes[iShape].getIndexedFaceSet().coordIndex.size();
      }
    os << "0 };" << endl;

    os << "for (unsigned int i=0;i<" 
       << lNbFaces-lCorrectionFaces
       << ";i++) {" << endl;
    os << "\t hppPolyhedron->CkcdPolyhedron::addTriangle(itab" << ajric 
       << "[3*i],itab" << ajric << "[3*i+1],itab" << ajric << "[3*i+2],rank);" 
       <<endl << "}" << endl;
  }

  void Tools::GenerateRobotForHppBuilder::GeneratePoints(CjrlJoint *aJoint, 
							 ostream &os,
							 string shifttab,
							 unsigned int &gindex)
  {
    unsigned int ajric = aJoint->rankInConfiguration();

    // Generate points
    os << "double dtab" << ajric << "[]={";
    const std::vector< Geometry::Shape  > & Shapes = m_AccessToData[gindex].getShapes();
    unsigned long int lNbPoints = 0;
    unsigned long int lNbPolyhedron = 0;
    if (m_Verbosity>2)
      cout << "New polyhedron" << endl;
    for(unsigned int iShape=0;
	iShape< Shapes.size();
	iShape++)
      {
	const Geometry::IndexedFaceSet & aIFS = Shapes[iShape].getIndexedFaceSet();
	if (m_Verbosity>2)
	  cout << "aIFS.size:" << aIFS.coord.size() << endl;
	const std::vector<vector3d> & VecOfvec3d =  aIFS.coord;
	for(unsigned int i=0;i<VecOfvec3d.size();i++)
	  {
	    vector3d avec = VecOfvec3d[i];
	      os << avec(0) << " ," 
		 << avec(1) << " ," 
		 << avec(2) << " ,";
	      if (i==VecOfvec3d.size()-1)
		os << " /* End of polygon " 
		   << lNbPolyhedron++ << " " 
		   << lNbPoints << " " 
		   << VecOfvec3d.size() << " " 
		   << " */" ;
	      os << endl;
	  }
	lNbPoints+=  VecOfvec3d.size();
       
      }
    os << " 0.0};" << endl;
    os << "for (unsigned int i=0;i<" 
       << lNbPoints
       << ";i++) {" << endl;
    os << "\t hppPolyhedron->CkcdPolyhedron::addPoint(dtab" << ajric 
       << "[3*i],dtab" << ajric << "[3*i+1],dtab" << ajric << "[3*i+2],rank);" 
       <<endl << "}" << endl;
  }


  void Tools::GenerateRobotForHppBuilder::GenerateMaterial(CjrlJoint *aJoint, 
							   ostream &os,
							   string shifttab,
							   unsigned int &gindex)
  {

    // Generate material
    const std::vector< Geometry::Shape  > & Shapes = m_AccessToData[gindex].getShapes();
    unsigned long int lNbPoints = 0;
    for(unsigned int iShape=0;
	iShape< Shapes.size();
	iShape++)
      {
	const Geometry::Appearance & anAppearance = Shapes[iShape].getAppearance();
	const Geometry::Material & aMaterial = anAppearance.getMaterial();
	double maxdiffuseColor = aMaterial.diffuseColor[0];
	if (aMaterial.diffuseColor[0] < aMaterial.diffuseColor[1])
	  {
	    if (aMaterial.diffuseColor[1] < aMaterial.diffuseColor[2])
	      maxdiffuseColor = aMaterial.diffuseColor[2];
	    else
	      maxdiffuseColor = aMaterial.diffuseColor[1];
	  }
	else
	  {
	    if (aMaterial.diffuseColor[0] < aMaterial.diffuseColor[2])
	       maxdiffuseColor = aMaterial.diffuseColor[2];
	  }
	      
	os << "material.diffuseColor(CkppColor(" 
	   << aMaterial.diffuseColor[0] << " , " 
	   << aMaterial.diffuseColor[1] << " , " 
	   << aMaterial.diffuseColor[2] << " , "
	   << maxdiffuseColor << "));" << endl;

	os << "materialVector.push_back(material);" << endl;
	
      }

    lNbPoints = 0;
    for(unsigned int iShape=0;
	iShape< Shapes.size();
	iShape++)
      {
	unsigned int lSizeOfIFS = Shapes[iShape].getIndexedFaceSet().coordIndex.size();
	os << "hppPolyhedron->setMaterial("<< lNbPoints 
	   << " , "<< lNbPoints+lSizeOfIFS-1 << " ," 
	   << "materialVector[ " << iShape << "]);" << endl;
	
	lNbPoints+=lSizeOfIFS;
      }
    os<< "hppPolyhedron->makeCollisionEntity();" << endl;
  }


  void Tools::GenerateRobotForHppBuilder::GenerateBodyData(CjrlJoint *aJoint, 
							   ostream &os,
							   string shifttab,
							   unsigned int &gindex)
  {

    CjrlBody *aBody= aJoint->linkedBody();
    if (aBody==0)
      return;
    os << "hppBody = ChppBody::create(std::string(\"RANK_"
       << aJoint->rankInConfiguration() << "\"));"<< endl;
    os << "hppBody->mass("<< aBody->mass() << ");"<< endl;
    const matrix3d & aI = aBody->inertiaMatrix();

    os << "hppBody->inertiaMatrix(matrix3d(";
    for(unsigned int li=0;li<3;li++)
      for(unsigned int lj=0;lj<3;lj++)
	{
	  os << MAL_S3x3_MATRIX_ACCESS_I_J(aI,li,lj);
	  if ((li!=2) || (lj!=2))
	    os << ", ";
	}
    os << "));" << endl;
    
    vector3d lcom = aBody->localCenterOfMass();
    for(unsigned int li=0;li<3;li++)
      os << "MAL_S3_VECTOR_ACCESS(hppBodyRelCom," << li << ") = " << lcom(li)<< ";"<< endl;
    os << "hppBody->localCenterOfMass(hppBodyRelCom);" << endl;
    os << "hppJoint_->setAttachedBody(hppBody);" << endl;
    os << "hppBody->addInnerObject(CkppSolidComponentRef::create(hppPolyhedron)," << endl;
    os << "        fillMat4(1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1),true);"<< endl;
    
    
  }
  
  void Tools::GenerateRobotForHppBuilder::GenerateBody(CjrlJoint *aJoint, 
						       ostream &os,
						       string shifttab,
						       unsigned int &gindex)
  {

    CjrlBody *aBody= aJoint->linkedBody();
    if (aBody==0)
      return;

    GenerateJoint(aJoint,os,shifttab,gindex);
    // Geometrical part.
    os << "hppPolyhedron = CkppKCDPolyhedron::create(std::string(\"";
    os << "RANK_" << aJoint->rankInConfiguration();
    os << "\"));"<< endl;
    
    GeneratePoints(aJoint,os, shifttab, gindex);
    GenerateTriangles(aJoint,os, shifttab, gindex);    
    GenerateMaterial(aJoint,os, shifttab, gindex);    
    GenerateBodyData(aJoint,os, shifttab, gindex);    

    gindex++;
    // Call the sons.
    for(unsigned int i=0;i<aJoint->countChildJoints();i++)
      {
	GenerateBody(aJoint->childJoint(i),os,shifttab,gindex);
      }

  }

}

