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
							 string shifttab,
							 string &JointName)
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
      }
    else if (aJoint->numberDof()==1)
      {
	os << "createRotation(std::string(\"RANK_" 
	   << aJoint->rankInConfiguration()
	   << "\")," << endl;
	os << "hppJoint_->bounds(" << aJoint->lowerBound(aric)<< " ," << aJoint->upperBound(aric) << ")" << endl;
	os << "hppJoint_->isBounded(0, true);" << endl;
	os << "hppJoint_->velocityBounds(" 
	   << aJoint->lowerVelocityBound(aric) << " , " 
	   << aJoint->upperVelocityBound(aric) << ");" << endl;
	CjrlJoint *ParentJoint = aJoint->parentJoint();
	if (ParentJoint!=0)
	  {
	    os << "addChildJoint(std::string(\"RANK_" << ParentJoint->rankInConfiguration() 
	       << "\"),std::string(\"RANK_" << aric << "));" << endl; 
	  }
      }

    os << "\tfillMat4(";
    for(unsigned int i=0;i<4;i++)
      for(unsigned int j=0;j<4;j++)
	os << MAL_S4x4_MATRIX_ACCESS_I_J(anInitMat,i,j) << ",  ";
    os << "));" << endl;
    
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
    const std::vector< Geometry::Shape * > & Shapes = m_AccessToData[gindex].getShapes();
    std::vector< Geometry::Shape *>::iterator  it_Shapes = Shapes.begin();
    unsigned long int lNbFaces = 0;
    while(it_Shapes!=m_AccessToData[gindex].getShapes().end())
      {
	
	std::vector<Geometry::polygonIndex>::iterator it_int =  it_Shapes.getIndexedFaceSet().coordIndex.begin();
	while(it_int!= Shapes.getIndexedFaceSet().coordIndex.end())
	  {
	    for(unsigned int i;i<(*it_int).size();i++)
	      {
		os << it_int[i] << " , " ;
	      }
	    os << endl;
	    it_int++;
	  }
	lNbFaces+=  it_Shapes.getIndexedFaceSet().coordIndex.size();
	it_Shapes++;
      }
    os << "0 };" << endl;

    os << "for (unsigned int i=0;i<" 
       << lNbFaces
       << ";i++)" << endl;
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
    std::vector< Geometry::Shape >::iterator  it_Shapes = m_AccessToData[gindex].getShapes().begin();
    unsigned long int lNbPoints = 0;
    while(it_Shapes!=m_AccessToData[gindex].getShapes().end())
      {
	std::vector<vector3d>::iterator it_vec3d =  it_Shapes.getIndexedFaceSet().coord.begin();
	while(it_vec3d!= it_Shapes.getIndexedFaceSet().coord.end())
	  {
	    vector3d avec = (*it_vec3d);
	      os << avec(0) << " ," 
		 << avec(1) << " ," 
		 << avec(2) << " ," << endl;
	    it_vec3d++;
	  }
	lNbPoints+=  it_Shapes.getIndexedFaceSet().coord.size();
	it_Shapes++;
      }
    os << " 0.0};" << endl;
    os << "for (unsigned int i=0;i<" 
       << lNbPoints
       << ";i++)" << endl;
    os << "\t hppPolyhedron->CkcdPolyhedron::addPoint(dtab" << ajric 
       << "[3*i],dtab" << ajric << "[3*i+1],dtab" << ajric << "[3*i+2],rank);" 
       <<endl << "}" << endl;
  }


  void Tools::GenerateRobotForHppBuilder::GenerateMaterial(CjrlJoint *aJoint, 
							   ostream &os,
							   string shifttab,
							   unsigned int &gindex)
  {

    unsigned int ajric = aJoint->rankInConfiguration();

    // Generate material
    std::vector< Geometry::Shape >::iterator  it_Shapes = m_AccessToData[gindex].getShapes().begin();
    unsigned long int lNbPoints = 0;
    while(it_Shapes!=m_AccessToData[gindex].getShapes().end())
      {
	Material & aMaterial = (*it_Shapes).getAppearance();
	os << "material.diffuseColor(CkppColor(" 
	   << aMaterial.diffuseColor[0] << " , " 
	   << aMaterial.diffuseColor[1] << " , " 
	   << aMaterial.diffuseColor[2] << "));" << endl;
	
	os << "materialVector.push_back(material);" << endl;
	
	it_Shapes++;
      }

    it_Shapes = m_AccessToData[gindex].getShapes().begin();
    unsigned long int lNbPoints = 0,li=0;
    while(it_Shapes!=m_AccessToData[gindex].getShapes().end())
      {
	unsigned int lSizeOfIFS = (*it_Shapes).getIndexedFaceSet().coord.size();
	os << "hppPolyhedron->setMaterial("<< lNbPoints 
	   << " , "<< lNbPoints+lSizeOfIFS 
	   << "materialVector[ << " << li++ << "]);" << endl;
	
	lNbPoints+=lSizeOfIFS;
	it_Shapes++;
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
    os << " hppBody = ChppBody::create(std::string(\"RANK_"
       << aJoint->rankInConfiguration() << "\"));"<< endl;
    os << "hppBody->mass("<< aBody->mass() << ");"<< endl;
    const matrix3d & aI = aBody->matrix3d();

    os << "hppBody->inertiaMatrix(";
    for(unsigned int li=0;li<3;li++)
      for(unsigned int lj=0;lj<3;lj++)
	{
	  os << aI(li,lj);
	  if ((li!=2) || (lj!=3))
	    os << ", ";
	}
    os << ");" << endl;
    
    const vector3d & lcom = aBody->localCenterOfMass();
    for(unsigned int li=0;li<3;li++)
      os << "MAL_S3_VECTOR_ACCESS(hppBodyRelCom," << li << ") = " << lcom(li)<< endl;
    os << "hppBody->localCenterOfMass(hppBodyRelCom);" << endl;
    os << "hppJoint_->setAttachedBody(hppBody);" << endl;
    os << "hppBody->addInnerObject(CkppSolidComponentRef::create(hppPolyhedron)," << endl;
    os << "        fillMat4(1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1),true);"
    
    
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
    os << "hppPolyhedron = CkppKCDPolyh::create(std::string(\"";
    os << "\"));"<< endl;
    
    GeneratePoints(aJoint,os, shifttab, gindex);
    GenerateTriangles(aJoint,os, shifttab, gindex);    
    GenerateMaterial(aJoint,os, shifttab, gindex);    
    
    gindex++;
    // Call the sons.
    for(unsigned int i=0;i<aJoint->countChildJoints();i++)
      {
	GenerateBody(aJoint->childJoint(i),os,shifttab,gindex);
      }

  }

}

