/* @doc Object used to parse a VRML file describing a robot.

   Copyright (c) 2005-2009, 

   @author : 
   Olivier Stasse
   
   JRL-Japan, CNRS/AIST

   All rights reserved.
   
   Please refers to file License.txt for details on the license.

*/

#include <fstream>
#include <sstream>

//#define BOOST_SPIRIT_DEBUG
#define BOOST_SPIRIT_RULE_SCANNERTYPE_LIMIT 2

#define DEPTH_MAX 30
#include <map>

#include "Debug.h"

#if 0
#include <boost/spirit.hpp>
#include <boost/spirit/core.hpp>
#include <boost/spirit/phoenix/binders.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/lambda/bind.hpp>
#include <boost/spirit/actor/push_back_actor.hpp>
#include <boost/spirit/utility.hpp>
#else
#include <boost/spirit.hpp>
#include <boost/spirit/phoenix/binders.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/lambda/bind.hpp>
#endif


#include "MultiBody.h"
#include "SpiritVRMLReader.h"

using namespace std;
using namespace boost::spirit;
using namespace boost::spirit::utility;
using namespace phoenix;

//#define SVRBIND(x) bind(&SpiritVRMLReader::x,this,_1)
#define SVRBIND(x) &x
#define SVRBIND2(x,y) bind(&SpiritOpenHRP::x)y

namespace dynamicsJRLJapan 
{
  namespace VRMLReader
  {
    // Semantic actions can be functors. The operator() function is called
    // with two iterators that specify the range of input that matches the production.
    struct do_help {
      template<typename Iter>
      void operator()(Iter, Iter) const
      {
	std::cout << "help - to be implemented\n";
      }
    };

    struct do_quit {
      template<typename Iter>
      void operator()(Iter, Iter) const
      {
	std::exit(EXIT_SUCCESS);
      }
    };


    struct s_DataForParsing
    {

      // Variable to build the VRML tree.
      int NbOfBodies;

      // Depth
      int Depth;

      // Joint memory allocation done for new depth.
      bool JointMemoryAllocationForNewDepth;

      // Current Link.
      internalLink CurrentLink;

       // Vector of current branch
      vector<Body *> CurrentBody;

      // CurrentTranslation.
      MAL_S3_VECTOR(,double) JointTranslation;

      // Rotation axis and quantity.
      vector3d RotationAxis;

      // aName (for several entities)
      string aName;

      // Inertie matrix
      double mi[9];
      int index_mi;

      // Mass of the body.
      double mass;

      // Center of Mass.
      double cm[3];

      // Constructor
      s_DataForParsing()
      {
	CurrentLink.aJoint = 0;
      }

    };

    struct SkipGrammar : public grammar<SkipGrammar>
    {
      template <typename ScannerT>
      struct definition
      {
	definition(SkipGrammar const &self)
	{
	  skip_r = space_p | comment_p("#",eol_p);
	};
	
	rule<ScannerT> skip_r;
	rule<ScannerT> const& start() const {return skip_r;}
      };
    };


  
    /*! \brief Object to read a VRML file in the OpenHRP format. */
    struct SpiritOpenHRP : public grammar<SpiritOpenHRP>
    {

      void fskyColor(char const *str, char const *end) const
      {
	string s(str,end);
	if (m_Verbose>1)
	  std::cout<< "fskyColor: "<< s<< endl;  
      }
    
      void fViewpointPos(char const *str, char const *end) const
      {
	string s(str,end);
	if (m_Verbose>1)
	  std::cout<< "fViewpointPos: "<< s<< endl;  
      }
    
      void fViewpointOri(char const *str, char const *end) const
      { 
	string s(str,end);
	if (m_Verbose>1)
	  std::cout<< "fViewpointOri: "<< s<< endl;  
      }
    
      void fNI_Type(char const *str, char const *end) const
      {
	string s(str,end);
	if (m_Verbose>1)
	  std::cout<< "fNIType: "<< s<< endl;
      
      }
    
      void fNI_Headlight(char const *str, char const *end) const
      {
	string s(str,end);
	if (m_Verbose>1)
	  std::cout<< "fHeadlight: "<< s<< endl;
      
      }
    
      void fNI_AvatarSize(double s) const
      {
	if (m_Verbose>1)
	  std::cout<< "fAvataSize: "<< s<< endl;
      }
    
      void fSFInt32(char const *str, char const *end) const
      {
	string s(str,end);
	if (m_Verbose>1)
	  std::cout<< "fSFInt32: " << s<< endl;
      }
      void fTransformToField(char const *str, char const *end) const
      {
	string s(str,end);
	if (m_Verbose>1)
	  std::cout<<"fTransformToField: "<< s<< endl;
      }
    
      void fISFromNameToField(char const *str, char const *end) const
      {
	string s(str,end);
	if (m_Verbose>1)
	  std::cout<<"ISFromNameToField: "<< s<< endl;
      }
    
      void fProtoSndBlock(char const *str, char const *end) const
      {
	string s(str,end);
	if (m_Verbose>1)
	  std::cout<<"ProtoSndBlock: "<< s<< endl;
      }
    
      void fProtoBlock(char const *str, char const *end) const
      {
	string s(str,end);
	if (m_Verbose>1)
	  std::cout<<"ProtoBlock: "<< s<< endl;
      }
    
      void fNameToField(char const* str, char const* end) const
      {
	string s(str,end);
	if (m_Verbose>1)
	  std::cout<<"NameToField:" << s<< endl;
      }
      void fNameToField2(char const* str, char const* end) const
      {
	string s(str,end);
	if (m_Verbose>1)
	  std::cout<<"IS NameToField:" << s<< endl;
      }
    
      void fIgnoreComment(char const *str, char const *end) const 
      {
	string s(str,end);
	//  std::cout << "IgnoreComment :" << s << endl;
      }
    
      void fProtoLineName(char const *str, char const *end) const
      {
	string s(str,end);
	if (m_Verbose>1)
	  std::cout << "fProtoLineName :" << s << endl;
      }
    
      void ProtoType(char const *str, char const *end) const
      {
	string s(str,end);
	if (m_Verbose>1)
	  std::cout << "ProtoType :" << s << endl;
      }
    
    
      void GiveDataDefault(char const *str, char const *end) const
      {
	string s(str,end);
	if (m_Verbose>1)
	  std::cout << "GiveDataDefault :" << s << endl;
      }

      void HumanoidBody(char const *str, char const *end) const
      {
	string s(str,end);
	if (m_Verbose>1)
	  std::cout << "HumanoidBody :" << s << endl;
      }
    
      void fAssignHumanoidName(char const *str, char const *end) const
      {
	string s(str,end);
	if (m_Verbose>1)
	  std::cout << "Humanoid Name :" << s << endl;
      }

      void fHumanoidName(char const *str, char const *end) const
      {
	string s(str,end);
	if (m_Verbose>1)
	  std::cout<< "Reading the name of the Humanoid: |" << s<<"|" << endl;
      }
    
      void fProtoName(char const *str, char const *end) const
      {
	string s(str,end);
	if (m_Verbose>1)
	  std::cout<< "Reading the name of the proto: |" << s<<"|" << endl;
      }
    
      void fAddBody() const
      {
	int lDepth = m_DataForParsing->Depth;
	Body * lCurrentBody = m_DataForParsing->CurrentBody[lDepth];
	lCurrentBody->setLabel(m_DataForParsing->NbOfBodies++);
	lCurrentBody->setName((char *)(m_DataForParsing->aName).c_str());
	lCurrentBody->setInertie(m_DataForParsing->mi);
	lCurrentBody->setMasse(m_DataForParsing->mass);
	lCurrentBody->setPositionCoM(m_DataForParsing->cm);
	if (m_DataForParsing->Depth!=0)
	      lCurrentBody->setLabelMother(m_DataForParsing->CurrentBody[lDepth-1]->getLabel());
	m_MultiBody->ajouterCorps(*lCurrentBody);
	m_MultiBody->ajouterLiaison(*m_DataForParsing->CurrentBody[lDepth-1],
				    *lCurrentBody,
				    m_DataForParsing->CurrentLink);
	m_DataForParsing->JointMemoryAllocationForNewDepth=false;
	
      }
      void fJCDEFBlocks(char const *str, char const *end) const
      {
	string s(str,end);
	m_DataForParsing->aName=s;
	if (m_Verbose>1)
	  std::cout<< "Reading aName (part of the JointChildrenBlock): |" << s<<"|" << endl;
	
      }
    
      void fJointName(char const *str, char const *end) const
      {
	string s(str,end);
	m_DataForParsing->aName=s;

	
	if (m_DataForParsing->JointMemoryAllocationForNewDepth)
	  delete m_DataForParsing->CurrentLink.aJoint;
	
	m_DataForParsing->CurrentLink.aJoint = new JointPrivate();
	m_DataForParsing->JointMemoryAllocationForNewDepth=true;
	m_DataForParsing->CurrentLink.aJoint->setName(s);
	m_DataForParsing->CurrentLink.aJoint->setIDinActuated(-1);
		
	if (m_Verbose>1)
	  std::cout<< "Reading the name of the JointPrivate: |" << s<<"|" << endl;
      }

      void fJointSubBlockName() const
      {
	if (m_DataForParsing->JointMemoryAllocationForNewDepth)
	  delete m_DataForParsing->CurrentLink.aJoint;

	m_DataForParsing->CurrentLink.aJoint = new JointPrivate();
	m_DataForParsing->JointMemoryAllocationForNewDepth=true;
	m_DataForParsing->CurrentLink.aJoint->setIDinActuated(-1);
	m_DataForParsing->CurrentLink.aJoint->setName(m_DataForParsing->aName);
	if (m_Verbose>1)
	  std::cout<< "Reading the `name of the JointPrivate SubBlockName: |" << m_DataForParsing->aName<<"|" << endl;
      }

      void fBodySubBlockName() const
      {
	/*
	if (m_DataForParsing->CurrentBody[m_DataForParsing->Depth]!=0)
	  {
	    delete m_DataForParsing->CurrentBody[m_DataForParsing->Depth];
	    std::cout << "Current depth :" << m_DataForParsing->Depth << std::endl;
	    }*/
	m_DataForParsing->CurrentBody[m_DataForParsing->Depth] = new Body() ;
	m_DataForParsing->CurrentBody[m_DataForParsing->Depth]->setName((char *)m_DataForParsing->aName.c_str());
	if (m_Verbose>1)
	  std::cout<< "Reading the name of the JointPrivate: |" << m_DataForParsing->aName<<"|" << endl;
      }
    
      void fSFVec3f_0(char const *str, char const *end) const
      {
	string s(str,end);
	if (m_Verbose>1)
	  std::cout<< "SFVec3f_0" << s<<"|" << endl;
      }
    
      void fSFVec3f_1(char const *str, char const *end) const
      {
	string s(str,end);
	if (m_Verbose>1)
	  std::cout<< "SFVec3f_1" << s<<"|" << endl;
      
      }
    
      void fSFVec3f_2(char const *str, char const *end) const
      {
	string s(str,end);
	if (m_Verbose>1)
	  std::cout<< "SFVec3f_2" << s<<"|" << endl;
      }
    
      void fJointTranslationX(double x) const
      {
	m_DataForParsing->JointTranslation(0) = x;
      }
    
      void fJointTranslationY(double y) const
      {
	m_DataForParsing->JointTranslation(1) = y;
      }
    
      void fJointTranslationZ(double z) const
      {
	m_DataForParsing->JointTranslation(2) = z;

	// IMPORTANT POLICY: all the free joint have their static 
	// translation set to zero.
	if (m_DataForParsing->CurrentLink.aJoint->type()!=JointPrivate::FREE_JOINT)
	  m_DataForParsing->CurrentLink.aJoint->setStaticTranslation(m_DataForParsing->JointTranslation);
	else
	  {
	    MAL_S3_VECTOR(,double) lnull;
	    lnull(0) = lnull(1) = lnull(2) = 0.0;
	    m_DataForParsing->CurrentLink.aJoint->setStaticTranslation(lnull);
	  }
	  
	if (m_Verbose>1)
	  cout << "JointPrivate" << m_DataForParsing->CurrentLink.aJoint->getName()
	       << ":" << m_DataForParsing->JointTranslation << endl;
      }
    
      void fJointRotationX(double x)  const
      {
	m_DataForParsing->RotationAxis(0) = x;
      }
    
      void fJointRotationY(double y) const
      {
	m_DataForParsing->RotationAxis(1) = y;
      }
    
      void fJointRotationZ(double z) const
      {
	m_DataForParsing->RotationAxis(2) = z;
      }
      
      void fJointRotationAngle(double lQ) const
      {
	double lQuantity = lQ;
	matrix3d R;
	AxeAngle2Matrix(m_DataForParsing->RotationAxis, lQuantity, R);
	m_DataForParsing->CurrentLink.aJoint->setStaticRotation(R);
	if (m_Verbose>1)
	  std::cerr<< "JointRotation:" <<R << endl;
      }
      
      void fBodyMass(double mass) const
      {
	m_DataForParsing->mass = mass;
      }

      void fBodyCenterOfMassX(double x) const
      {
	m_DataForParsing->cm[0] = x;
	if (m_Verbose>1)
	  std::cout << "fBodyCenterOfMassX :" << x << endl;
	
      }

      void fBodyCenterOfMassY(double y) const
      {
	m_DataForParsing->cm[1] = y;
	if (m_Verbose>1)
	  std::cout << "fBodyCenterOfMassY :" << y <<endl;
      }

      void fBodyCenterOfMassZ(double z)  const
      {
	m_DataForParsing->cm[2] = z;

	if (m_Verbose>1)
	  std::cout << "fBodyCenterOfMassZ :" << z <<endl;
      }

      void fBodyInlineUrl(const char *str, const char *end)  const
      {
	string s(str,end);
	if (m_Verbose>1)
	  std::cout << "fBodyInlineUrl:" << s << endl;
      }

      void fJointType(char const *str, char const *end)  const
      {
	MAL_S3_VECTOR(lnull,double);
	lnull[0]=0.0;lnull[1]=0.0;lnull[2]=0.0;
  
	string s(str,end);
	if (m_Verbose>1)
	  cout << "jointType: " << s << endl;

	if (s=="free")
	  {
	    m_DataForParsing->CurrentLink.aJoint->type(JointPrivate::FREE_JOINT);
	  }
	else if (s=="rotate")
	  {
	    m_DataForParsing->CurrentLink.aJoint->type(JointPrivate::REVOLUTE_JOINT);
	  }
	  
      }

      void fJointID(int aJointID)  const
      {
	m_DataForParsing->CurrentLink.aJoint->setIDinActuated(aJointID);
	if (m_Verbose>1)
	  std::cout << "JointID :" << aJointID << endl;
      }

      void fJointXAxis(char const ach)  const
      {
	MAL_S3_VECTOR(lxaxis,double);
	lxaxis[0]=1.0;lxaxis[1]=0.0;lxaxis[2]=0.0;
	m_DataForParsing->CurrentLink.aJoint->type(JointPrivate::REVOLUTE_JOINT);
	m_DataForParsing->CurrentLink.aJoint->axe(lxaxis);
      }

      void fJointYAxis(char const end)  const
      {
	
	MAL_S3_VECTOR(lyaxis,double);
	lyaxis[0]=0.0;lyaxis[1]=1.0;lyaxis[2]=0.0;
	m_DataForParsing->CurrentLink.aJoint->type(JointPrivate::REVOLUTE_JOINT);
	m_DataForParsing->CurrentLink.aJoint->axe(lyaxis);

      }
      
      void fJointZAxis(char const end) const 
      {
	MAL_S3_VECTOR(lzaxis,double);
	lzaxis[0]=0.0;lzaxis[1]=0.0;lzaxis[2]=1.0;
	m_DataForParsing->CurrentLink.aJoint->type(JointPrivate::REVOLUTE_JOINT);
	m_DataForParsing->CurrentLink.aJoint->axe(lzaxis);

      }

      void fJointLLimit(double r) const
      {
	m_DataForParsing->CurrentLink.aJoint->lowerBound(0,r);
	if (m_Verbose>1)
	  std::cout << "fJointLLimit: "  << r << endl;
      }


      void fJointULimit(double r) const
      {
	m_DataForParsing->CurrentLink.aJoint->upperBound(0,r);
	if (m_Verbose>1)
	  std::cout << "fJointULimit: "  << r << endl;
      }


      void fJointLVLimit(double r) const
      {
	m_DataForParsing->CurrentLink.aJoint->lowerVelocityBound(0,r);
	if (m_Verbose>1)
	  std::cout << "fJointLVLimit: "  << r << endl;
      }


      void fJointUVLimit(double r) const 
      {
	m_DataForParsing->CurrentLink.aJoint->upperVelocityBound(0,r);
	if (m_Verbose>1)
	  std::cout << "fJointUVLimit: "  << r << endl;
      }

      void fJointEquivalentInertia(double r) const 
      {
	m_DataForParsing->CurrentLink.aJoint->equivalentInertia(r);
	if (m_Verbose>1)
	  std::cout << "fJointEquivalentInertia: "  << r << endl;
      }

      void fForceSensorName(char const *str, char const *end) const
      {
	string s(str,end);
	if (m_Verbose>1)
	  std::cout<< "fForceSensorName"<< endl;
      }

      void fFSTranslationX(double x) const
      {
	if (m_Verbose>1)
	  std::cout << "fFSTranslationX :" << x << endl;
      }

      void fFSTranslationY(double y) const
      {
	if (m_Verbose>1)
	  std::cout << "fFSTranslationY :" << y <<endl;
      }

      void fFSTranslationZ(double z) const
      {
	if (m_Verbose>1)
	  std::cout << "fFSTranslationZ :" << z <<endl;
      }

      void fFSRotationX(double x) const
      {
	if (m_Verbose>1)
	  std::cout << "fFSRotationX :" << x << endl;
      }

      void fFSRotationY(double y) const
      {
	if (m_Verbose>1)
	  std::cout << "fFSRotationY :" << y <<endl;
      }

      void fFSRotationZ(double z) const
      {
	if (m_Verbose>1)
	  std::cout << "fFSRotationZ :" << z <<endl;
      }

      void fFSAngle(double z) const
      {
	if (m_Verbose>1)
	  std::cout << "fFSAngle :" << z <<endl;
      }

      void fFSID(int aFSID) const
      {
	if (m_Verbose>1)
	  std::cout << "FSID :" << aFSID << endl;
      }

      void fAccelerationSensorName(char const *str, char const *end) const
      {
	string s(str,end);
	if (m_Verbose>1)
	  std::cout<< "fAccelerationSensorName"<< endl;
      }

      void fASTranslationX(double x) const
      {
	if (m_Verbose>1)
	  std::cout << "fASTranslationX :" << x << endl;
      }

      void fASTranslationY(double y) const
      {
	if (m_Verbose>1)
	  std::cout << "fASTranslationY :" << y <<endl;
      }

      void fASTranslationZ(double z) const
      {
	if (m_Verbose>1)
	  std::cout << "fASTranslationZ :" << z <<endl;
      }

      void fASRotationX(double x) const
      {
	if (m_Verbose>1)
	  std::cout << "fASRotationX :" << x << endl;
      }

      void fASRotationY(double y) const
      {
	if (m_Verbose>1)
	  std::cout << "fASRotationY :" << y <<endl;
      }

      void fASRotationZ(double z) const
      {
	if (m_Verbose>1)
	  std::cout << "fASRotationZ :" << z <<endl;
      }

      void fASAngle(double z) const
      {
	if (m_Verbose>1)
	  std::cout << "fASAngle :" << z <<endl;
      }

      void fASID(int aASID) const
      {
	if (m_Verbose>1)
	  std::cout << "ASID :" << aASID << endl;
      }

      void fGyrometerSensorName(char const *str, char const *end) const
      {
	string s(str,end);
	if (m_Verbose>1)
	  std::cout<< "fAccelerationSensorName"<< endl;
      }

      void fGyroTranslationX(double x) const
      {
	if (m_Verbose>1)
	  std::cout << "fGyroTranslationX :" << x << endl;
      }

      void fGyroTranslationY(double y) const
      {
	if (m_Verbose>1)
	  std::cout << "fGyroTranslationY :" << y <<endl;
      }

      void fGyroTranslationZ(double z) const
      {
	if (m_Verbose>1)
	  std::cout << "fGyroTranslationZ :" << z <<endl;
      }

      void fGyroRotationX(double x) const
      {
	if (m_Verbose>1)
	  std::cout << "fGyroRotationX :" << x << endl;
      }

      void fGyroRotationY(double y) const
      {
	if (m_Verbose>1)
	  std::cout << "fGyroRotationY :" << y <<endl;
      }

      void fGyroRotationZ(double z) const
      {
	if (m_Verbose>1)
	  std::cout << "fGyroRotationZ :" << z <<endl;
      }

      void fGyroAngle(double z) const
      {
	if (m_Verbose>1)
	  std::cout << "fGyroAngle :" << z <<endl;
      }

      void fGyroID(int aGyroID) const
      {
	if (m_Verbose>1)
	  std::cout << "GyroID :" << aGyroID << endl;
      }

      void fFillmomentsOfInertia(double z) const
      {
	m_DataForParsing->mi[m_DataForParsing->index_mi++]=z;
	if (m_DataForParsing->index_mi==9)
	  {
	    if (m_Verbose>1)
	      {
		for(int i=0;i<9;i++)
		  {
		    cout << m_DataForParsing->mi[i] << " ";
		    if (i%3==2)
		      cout<< endl;
		  }
	      }
	    m_DataForParsing->index_mi=0;
	  }
      }
 
      void fVisionSensorName(char const *str, char const *end) const
      {
	string s(str,end);
	if (m_Verbose>1)
	  std::cout<< "fVisionSensorName"<< endl;
      }

      void fVSTranslationX(double x) const
      {
	if (m_Verbose>1)
	  std::cout << "fVSTranslationX :" << x << endl;
      }

      void fVSTranslationY(double y) const
      {
	if (m_Verbose>1)
	  std::cout << "fVSTranslationY :" << y <<endl;
      }

      void fVSTranslationZ(double z) const
      {
	if (m_Verbose>1)
	  std::cout << "fVSTranslationZ :" << z <<endl;
      }

      void fVSRotationX(double x) const
      {
	if (m_Verbose>1)
	  std::cout << "fVSRotationX :" << x << endl;
      }

      void fVSRotationY(double y) const
      {
	if (m_Verbose>1)
	  std::cout << "fVSRotationY :" << y <<endl;
      }

      void fVSRotationZ(double z) const
      {
	if (m_Verbose>1)
	  std::cout << "fVSRotationZ :" << z <<endl;
      }

      void fVSAngle(double z) const
      {
	if (m_Verbose>1)
	  std::cout << "fVSAngle :" << z <<endl;
      }

      void fVSID(int aVSID) const
      {
	if (m_Verbose>1)
	  std::cout << "VSID :" << aVSID << endl;
      }
      
      void fIncreaseDepth() const
      {
	m_DataForParsing->Depth++;
	if (m_Verbose>1)
	  std::cout << "Increased depth "<< m_DataForParsing->Depth << endl;
      }

      void fDecreaseDepth() const
      {
	m_DataForParsing->Depth--;
	if (m_Verbose>1)
	  std::cout << "Decreased depth "<< m_DataForParsing->Depth << endl;
      }

      // The parser object is copied a lot, so instead of keeping its own table
      // of variables, it keeps track of a reference to a common table.
      SpiritOpenHRP(MultiBody *aMB,
		    struct s_DataForParsing *aDFP) : 
	m_MultiBody(aMB),
	m_DataForParsing(aDFP)
	
      {
	m_Verbose = 5;
      }
          
      template <typename ScannerT>
      struct definition
      {

	// bool ParseVRMLFile(char const *str)
      
	definition(SpiritOpenHRP const &self)
	{
      
	  // Fields to link a name and field 
	  scaleMultiple_r = str_p("scale")>> !str_p("Orientation");
	  NameToField_r = str_p("center") | str_p("children") |
	    str_p("rotation") | scaleMultiple_r |
	    str_p("translation") | str_p("bboxCenter") | str_p("bboxSize") |
	    str_p("addChildren") | str_p("removeChildren") | str_p("viewpoints") | str_p("humanoidBody") |
	    str_p("transmitter") | str_p("receiver");
      
	  TransformToField_r = (NameToField_r)[SVRBIND2(fNameToField,(self,arg1,arg2))] 
	    >> *(blank_p) 
	    >> (str_p("IS"))[SVRBIND2(fISFromNameToField,(self,arg1,arg2))] 
	    >> (NameToField_r)[SVRBIND2(fNameToField2,(self,arg1,arg2))];
	  // Fields for group
	  GroupBlock_r = str_p("Group") 
	    >> ch_p('{') 
	    >> *(TransformToField_r) 
	    >> ch_p('}');
      
	  // Fields of Transform block.
	  TCBChildren_r = str_p("children") 
	    >> ch_p('[') 
	    >> *(GroupBlock_r|TransformBlock_r )
	    >> ch_p(']');
      
	  TransformChildrenBlock_r = ch_p('[') 
	    >> *(GroupBlock_r | TCBChildren_r) 
	    >> ch_p(']');
      
	  TransformChildren_r= str_p("children") 
	    >> ((str_p("IS")  >> str_p("children")) |
		TransformChildrenBlock_r);
	  
	  TransformLine_r = (TransformToField_r)[SVRBIND2(fTransformToField,(self,arg1,arg2))]|
	    TransformChildren_r | TCBChildren_r  ;
      
	  TransformBlock_r = str_p("Transform") 
	    >> ch_p('{') 
	    >> *(TransformLine_r) 
	    >> ch_p('}');
      
	  // Fields of Proto []block.
      
	  SFVec3f_r= str_p("SFVec3f") 
	    >> (+alpha_p)[SVRBIND2(fProtoLineName,(self,arg1,arg2))] 
	    >> real_p >> real_p >> real_p; 
      
	  MF_brackets_r= ch_p('[') >> ch_p(']');
	  MFNode_r= str_p("MFNode") 
	    >> (+alpha_p)[SVRBIND2(fProtoLineName,(self,arg1,arg2))] 
	    >> !MF_brackets_r; 

      
	  SFNode_r= str_p("SFNode") 
	    >> (+alpha_p)[SVRBIND2(fProtoLineName,(self,arg1,arg2))] 
	    >> !((ch_p('[') >> ch_p(']')) | str_p("NULL")); 
      
	  MFFloat_r= str_p("MFFloat") >> (+alpha_p)[SVRBIND2(fProtoLineName,(self,arg1,arg2))] 
				      >>  ch_p('[') >> *(real_p) >> ch_p(']'); 
      
	  SFRotation_r= str_p("SFRotation") 
	    >> (+alpha_p)[SVRBIND2(fProtoLineName,(self,arg1,arg2))] 
	    >> real_p >> real_p >> real_p >> real_p; 

	  SFString_r = str_p("SFString") >> (+alpha_p)[SVRBIND2(fProtoLineName,(self,arg1,arg2))] >> ch_p('"') >>
	    *(alnum_p|ch_p('.')) >> ch_p('"');

	  MFString_r= str_p("MFString") >> (+alpha_p)[SVRBIND2(fProtoLineName,(self,arg1,arg2))] >> !MF_brackets_r; 

	  SFFloat_r = str_p("SFFloat") >> (+alpha_p)[SVRBIND2(fProtoLineName,(self,arg1,arg2))] >> *(real_p);
  
	  SFInt32_r = str_p("SFInt32") >> (+alpha_p)[SVRBIND2(fSFInt32,(self,arg1,arg2))] >> *(int_p);; 
	  ProtoLineTitle_r = str_p("exposedField") | str_p("field") | str_p("eventIn") ;

	  ProtoLine_r=  ProtoLineTitle_r >> (SFVec3f_r | SFInt32_r | 
					     SFNode_r | MFNode_r | MFFloat_r | 
					     SFRotation_r | SFString_r | MFString_r | SFFloat_r);

	  ProtoBlock_r = ch_p('[') >> *(ProtoLine_r) >> ch_p(']');

	  ProtoSndBlock_r = ch_p('{') >> *(TransformBlock_r| GroupBlock_r )>> ch_p('}');
 
	  Proto_r= str_p("PROTO") >> (+alpha_p)[SVRBIND2(fProtoName,(self,arg1,arg2))] 
				  >> (ProtoBlock_r)[SVRBIND2(fProtoBlock,(self,arg1, arg2))] 
				  >> (ProtoSndBlock_r)[SVRBIND2(fProtoSndBlock,(self,arg1,arg2))];

	  // Part of the joints.

	  // Read the translation of the joint.
	  JointTranslation_r = str_p("translation") >> 
	    (real_p)[SVRBIND2(fJointTranslationX,(self,arg1))] >> 
	    (real_p)[SVRBIND2(fJointTranslationY,(self,arg1))] >> 
	    (real_p)[SVRBIND2(fJointTranslationZ,(self,arg1))];

	  // Read the rotation of the joint.
	  JointRotation_r = str_p("rotation") >> 
	    (real_p)[SVRBIND2(fJointRotationX,(self,arg1))] >> 
	    (real_p)[SVRBIND2(fJointRotationY,(self,arg1))] >> 
	    (real_p)[SVRBIND2(fJointRotationZ,(self,arg1))] >>
	    (real_p)[SVRBIND2(fJointRotationAngle,(self,arg1))];
    
	  // Type of the joint.
	  JointType_r = str_p("jointType") >> ch_p('"')
					   >> (+alpha_p)[SVRBIND2(fJointType,(self,arg1,arg2))]
					   >> ch_p('"');

	  // Identifient of the joint.
	  JointID_r = str_p("jointId") >> (int_p)[SVRBIND2(fJointID,(self,arg1))];

	  // Specify the axis along which the rotation take place for this joint.
	  JointAxis_r = str_p("jointAxis") >> ch_p('"') 
					   >> ( (ch_p('X'))[SVRBIND2(fJointXAxis,(self,arg1))] | 
						(ch_p('Y'))[SVRBIND2(fJointYAxis,(self,arg1))] | 
						(ch_p('Z'))[SVRBIND2(fJointZAxis,(self,arg1))] )
					   >> ch_p('"');
	  // Not used
	  Jointdh_r = str_p("dh") >> ch_p('[') >> *(real_p)
				  >> ch_p(']'); // not used.

	  // Lower Position Limit for the joint.
	  Jointllimit_r = str_p("llimit") >> ch_p('[') >> (real_p)[SVRBIND2(fJointLLimit,(self,arg1))] 
					  >> ch_p(']'); 

	  // Upper Position Limit for the joint.
	  Jointulimit_r = str_p("ulimit") >> ch_p('[') >> (real_p)[SVRBIND2(fJointULimit,(self,arg1))] 
					  >> ch_p(']'); 
 
	  // Lower Speed Limit for the joint 
	  Jointlvlimit_r = str_p("lvlimit") >> ch_p('[') >> (real_p)[SVRBIND2(fJointLVLimit,(self,arg1))] 
					    >> ch_p(']'); 

	  // Upper Speed Limit for the joint.
	  Jointuvlimit_r = str_p("uvlimit") >> ch_p('[') >> (real_p)[SVRBIND2(fJointUVLimit,(self,arg1))] 
					    >> ch_p(']'); 

	  // Upper Speed Limit for the joint.
	  Jointequivalentinertia_r = str_p("equivalentInertia") >> (real_p)[SVRBIND2(fJointEquivalentInertia,(self,arg1))] ;
					    
 
	  JointField_r = JointType_r | 
	    JointTranslation_r | 
	    JointAxis_r | 
	    JointRotation_r | 
	    JointID_r |
	    Jointdh_r |
	    Jointllimit_r |
	    Jointulimit_r |
	    Jointlvlimit_r |
	    Jointuvlimit_r |
            Jointequivalentinertia_r ;
	
	  // Parts of the force sensor
	  FSTranslation_r = str_p("translation") >> 
	    (real_p)[SVRBIND2(fFSTranslationX,(self,arg1))] >> 
	    (real_p)[SVRBIND2(fFSTranslationY,(self,arg1))] >> 
	    (real_p)[SVRBIND2(fFSTranslationZ,(self,arg1))];

	  // Read the rotation of the force sensor.
	  FSRotation_r = str_p("rotation") >> 
	    (real_p)[SVRBIND2(fFSRotationX,(self,arg1))] >> 
	    (real_p)[SVRBIND2(fFSRotationY,(self,arg1))] >> 
	    (real_p)[SVRBIND2(fFSRotationZ,(self,arg1))] >>
	    (real_p)[SVRBIND2(fFSAngle,(self,arg1))];
	  FSID_r = str_p("sensorId") >> (int_p)[SVRBIND2(fFSID,(self,arg1))];
	  ForceSensorBlock_r = *(FSTranslation_r | FSRotation_r | FSID_r );
	  ForceSensor_r = str_p("ForceSensor") >> ch_p('{') >> ForceSensorBlock_r >> ch_p('}');

	  // Parts of the Gyroscope sensor
	  GyroTranslation_r = str_p("translation") >> 
	    (real_p)[SVRBIND2(fGyroTranslationX,(self,arg1))] >> 
	    (real_p)[SVRBIND2(fGyroTranslationY,(self,arg1))] >> 
	    (real_p)[SVRBIND2(fGyroTranslationZ,(self,arg1))];

	  // Read the rotation of the Gyroscope sensor.
	  GyroRotation_r = str_p("rotation") >> 
	    (real_p)[SVRBIND2(fGyroRotationX,(self,arg1))] >> 
	    (real_p)[SVRBIND2(fGyroRotationY,(self,arg1))] >> 
	    (real_p)[SVRBIND2(fGyroRotationZ,(self,arg1))] >>
	    (real_p)[SVRBIND2(fGyroAngle,(self,arg1))];
	  GyroID_r = str_p("sensorId") >> (int_p)[SVRBIND2(fGyroID,(self,arg1))];
	  GyrometerSensorBlock_r = *(GyroTranslation_r | GyroRotation_r | GyroID_r);
	  GyrometerSensor_r =  str_p("Gyro") >> ch_p('{') >> GyrometerSensorBlock_r >> ch_p('}');
  

	  // Parts of the Acceleration sensor
	  ASTranslation_r = str_p("translation") >> 
	    (real_p)[SVRBIND2(fASTranslationX,(self,arg1))] >> 
	    (real_p)[SVRBIND2(fASTranslationY,(self,arg1))] >> 
	    (real_p)[SVRBIND2(fASTranslationZ,(self,arg1))];

	  // Read the rotation of the Acceleration sensor.
	  ASRotation_r = str_p("rotation") >> 
	    (real_p)[SVRBIND2(fASRotationX,(self,arg1))] >> 
	    (real_p)[SVRBIND2(fASRotationY,(self,arg1))] >> 
	    (real_p)[SVRBIND2(fASRotationZ,(self,arg1))] >>
	    (real_p)[SVRBIND2(fASAngle,(self,arg1))];
	  ASID_r = str_p("sensorId") >> (int_p)[SVRBIND2(fASID,(self,arg1))];
	  AccelerationSensorBlock_r = *(ASTranslation_r | ASRotation_r | ASID_r);
	  AccelerationSensor_r =  str_p("AccelerationSensor") >> ch_p('{') 
							      >> AccelerationSensorBlock_r >> ch_p('}');
  
	  // Read the vision sensor.
  
	  // Translation 
	  VSTranslation_r = str_p("translation") >> 
	    (real_p)[SVRBIND2(fVSTranslationX,(self,arg1))] >> 
	    (real_p)[SVRBIND2(fVSTranslationY,(self,arg1))] >> 
	    (real_p)[SVRBIND2(fVSTranslationZ,(self,arg1))];

	  // Read the rotation of the Acceleration sensor.
	  VSRotation_r = str_p("rotation") >> 
	    (real_p)[SVRBIND2(fVSRotationX,(self,arg1))] >> 
	    (real_p)[SVRBIND2(fVSRotationY,(self,arg1))] >> 
	    (real_p)[SVRBIND2(fVSRotationZ,(self,arg1))] >>
	    (real_p)[SVRBIND2(fVSAngle,(self,arg1))];

	  VSID_r = str_p("sensorId") >> (int_p)[SVRBIND2(fVSID,(self,arg1))];
  
	  VSFrontClipDistance_r = str_p("frontClipDistance") >> real_p;
	  VSBackClipDistance_r = str_p("backClipDistance") >> real_p;
	  VSwidth_r = str_p("width") >> int_p;
	  VSheight_r = str_p("height") >> int_p;
	  VStype_r = str_p("type") >> ch_p('"') >> lexeme_d[+alnum_p] >> ch_p('"');
	  VSFieldOfView_r = str_p("fieldOfView") >> real_p;
	  VSName_r = str_p("name") >> ch_p('"') >> lexeme_d[+alnum_p] >> ch_p('"');

	  VisionSensorBlock_r = *(VSTranslation_r | VSRotation_r | VSID_r |
				  VSFrontClipDistance_r | VSBackClipDistance_r  |
				  VSwidth_r | VSheight_r | VStype_r | VSName_r | 
				  VSFieldOfView_r  );
	  VisionSensor_r = str_p("VisionSensor") >> ch_p('{') >> VisionSensorBlock_r >> ch_p('}');
  
	  ListSensors_r =  VisionSensor_r |
	    AccelerationSensor_r |
	    ForceSensor_r |
	    GyrometerSensor_r ;

	  Sensors_r =  str_p("DEF") >> lexeme_d[+(alpha_p|ch_p('_'))] >> ListSensors_r;

	  // Parts of the body
	  CenterOfMass_r = str_p("centerOfMass") >> 
	    (real_p)[SVRBIND2(fBodyCenterOfMassX,(self,arg1))] >> 
	    (real_p)[SVRBIND2(fBodyCenterOfMassY,(self,arg1))] >> 
	    (real_p)[SVRBIND2(fBodyCenterOfMassZ,(self,arg1))];

	  Mass_r = str_p("mass") >> (real_p)[SVRBIND2(fBodyMass,(self,arg1))];
	  MomentsOfInertia_r = str_p("momentsOfInertia") >> ch_p('[') 
							 >> *((real_p)[SVRBIND2(fFillmomentsOfInertia,(self,arg1))])
							 >> ch_p(']') ;

	  BodySubBlock_r =CenterOfMass_r | Mass_r | MomentsOfInertia_r ;
	  BodyChildrenInlineUrl_r = str_p("url") 
	    >> ch_p('"') 
	    >> (lexeme_d[+(alnum_p|ch_p('_')|ch_p('.')|ch_p('/'))])[SVRBIND2(fBodyInlineUrl,(self,arg1,arg2))] 
	    >> ch_p('"');

	  BodyChildrenInline_r = ch_p('{') >> 
	    *(BodyChildrenInlineUrl_r)>> ch_p('}');
  
	  BodyChildrenField_r= (str_p("Inline") >> BodyChildrenInline_r ) |
	    Sensors_r;
	
	  BodyChildren_r = str_p("children") >> ch_p('[') >> (*BodyChildrenField_r)[SVRBIND2(fProtoName,(self,arg1,arg2))]
					     >> ch_p(']');
	  // Define the entry rules for body and hint
	  BodyBlock_r =  (str_p("Segment"))[SVRBIND2(fBodySubBlockName,(self))]
	    >> ch_p('{') >> *(BodySubBlock_r |
			      (BodyChildren_r)[SVRBIND2(fProtoName,(self,arg1,arg2))])
	    >> (ch_p('}'))[SVRBIND2(fAddBody,(self))];
	  
	  JointChildrenDEFBlocks_r = str_p("DEF") >> (lexeme_d[+(alnum_p|ch_p('_'))])
	    [SVRBIND2(fJCDEFBlocks,(self,arg1,arg2))] |
	    ( BodyBlock_r| 
	      JointSubBlock_r |
	      ListSensors_r);

	  JointChildren_r = str_p("children") >> (ch_p('['))[SVRBIND2(fIncreaseDepth,(self))]
					      >> *( JointChildrenDEFBlocks_r ) 
					      >> ch_p(']')[SVRBIND2(fDecreaseDepth,(self))];

	  JointSubBlock_r = (str_p("Joint"))[SVRBIND2(fJointSubBlockName,(self))] >> ch_p('{') 
					   >> *(JointField_r | JointChildren_r ) >> ch_p('}');

	  JointBlock_r = str_p("DEF") 
	    >> (lexeme_d[+(alnum_p|ch_p('_'))])[SVRBIND2(fJointName,(self,arg1,arg2))] 
	    >> JointSubBlock_r;

	  // Define the entry rules for huanoid.
	  HumanoidBlock_r =  str_p("humanoidBody") 
	    >> ch_p('[') >> *JointBlock_r >> ch_p(']') ;
  
	  HumanoidTrail_r = str_p("Humanoid")
	    >> ch_p('{') >> HumanoidBlock_r >> ch_p('}');

	  Humanoid_r = (str_p("DEF"))[SVRBIND2(fProtoName,(self,arg1,arg2))] 
	    >> (lexeme_d[+alnum_p])[SVRBIND2(fAssignHumanoidName,(self,arg1,arg2))] 
	    >> HumanoidTrail_r;

	  // Navigation Info
	  NIavatarSize_r = str_p("avatarSize") >> (real_p)[SVRBIND2(fNI_AvatarSize,(self,arg1))];
	  NIHeadlight_r = str_p("headlight") >> (str_p("TRUE") | str_p("FALSE"));
	  NIType_r = str_p("type") >> ch_p('[')
				   >> ch_p('"') >> str_p("EXAMINE") >> ch_p('"')
				   >> ch_p(',')
				   >> ch_p('"') >> str_p("ANY")  >> ch_p('"')
				   >> ch_p(']');
	  NavigationInfo_r = str_p("NavigationInfo") >> ch_p('{') >> 
	    *((NIType_r)[SVRBIND2(fNI_Type,(self,arg1,arg2))] | 
	      (NIHeadlight_r)[SVRBIND2(fNI_Headlight,(self,arg1,arg2))] | 
	      (NIavatarSize_r))>> ch_p('}');
  
	  // Background 
	  skyColor_r = str_p("skyColor") >> real_p >> real_p >> real_p;
	  Background_r = str_p("Background") 
	    >> ch_p('{') >> *( (skyColor_r)[SVRBIND2(fskyColor,(self,arg1,arg2))] )
	    >> ch_p('}');

	  // Viewpoint
	  ViewpointOri_r = str_p("orientation")>> real_p >> real_p >> real_p >> real_p;
	  ViewpointPos_r = str_p("position") >> real_p >> real_p >> real_p;
	  Viewpoint_r = str_p("Viewpoint") >> ch_p('{') >> *(
							     (ViewpointPos_r)[SVRBIND2(fViewpointPos,(self,arg1,arg2))]|
							     (ViewpointOri_r)[SVRBIND2(fViewpointOri,(self,arg1,arg2))]
							     ) >> ch_p('}');
	
	  EntryPoint = *(Proto_r| 
			 Humanoid_r| 
			 Background_r | 
			 NavigationInfo_r | 
			 Viewpoint_r );  
	}
    
	rule<ScannerT> scaleMultiple_r, NameToField_r, TransformToField_r,
	  GroupBlock_r, TransformBlock_r, TCBChildren_r, TransformChildrenBlock_r,
	  TransformChildren_r, 
	  TransformLine_r, SFVec3f_r, MF_brackets_r, MFNode_r, SFNode_r, MFFloat_r,
	  SFRotation_r, SFString_r, MFString_r, SFFloat_r, SFInt32_r,
	  ProtoLineTitle_r, ProtoLine_r, ProtoBlock_r, ProtoSndBlock_r, Proto_r;

	rule<ScannerT> JointTranslation_r, JointRotation_r, JointType_r,
	  JointID_r, JointAxis_r, Jointdh_r, 
	  Jointllimit_r, Jointulimit_r,
	  Jointlvlimit_r, Jointuvlimit_r, 
          Jointequivalentinertia_r, 
          JointField_r, JointBlock_r, JointSubBlock_r;

	rule<ScannerT> FSTranslation_r, FSRotation_r, FSID_r, 
	  ForceSensorBlock_r, ForceSensor_r;

	rule<ScannerT> GyroRotation_r,  GyroTranslation_r, GyroID_r, 
	  GyrometerSensorBlock_r, GyrometerSensor_r;

	rule<ScannerT> ASTranslation_r, ASRotation_r, ASID_r,
	  AccelerationSensorBlock_r, AccelerationSensor_r;

	rule<ScannerT> VSTranslation_r, VSRotation_r, VSID_r, 
	  VSFrontClipDistance_r, VSBackClipDistance_r,
	  VSwidth_r,  VSheight_r, VStype_r, VSFieldOfView_r, VSName_r,
	  VisionSensorBlock_r, VisionSensor_r;

	rule<ScannerT> ListSensors_r, Sensors_r, CenterOfMass_r,Mass_r, MomentsOfInertia_r;

	rule<ScannerT> BodySubBlock_r, BodyChildrenInlineUrl_r, BodyChildrenInline_r,
	  BodyChildrenField_r, BodyChildren_r, BodyBlock_r;

	rule<ScannerT> JointChildrenDEFBlocks_r, JointChildren_r, 
	  HumanoidBlock_r, HumanoidTrail_r, Humanoid_r;
      
	rule<ScannerT> NIavatarSize_r, NIHeadlight_r, NIType_r, NavigationInfo_r;

	rule<ScannerT> skyColor_r, Background_r, ViewpointOri_r, 
	  ViewpointPos_r, Viewpoint_r, EntryPoint;

	rule<ScannerT> const& start() const {return EntryPoint;}
	      
      };


      void Init()
      {

	m_DataForParsing->NbOfBodies = 0;

	// Creation du corps de reference
	m_DataForParsing->CurrentBody.resize(DEPTH_MAX);
	m_DataForParsing->CurrentBody[0] = new Body();
	m_DataForParsing->CurrentBody[0]->setLabel(m_DataForParsing->NbOfBodies++);
	m_MultiBody->ajouterCorps(*m_DataForParsing->CurrentBody[0]);
	m_DataForParsing->Depth = 0;
	MAL_S3_VECTOR(,double) dummy;
	m_DataForParsing->CurrentLink.label= 0;
	m_DataForParsing->CurrentLink.aJoint = new JointPrivate(JointPrivate::FIX_JOINT,dummy,0.0);
	m_DataForParsing->JointMemoryAllocationForNewDepth = true;
	m_DataForParsing->CurrentLink.indexCorps1 = 0;
	m_DataForParsing->CurrentLink.indexCorps2 = 0;
	m_DataForParsing->index_mi = 0;
      }

      int getVerbose()
      {return m_Verbose;}
      
      void setVerbose(int lVerbose)
      {m_Verbose = lVerbose;}

    private:

      MultiBody * m_MultiBody;
      struct s_DataForParsing *m_DataForParsing;
      int m_Verbose;
    };

    int ParseVRMLFile(MultiBody *aMB, std::string aFileName)
    {
      ifstream aif;
      struct s_DataForParsing DataForParsing;

      SkipGrammar aSkipGrammar;
      SpiritOpenHRP aSpiritOpenHRP(aMB,
				   &DataForParsing);

      
      aif.open(aFileName.c_str(),ifstream::in|ifstream::binary);
      if (!aif.is_open())
	{
	  cout << "Unable to open file "<< aFileName<< endl;
	  return 0;
	}
      else {
	if (aSpiritOpenHRP.getVerbose()>10)
	  {
	    cout << "Succeeded in opening " << aFileName <<endl;
	  }
      }
      unsigned int length;
  
      // get length of file:
      aif.seekg (0, ios::end);
      length = aif.tellg();

      aif.seekg (0, ios::beg);
  
      // allocate memory:
      char * buffer = new char [length+1];
      // read data as a block:
      aif.read(buffer,length);
      aif.close();
  
      //  BOOST_SPIRIT_DEBUG_RULE(BodyBlock_r);
      //  BOOST_SPIRIT_DEBUG_RULE(Humanoid_r);
      aSpiritOpenHRP.Init();
      aSpiritOpenHRP.setVerbose(10);
      
      parse(buffer,aSpiritOpenHRP,aSkipGrammar).full;
      delete [] buffer;
      return 1;

    };
  };
};
