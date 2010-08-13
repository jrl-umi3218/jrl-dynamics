/* @doc Action to take when tokens are detected by the VRML97 grammar.

   Copyright (c) 2005-2009, 

   @author : 
   Olivier Stasse
   
   JRL-Japan, CNRS/AIST

   All rights reserved.
   
   Please refers to file License.txt for details on the license.

*/

/*! System includes */
#include <fstream>
#include <sstream>
#include <map>
#include <queue>

#include <string.h>

/*!  Framework includes */
#include "JointFreeFlyerPrivate.h"
#include "JointTranslationPrivate.h"
#include "JointAnchorPrivate.h"
#include "JointRotationPrivate.h"

#include "MultiBody.h"
#include "SpiritVRMLReader.h"

#include "Debug.h"

using namespace std;

namespace dynamicsJRLJapan 
{
  namespace VRMLReader
  {
    struct Proto_t
    {
      std::map<string,string> exposedFields;
      std::map<string,string> field;
      std::map<string,string> eventIn;
      std::map<string,string> eventOut;

      string Name;
    };

    struct DataForParsing_t
    {

      // Variable to build the VRML tree.
      int NbOfBodies;

      // Depth
      int Depth;

      // Joint memory allocation done for new depth.
      //bool JointMemoryAllocationForNewDepth;
      
      // Current Link.
      internalLink CurrentLink;

      // Vector of current branch
      vector<Body *> CurrentBody;

      // Stack of Rotation Matrix for display.
      vector<matrix3d> StackOfRotationMatrixDisplay;

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
      vector3d cm;

      // Constructor
      DataForParsing_t()
      {
	CurrentLink.aJoint = 0;
	mass = 0.0;
	index_mi = -1;
	Depth = 0.0;
	m_BodyGeometry =0;
	m_LOUIndex = new int;
      }
      
      // Destructor.
      ~DataForParsing_t()
      {
	for (unsigned int i=0;
	     i<m_ListOfURLs.size();
	     i++)
	  delete m_ListOfURLs[i];

	delete m_LOUIndex;
      }
      
      
      // String for the url.
      BodyGeometricalData * m_BodyGeometry;
      
      // Generic vector3d.
      vector3d m_Genericvec3d;

      // Generic vector of vector3d.
      std::vector<vector3d> m_vectorgvec3d;

      // Generic vector of int32d.
      std::vector<int> m_vectorgint32;

      // Map of Proto.
      std::map<std::string, struct Proto_t> Protos;

      // Current Proto being parser.
      struct Proto_t cProto;

      MultiBody m_MultiBody;
      
      vector<BodyGeometricalData * > m_ListOfURLs;

      int * m_LOUIndex;

      Geometry::Shape m_Shape;

    };

    struct Actions
    {
      Actions():
	fSFVec3fX(*this,0),
	fSFVec3fY(*this,1),
	fSFVec3fZ(*this,2),
	fPushGenericvec3d(*this),
	fPushGenericvecint32(*this),
	fJointTranslationX(*this,0),
	fJointTranslationY(*this,1),
	fJointTranslationZ(*this,2),
	fJointRotationX(*this,0),
	fJointRotationY(*this,1),
	fJointRotationZ(*this,2),
	fJointRotationAngle(*this,3),
	fBodyMass(*this),
	fBodyCenterOfMassX(*this,0),
	fBodyCenterOfMassY(*this,1),
	fBodyCenterOfMassZ(*this,2),
	fProtoName(*this),
	fProtoExposedField(*this),
	fDEFName(*this),
	fJointType(*this),
	fJointID(*this),
	fJointXAxis(*this,0),
	fJointYAxis(*this,1),
	fJointZAxis(*this,2),
	fJointLLimit(*this,0,0),
	fJointULimit(*this,0,1),
	fJointLVLimit(*this,1,0),
	fJointUVLimit(*this,1,1),
	fEquivalentInertia(*this),
	fUpdateAndAddBody(*this),
	fFillMomentsOfInertia(*this),
	fIncreaseDepth(*this),
	fDecreaseDepth(*this),
	fAddURL(*this),
	fMaterialAmbientIntensity(*this,0),
	fMaterialDiffuseColorR(*this,1),
	fMaterialDiffuseColorG(*this,2),
	fMaterialDiffuseColorB(*this,3),
	fMaterialEmissiveColorR(*this,4),
	fMaterialEmissiveColorG(*this,5),
	fMaterialEmissiveColorB(*this,6),
	fMaterialShininess(*this,7),
	fMaterialSpecularColorR(*this,8),
	fMaterialSpecularColorG(*this,9),
	fMaterialSpecularColorB(*this,10),	
	fMaterialTransparency(*this,11),
	fIndexedFaceSetccw(*this,0),
	fIndexedFaceSetcolorPerVertex(*this,1),
	fIndexedFaceSetconvex(*this,2),
	fIndexedFaceSetnormalPerVertex(*this,3),
	fIndexedFaceSetsolid(*this,4),
	fCoordinates(*this),
	fCoordIndex(*this),
	fStoreShape(*this),
	m_Verbose(0)
      { }
      
      // Generic action for display 
      struct fDisplay_t {

	void operator()(const char &c) const
	{
	  ODEBUG("Display " << c  );
	}

	template <typename IteratorT>
	void operator()(const IteratorT &begin, 
			const IteratorT &end) const 
	{
	  std::string x(begin,end);
	  file_position fp_cur;
	  
	  // Store the current file position
	  fp_cur = begin.get_position();
	  ODEBUG("Display - Current file: " << fp_cur.file );
	  ODEBUG( "Line   : " << fp_cur.line  
		  << " Column : " << fp_cur.column 
		  << endl
		  << "Parsed : " << x );
	} 
      } fDisplay;

      // Action for a vector of 3 floats.
      struct fSFVec3f_i_t {

	explicit fSFVec3f_i_t(Actions &actions, 
			      unsigned int anIndex): 
	  m_actions(actions),
	  m_index(anIndex){};
	  
	void operator()(const double x) const 
	{
	  m_actions.m_DataForParsing.m_Genericvec3d(m_index) = x;
	  if (m_actions.m_Verbose>1)
	    std::cout<< "SFVec3f_" << m_index << " : " <<x<<"|" << endl;
	}

      private:
	Actions & m_actions;
	unsigned int m_index;
      } fSFVec3fX,fSFVec3fY,fSFVec3fZ;

      // Build an array of vectors
      struct fPushGenericvec3d_t {

	explicit fPushGenericvec3d_t(Actions &actions): 
	  m_actions(actions) {};
	
	template <typename IteratorT>
	void operator()(IteratorT, IteratorT) const 
	{
	  ODEBUG("Pushing :" 
		 << m_actions.m_DataForParsing.m_Genericvec3d(0) << " " 
		 << m_actions.m_DataForParsing.m_Genericvec3d(1) << " " 
		 << m_actions.m_DataForParsing.m_Genericvec3d(2) );
	  m_actions.m_DataForParsing.
	    m_vectorgvec3d.push_back(m_actions.m_DataForParsing.
				     m_Genericvec3d);
	}
      private:
	Actions & m_actions;
      } fPushGenericvec3d;

      // Build an array of int
      struct fPushGenericvecint32_t {

	explicit fPushGenericvecint32_t(Actions &actions): 
	  m_actions(actions) {};
	
	void operator()(int x) const 
	{

	  ODEBUG("Pushing :" << x );
	  m_actions.m_DataForParsing.
	    m_vectorgint32.push_back(x);	  
	}

      private:
	Actions & m_actions;
      } fPushGenericvecint32;
      
      // Action for a translation.
      struct fJointTranslation_i_t {

	explicit fJointTranslation_i_t(Actions &actions, 
				       unsigned int anIndex): 
	  m_actions(actions),
	  m_index(anIndex){};
	  
	void operator()(const double x) const 
	{
	  m_actions.m_DataForParsing.JointTranslation(m_index) = x;
	  if (m_actions.m_Verbose>1)
	    std::cout<< "fJointTranslation_i_t(" << m_index
		     <<")= " << x<<"|" << endl;

	  if (m_index==2)
	    {
	      // IMPORTANT POLICY: all the free joint have their static 
	      // translation set to zero.
	      if (m_actions.m_DataForParsing.CurrentLink.aJoint->type()!=JointPrivate::FREE_JOINT)
		m_actions.m_DataForParsing.CurrentLink.aJoint->
		  setStaticTranslation(m_actions.m_DataForParsing.JointTranslation);
	      else
		{
		  MAL_S3_VECTOR(,double) lnull;
		  lnull(0) = lnull(1) = lnull(2) = 0.0;
		  m_actions.m_DataForParsing.CurrentLink.aJoint->setStaticTranslation(lnull);
		}
	      
	      if (m_actions.m_Verbose>1)
		{
		  cout << "JointPrivate" 
		       << m_actions.m_DataForParsing.CurrentLink.aJoint->getName()
		       << ":" 
		       << m_actions.m_DataForParsing.JointTranslation << endl;
		}
	    }
	}
      private:
	Actions & m_actions;
	unsigned int m_index;
      } fJointTranslationX,fJointTranslationY,fJointTranslationZ;

      // Action for a Joint Rotation
      struct fJointRotation_i_t {

	explicit fJointRotation_i_t(Actions &actions, 
				    unsigned int anIndex): 
	  m_actions(actions),
	  m_index(anIndex){};
	  
	void operator()(const double x) const 
	{
	  if (m_index<3)
	    m_actions.m_DataForParsing.RotationAxis(m_index) = x;
	  else
	    {
	      double lQuantity = x;
	      matrix3d R;
	      matrix3d displayR;
	      // Conversion from rotation axis to rotation matrix.
	      AxisAngle2Matrix(m_actions.m_DataForParsing.RotationAxis, 
			       lQuantity, R);
	      
	      
	      // From the current branch of rotation computes
	      // the rotation at the global level for display.
	      if (m_actions.m_DataForParsing.Depth!=0)
		{
		  matrix3d pR = m_actions.m_DataForParsing.
		    StackOfRotationMatrixDisplay[m_actions.m_DataForParsing.Depth-1];
		  MAL_S3x3_C_eq_A_by_B(displayR,pR,R);
		}
	      else 
		{
		  MAL_S3x3_MATRIX_SET_IDENTITY(displayR); 
		}
	      
	      m_actions.m_DataForParsing.StackOfRotationMatrixDisplay[m_actions.m_DataForParsing.Depth] = displayR;
	      if (m_actions.m_Verbose>1)
		cout << "StackOfRotationMatrixDisplay[" 
		     << m_actions.m_DataForParsing.Depth << " ] = " 
		     << displayR  << endl;
	      
	      // Then set the static rotation at the level joint 
	      // and 
	      m_actions.m_DataForParsing.CurrentLink.aJoint->setStaticRotation(R);
	      if (m_actions.m_DataForParsing.m_BodyGeometry!=0)
		{
		  m_actions.m_DataForParsing.m_BodyGeometry->setRotationForDisplay(displayR);
		  m_actions.m_DataForParsing.m_BodyGeometry->resetURL();
		}
	      else
		{
		  throw("Problem regarding m_BodyGeometry allocation");
		}
	      if (lQuantity!=0.0)
		{
		  if (m_actions.m_Verbose>1){
		    std::cerr << m_actions.m_DataForParsing.aName 
			      << " JointRotation:" << R << endl;
		    std::cerr << "Axis: "<< m_actions.m_DataForParsing.RotationAxis 
			      << " angle: " << lQuantity<<endl;
		    std::cerr << "displayR: " << displayR << endl;
		  }
		}
	    }	    
	}

      private:
	Actions & m_actions;
	unsigned int m_index;
      } fJointRotationX,fJointRotationY,fJointRotationZ,fJointRotationAngle;

      // Store the mass of a body
      struct fBodyMass_t {

	explicit fBodyMass_t(Actions &actions): 
	  m_actions(actions) {};
	
	void operator()(const double x) const 
	{
	  m_actions.m_DataForParsing.mass = x ;
	}
      private:
	Actions & m_actions;
      } fBodyMass;

      // Action to store the center of mass of the robot.
      struct fBodyCenterOfMass_t {

	explicit fBodyCenterOfMass_t(Actions &actions, 
				     unsigned int anIndex): 
	  m_actions(actions),
	  m_index(anIndex){};
	  
	void operator()(const double x) const 
	{
	  m_actions.m_DataForParsing.cm[m_index] = x;
	  if (m_actions.m_Verbose>1)
	    std::cout<< "fBodyCenterOfMass" << x<<"|" << endl;
	}
      private:
	Actions & m_actions;
	unsigned int m_index;
      } fBodyCenterOfMassX,fBodyCenterOfMassY,fBodyCenterOfMassZ;

      // Action to store the name of the proto.
      struct fProtoName_t {

	explicit fProtoName_t(Actions &actions): 
	  m_actions(actions)
	{};
	  
	template <typename IteratorT>
	void operator()(const IteratorT & begin, 
			const IteratorT & end) const 
	{

	  string x(begin, end);
	  m_actions.m_DataForParsing.cProto.Name = x;
	  if (m_actions.m_Verbose>1)
	    std::cout<< "fProtoName:" << m_actions.m_DataForParsing.cProto.Name <<"|" << endl;
	}
      private:
	Actions & m_actions;
      } fProtoName;
      
      struct fProtoExposedField_t {

	explicit fProtoExposedField_t(Actions &actions): 
	  m_actions(actions)
	{};
	  
	template <typename IteratorT>
	void operator()(const IteratorT & begin, 
			const IteratorT & end) const 
	{

	  string x(begin, end);
	  if (m_actions.m_Verbose>1)
	    std::cout<< "fProtoExposedField:" << x<<"|" << endl;
	}
      private:
	Actions & m_actions;
      } fProtoExposedField;

      // Extract the name of a definition.
      struct fDEFName_t {

	explicit fDEFName_t(Actions &actions): 
	  m_actions(actions)
	{};
	  
	template <typename IteratorT>
	void operator()(const IteratorT & begin, 
			const IteratorT & end) const 
	{

	  string x(begin, end);
	  m_actions.m_DataForParsing.aName = x;
	  if (m_actions.m_Verbose>1)
	    std::cout<< "fDEFName:" << x<<"|" << endl;
	}
      private:
	Actions & m_actions;
      } fDEFName;
	    
      // In case of Joint type.      
      struct fJointType_t {

	explicit fJointType_t(Actions &actions): 
	  m_actions(actions) {};
	
	template <typename IteratorT>
	void operator()(const IteratorT &begin, 
			const IteratorT &end) const 
	{
	  MAL_S3_VECTOR(lnull,double);
	  lnull[0]=0.0;lnull[1]=0.0;lnull[2]=0.0;
	  
	  string s(begin,end);
	  if (m_actions.m_Verbose>1)
	    cout << "jointType: " << s << endl;
	  
	  
	  if (s=="free")
	    {
	      
	      m_actions.m_DataForParsing.CurrentLink.aJoint = new JointFreeflyerPrivate();
	      m_actions.m_DataForParsing.CurrentLink.aJoint->setIDinActuated(-1);
	      m_actions.m_DataForParsing.CurrentLink.aJoint->setName(m_actions.m_DataForParsing.aName);
	      
	      m_actions.m_DataForParsing.CurrentLink.aJoint->type(JointPrivate::FREE_JOINT);
	    }
	  else if (s=="rotate")
	    {
	      ODEBUG("Joint Rotation Private");
	      m_actions.m_DataForParsing.CurrentLink.aJoint = new JointRotationPrivate();
	      m_actions.m_DataForParsing.CurrentLink.aJoint->setIDinActuated(-1);
	      m_actions.m_DataForParsing.CurrentLink.aJoint->setName(m_actions.m_DataForParsing.aName);
	      
	      m_actions.m_DataForParsing.CurrentLink.aJoint->type(JointPrivate::REVOLUTE_JOINT);
	    }
	  else if (s=="slide")
	    {
	      m_actions.m_DataForParsing.CurrentLink.aJoint = new JointTranslationPrivate();
	      m_actions.m_DataForParsing.CurrentLink.aJoint->setIDinActuated(-1);
	      m_actions.m_DataForParsing.CurrentLink.aJoint->setName(m_actions.m_DataForParsing.aName);
	      
	      m_actions.m_DataForParsing.CurrentLink.aJoint->type(JointPrivate::PRISMATIC_JOINT);
	      
	    }
	}
      private:
	Actions & m_actions;
      } fJointType;

      // Action for a Joint ID.
      struct fJointID_t {

	explicit fJointID_t(Actions &actions): 
	  m_actions(actions){};
	  
	void operator()(const int aJointID) const 
	{
	  m_actions.m_DataForParsing.CurrentLink.aJoint->setIDinActuated(aJointID);
	  if (m_actions.m_Verbose>1)
	    std::cout<< "aJointID" << aJointID<<"|" << endl;
	}
      private:
	Actions & m_actions;
      } fJointID;

      // Axis for a Joint.
      struct fJointAxis_t {

	explicit fJointAxis_t(Actions &actions,
			      unsigned int Index): 
	  m_actions(actions),
	  m_Index(Index){};
	  
	void operator()(const int aJointID) const 
	{
	  MAL_S3_VECTOR(laxis,double);
	  laxis[0] = laxis[1] = laxis[2] = 0.0;
	  laxis[m_Index] = 1.0;
	  
	  m_actions.m_DataForParsing.CurrentLink.aJoint->type(JointPrivate::REVOLUTE_JOINT);
	  m_actions.m_DataForParsing.CurrentLink.aJoint->axis(laxis);

	  if (m_actions.m_Verbose>1)
	    std::cout<< "aJointAxis" << m_Index<<"|" << endl;
	}
      private:
	Actions & m_actions;
	unsigned int m_Index;
      } fJointXAxis,fJointYAxis,fJointZAxis;

      // Action for a limit
      struct fJointLimits_t {

	explicit fJointLimits_t(Actions &actions,
				unsigned int PV,
				unsigned int LU): 
	  m_actions(actions),
	  m_PV(PV),
	  m_LU(LU){};
	  
	void operator()(const double aLimit) const 
	{
	  if (m_PV==0)
	    {
	      if (m_LU==0)
		m_actions.m_DataForParsing.CurrentLink.aJoint->lowerBound(0,aLimit);
	      else if (m_LU==1)
		m_actions.m_DataForParsing.CurrentLink.aJoint->upperBound(0,aLimit);
	    }
	  else if (m_PV==1)
	    {
	      if (m_LU==0)
		m_actions.m_DataForParsing.CurrentLink.aJoint->lowerVelocityBound(0,aLimit);
	      else if (m_LU==1)
		m_actions.m_DataForParsing.CurrentLink.aJoint->upperVelocityBound(0,aLimit);
	    }

	  if (m_actions.m_Verbose>1)
	    std::cout<< "Limit ( " << m_PV <<" , " <<m_LU << " ):" <<  aLimit<< "|" << endl;
	}
      private:
	Actions & m_actions;
	unsigned int m_PV, m_LU;
      } fJointLLimit, fJointULimit, fJointLVLimit, fJointUVLimit;

      // Action for equivalent inertia.
      struct fEquivalentInertia_t {

	explicit fEquivalentInertia_t(Actions &actions): 
	  m_actions(actions){};
	  
	void operator()(const double x) const 
	{
	  double lx = x;
	  m_actions.m_DataForParsing.CurrentLink.aJoint->equivalentInertia(lx);
	  if (m_actions.m_Verbose>1)
	    std::cout<< "equivalentInertia" << x<<"|" << endl;
	}
      private:
	Actions & m_actions;
      } fEquivalentInertia;

      // Update the data in a body and a new body.
      struct fUpdateAndAddBody_t {

	explicit fUpdateAndAddBody_t(Actions &actions): 
	  m_actions(actions){};

	template <typename IteratorT>
	void operator()(IteratorT &, 
			IteratorT &) const 
	{
	  if (m_actions.m_Verbose>1)
	    {
	      std::cout << "Starting Body." << std::endl;
	    }
	  DataForParsing_t &aDataForParsing = m_actions.m_DataForParsing;
	  
	  int lDepth = aDataForParsing.Depth;
	  ODEBUG("depth: "<< lDepth);
	  Body * lCurrentBody = aDataForParsing.CurrentBody[lDepth];
	  lCurrentBody->setLabel(aDataForParsing.NbOfBodies++);
	  lCurrentBody->setName((char *)(aDataForParsing.aName).c_str());
	  lCurrentBody->setInertie(aDataForParsing.mi);
	  lCurrentBody->setMass(aDataForParsing.mass);
	  lCurrentBody->localCenterOfMass(aDataForParsing.cm);
	  if (aDataForParsing.Depth!=0)
	    {
	      lCurrentBody->setLabelMother(aDataForParsing.CurrentBody[lDepth-1]->getLabel());
	      
	    }
	  aDataForParsing.m_MultiBody.addBody(*lCurrentBody);
	  aDataForParsing.m_MultiBody.addLink(*aDataForParsing.CurrentBody[lDepth-1],
					      *lCurrentBody,
					      aDataForParsing.CurrentLink);
	  //	aDataForParsing.JointMemoryAllocationForNewDepth=false;
	  aDataForParsing.m_BodyGeometry->setBodyName(lCurrentBody->getName());
	  aDataForParsing.m_BodyGeometry->
	    setRelatedJointName(m_actions.
				m_DataForParsing.
				CurrentLink.
				aJoint->getName());

	  aDataForParsing.m_ListOfURLs.push_back(aDataForParsing.m_BodyGeometry);
	  aDataForParsing.m_BodyGeometry = new BodyGeometricalData ();
	  lCurrentBody->setInitialized(true);

	  if (m_actions.m_Verbose>1)
	    {
	      std::cout << "Adding Body." << std::endl;
	    }
	}
	
      private:
	Actions & m_actions;

      } fUpdateAndAddBody;

      struct fFillMomentsOfInertia_t {

	explicit fFillMomentsOfInertia_t(Actions &actions): 
	  m_actions(actions){};
	
	void operator()(const double z) const 
	{
	  DataForParsing_t &aDataForParsing = m_actions.m_DataForParsing;
	  aDataForParsing.mi[aDataForParsing.index_mi++]=z;
	  if (aDataForParsing.index_mi==9)
	    {
	      if (m_actions.m_Verbose>1)
		{
		  for(int i=0;i<9;i++)
		    {
		      ODEBUG(aDataForParsing.mi[i]);
		      if (i%3==2)
			cout<< endl;
		    }
		}
	      aDataForParsing.index_mi=0;
	    }
	}
	
      private:
	Actions & m_actions;
	
      } fFillMomentsOfInertia;

      // Increase depth when parsing the robot tree.
      struct fIncreaseDepth_t {

	explicit fIncreaseDepth_t(Actions &actions): 
	  m_actions(actions){};

	void LocalAction() const
	{
	  DataForParsing_t &aDataForParsing = m_actions.m_DataForParsing;

	  int lDepth = aDataForParsing.Depth;
	  // After the initial body.
	  if (lDepth>0)
	    {
	      
	      // Check wether or not the current body is virtual or not.
	      if (!aDataForParsing.CurrentBody[lDepth]->getInitialized())
		{
		  // If it is virtual then do a basic initialization.
		  Body * lCurrentBody = aDataForParsing.CurrentBody[lDepth];
		  lCurrentBody->setLabelMother(aDataForParsing.CurrentBody[lDepth-1]->getLabel());
		  
		  char Buffer[1024];
		  memset(Buffer,0,1024);
		  sprintf(Buffer,"VIRTUAL_%d", aDataForParsing.NbOfBodies);
		  lCurrentBody->setLabel(aDataForParsing.NbOfBodies++);
		  lCurrentBody->setName(Buffer);
		  aDataForParsing.m_MultiBody.addBody(*lCurrentBody);
		  aDataForParsing.m_MultiBody.addLink(*aDataForParsing.CurrentBody[lDepth-1],
						      *lCurrentBody,
						      aDataForParsing.CurrentLink);
		  
		  lCurrentBody->setLabelMother(aDataForParsing.CurrentBody[lDepth-1]->getLabel());
		  lCurrentBody->setInitialized(true);
		}
	    }
	  else
	    {
	      m_actions.Init();
	    }
	  
	  // Increase Depth.
	  aDataForParsing.Depth++;
	  
	  // Creates a default body.
	  aDataForParsing.CurrentBody[aDataForParsing.Depth] = new Body() ;
	  
	  MAL_S3x3_MATRIX_SET_IDENTITY(aDataForParsing.StackOfRotationMatrixDisplay[aDataForParsing.Depth]);
	  
	  if (m_actions.m_Verbose>1)
	    std::cout << "Increased depth "<< aDataForParsing.Depth << endl;

	} 

	template <typename IteratorT>
	void operator()(IteratorT , 
			IteratorT ) const 
	{
	  LocalAction();
	}
	void operator()(const char & ) const
	{
	  LocalAction();
	}

      private:
	Actions & m_actions;

      } fIncreaseDepth;


      // Increase depth when parsing the robot tree.
      struct fDecreaseDepth_t {

	explicit fDecreaseDepth_t(Actions &actions): 
	  m_actions(actions){};

	void LocalAction() const
	{
	  m_actions.m_DataForParsing.Depth--;
	  
	  if (m_actions.m_Verbose>1)
	    std::cout << "Decreased depth "<< m_actions.m_DataForParsing.Depth << endl;
	}

	template <typename IteratorT>
	void operator()(IteratorT,
			IteratorT) const
	{
	  LocalAction();
	}
	
	void operator()(const char &) const
	{
	  LocalAction();
	}

      private:
	Actions & m_actions;
	
      } fDecreaseDepth;


      // Add URL information
      struct fAddURL_t {

	explicit fAddURL_t(Actions &actions): 
	  m_actions(actions){};


	template <typename IteratorT>
	void operator()(const IteratorT &begin,
			const IteratorT &end) const
	{
	  string s(begin,end);
	  file_position fp_cur;
	  
	  // Store the current file position
	  fp_cur = begin.get_position();
	  ODEBUG( "Current file: " << fp_cur.file );
	  ODEBUG("Line   : " << fp_cur.line  
		 << " Column : " << fp_cur.column 
		 << endl
		 << "File to be included : " << s);
	  
	  string sp2 = s;
	  ODEBUG( " " << sp2);
	  m_actions.m_DataForParsing.m_BodyGeometry->addURL(sp2);
	}
	
      private:
	Actions & m_actions;
	
      } fAddURL;



      struct fMaterial_t {
	explicit fMaterial_t(Actions &actions,
			     unsigned int anIndex):
	  m_actions(actions),
	  m_index(anIndex){};
	
	void operator()(double x) const
	{
	  ODEBUG("fMaterial ("<<m_index << ")=" << x);
 
	  Geometry::Material & aMaterial = 
	    m_actions.m_DataForParsing.m_Shape.getAppearance().getMaterial();

	  if (m_index==0)
	    aMaterial.ambientIntensity = x;
	  else if (m_index==1)
	    aMaterial.diffuseColor[0] = x;
	  else if (m_index==2)
	    aMaterial.diffuseColor[1] = x;
	  else if (m_index==3)
	    aMaterial.diffuseColor[2] = x;
 	  else if (m_index==4)
	    aMaterial.emissiveColor[0] = x;
 	  else if (m_index==5)
	    aMaterial.emissiveColor[1] = x;
 	  else if (m_index==6)
	    aMaterial.emissiveColor[2] = x;
 	  else if (m_index==7)
	    aMaterial.shininess = x;
 	  else if (m_index==8)
	    aMaterial.specularColor[0] = x;
 	  else if (m_index==9)
	    aMaterial.specularColor[1] = x;
 	  else if (m_index==10)
	    aMaterial.specularColor[2] = x;
	  else if (m_index==10)
	    aMaterial.transparency = x;

	}
	
      private:
	Actions &m_actions;
	unsigned int m_index;
      } fMaterialAmbientIntensity, 
	fMaterialDiffuseColorR,
	fMaterialDiffuseColorG, 
	fMaterialDiffuseColorB,       
	fMaterialEmissiveColorR,
	fMaterialEmissiveColorG, 
	fMaterialEmissiveColorB,
        fMaterialShininess,
	fMaterialSpecularColorR,
	fMaterialSpecularColorG, 
	fMaterialSpecularColorB,
	fMaterialTransparency;
      

      struct fIndexedFaceSet_t {
	explicit fIndexedFaceSet_t(Actions &actions,
				   unsigned int anIndex):
	  m_actions(actions),
	  m_index(anIndex){};
	
	void operator()(bool x) const
	{
	  ODEBUG( "fIndexedFaceSet ("<<m_index << ")=" << x);
 
	  Geometry::IndexedFaceSet &aIndexedFaceSet = m_actions.
	    m_DataForParsing.m_Shape.getIndexedFaceSet();
	  
	  if (m_index==0)
	    { aIndexedFaceSet.ccw = x; }
	  else if (m_index==1)
	    { aIndexedFaceSet.colorPerVertex = x; }
	  else if (m_index==2)
            { aIndexedFaceSet.convex=x;}
          else if (m_index==3)
            { aIndexedFaceSet.normalPerVertex=x;}
          else if (m_index==4)
	    { aIndexedFaceSet.solid=x;}
	}
	
      private:
	Actions &m_actions;
	unsigned int m_index;
	
      } fIndexedFaceSetccw,
        fIndexedFaceSetcolorPerVertex,
        fIndexedFaceSetconvex,
        fIndexedFaceSetnormalPerVertex,
        fIndexedFaceSetsolid;

      struct fCoordinates_t {

	explicit fCoordinates_t(Actions &actions):
	  m_actions(actions) {};
	  
	template <typename IteratorT>
	void operator()(const IteratorT &begin, 
			const IteratorT &end) const 
	{
	  m_actions.m_DataForParsing.m_Shape.getIndexedFaceSet().coord =
	    m_actions.m_DataForParsing.m_vectorgvec3d;
	  m_actions.m_DataForParsing.m_vectorgvec3d.clear();
	}
      private:
	Actions & m_actions;
      } fCoordinates;

      struct fCoordIndex_t {

	explicit fCoordIndex_t(Actions &actions):
	  m_actions(actions) {};
	  
	template <typename IteratorT>
	void operator()(const IteratorT &begin, 
			const IteratorT &end) const 
	{
	  m_actions.m_DataForParsing.m_Shape.getIndexedFaceSet().coordIndex.
	    push_back(m_actions.m_DataForParsing.m_vectorgint32);
	  m_actions.m_DataForParsing.m_vectorgint32.clear();
	}
      private:
	Actions & m_actions;
      } fCoordIndex;

      struct fStoreShape_t {

	explicit fStoreShape_t(Actions &actions):
	  m_actions(actions) {};
	  
	template <typename IteratorT>
	void operator()(const IteratorT &begin, 
			const IteratorT &end) const 
	{
	  file_position fp_cur;
	  fp_cur = begin.get_position();
	  ODEBUG("Display - Current file: " << fp_cur.file );
	  ODEBUG( "Line   : " << fp_cur.line  
		  << " Column : " << fp_cur.column );

	  int lindex = *m_actions.m_DataForParsing.m_LOUIndex;
	  if (lindex!=-1)
	    {
	      ODEBUG(" Store inside :"<< lindex );
	      m_actions.m_DataForParsing.m_ListOfURLs[lindex]->addShape(m_actions.m_DataForParsing.m_Shape);
	      ODEBUG(m_actions.m_DataForParsing.m_Shape.getAppearance().getMaterial());
	      m_actions.m_DataForParsing.m_Shape.reset();
	    }
	}
      private:
	Actions & m_actions;
      } fStoreShape;

      
      // Fields of Actions:
      struct DataForParsing_t m_DataForParsing;

      unsigned int m_Verbose;

      void Init()
      {

	m_DataForParsing.NbOfBodies = 0;

	// Resize stacks
	m_DataForParsing.CurrentBody.resize(DEPTH_MAX);
	m_DataForParsing.StackOfRotationMatrixDisplay.resize(DEPTH_MAX);

	// Create reference body.
	m_DataForParsing.CurrentBody[0] = new Body();
	m_DataForParsing.CurrentBody[0]->setLabel(m_DataForParsing.NbOfBodies++);
	m_DataForParsing.m_MultiBody.addBody(*m_DataForParsing.CurrentBody[0]);
	m_DataForParsing.Depth = 0;

	// Initialization of the initial matrix.
	MAL_S3x3_MATRIX_SET_IDENTITY(m_DataForParsing.StackOfRotationMatrixDisplay[0]);

	matrix4d eye;
	MAL_S4x4_MATRIX_SET_IDENTITY(eye);
	m_DataForParsing.CurrentLink.label= 0;
	m_DataForParsing.CurrentLink.aJoint = new JointFreeflyerPrivate(eye);
	ODEBUG(" m_DataForParsing.CurrentLink.aJoint->m_globalConfiguration"<<
	       m_DataForParsing.CurrentLink.aJoint->initialPosition());

	m_DataForParsing.CurrentLink.indexCorps1 = 0;
	m_DataForParsing.CurrentLink.indexCorps2 = 0;
	m_DataForParsing.index_mi = 0;

	m_DataForParsing.m_BodyGeometry = new BodyGeometricalData ();
      }

    };
    
  };
};
