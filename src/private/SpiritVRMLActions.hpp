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

#include <string.h>

/*!  Framework includes */
#include "JointFreeFlyerPrivate.h"
#include "JointTranslationPrivate.h"
#include "JointAnchorPrivate.h"
#include "JointRotationPrivate.h"
#include "MultiBody.h"
#include "SpiritVRMLReader.h"

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
    }

    struct s_DataForParsing
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
      s_DataForParsing()
      {
	CurrentLink.aJoint = 0;
      }
      
      // String for the url.
      BodyGeometricalData m_BodyGeometry;
      
      // Generic vector3d.
      vector3d m_Genericvec3d;

      // vector of vector3d.
      std::vector<vector3d> m_vectorgvec3d;

      // Map of Proto.
      std::map<std::string, Proto_t> Protos;

      // Current Proto being parser.
      Proto_t cProto;
    };


    struct Actions
    {
      Actions():
	fSFVec3fX(*this,0),
	fSFVec3fY(*this,1),
	fSFVec3fZ(*this,2),
	fPushGenericvec3d(*this),
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
	m_Verbose(2)
      {}
      
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
	    std::cout<< "SFVec3f_0" << x<<"|" << endl;
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
	  m_actions.m_DataForParsing.m_vectorgvec3d.push_back(m_actions.m_DataForParsing.m_Genericvec3d);
	}
      private:
	Actions & m_actions;
      } fPushGenericvec3d;
      
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
		cout << "JointPrivate" 
		     << m_actions.m_DataForParsing.CurrentLink.aJoint->getName()
		     << ":" 
		     << m_actions.m_DataForParsing.JointTranslation << endl;
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
	      m_actions.m_DataForParsing.m_BodyGeometry.setRotationForDisplay(displayR);
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
	  m_actions.cProto.Name = x;
	  if (m_actions.m_Verbose>1)
	    std::cout<< "fProtoName:" << m_actions.cProto.Name <<"|" << endl;
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
	  m_actions.cProto.Name = x;
	  if (m_actions.m_Verbose>1)
	    std::cout<< "fProtoName:" << m_actions.cProto.Name <<"|" << endl;
	}
      private:
	Actions & m_actions;
      } fProtoName;
      


      // Fields of Actions:
      struct s_DataForParsing m_DataForParsing;
      unsigned int m_Verbose;
    };
    
  };
};
