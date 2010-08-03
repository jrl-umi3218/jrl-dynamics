/* @doc Object used to parse a VRML file describing a robot.

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
#include <string.h>

/*! Parsing related macros */
//#define BOOST_SPIRIT_DEBUG
//#define BOOST_SPIRIT_RULE_SCANNERTYPE_LIMIT 2
#define DEPTH_MAX 40


/*!  Framework includes */
//#define DEBUG_MODE
#include "Debug.h"

/*! Boost includes */
#include "boost/version.hpp"

#if BOOST_VERSION < 104000
#include <boost/spirit.hpp>
#include <boost/spirit/phoenix/binders.hpp>
#include <boost/spirit/utility/chset.hpp>
#else
#include <boost/spirit/include/classic.hpp>
#include <boost/spirit/include/phoenix1_binders.hpp>
#include <boost/spirit/include/classic_chset.hpp>
#endif

#include <boost/lambda/lambda.hpp>
#include <boost/lambda/bind.hpp>

using namespace std;
using namespace boost::spirit;

#if BOOST_VERSION < 104000
using namespace boost::spirit::utility;
#else
using namespace boost::spirit::classic;
using namespace boost::spirit::classic::utility;
#endif

#include "MultiBody.h"
//#include "SpiritVRMLError.hpp"

using namespace phoenix;

//#define SVRBIND(x) bind(&SpiritVRMLReader::x,this,_1)
#define SVRBIND(x) &x
#define SVRBIND2(x,y) cout << y <<endl;

namespace dynamicsJRLJapan 
{
  namespace VRMLReader
  {

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


    template <typename tActions>
    struct SpiritOpenHRP: 
      boost::spirit::classic::grammar<SpiritOpenHRP<tActions> >
    {
      template <typename ScannerT>
      struct definition
      {

	// bool ParseVRMLFile(char const *str)
	
	definition(SpiritOpenHRP const &self)
	{
      
	  // Basic types
	  SFBool_r = str_p("TRUE") | str_p("FALSE");
	  
	  SFVec3f_r = 
	    real_p[self.actions.fSFVec3fX] >> 
	    real_p[self.actions.fSFVec3fY] >> 
	    real_p[self.actions.fSFVec3fZ];

	  MFVec3f_r = *((SFVec3f_r)[self.actions.fPushGenericvec3d]
			| ch_p(','));

	  MFInt32_r = *(int_p | ch_p(','));

	  MFString_r = ch_p('[') 
	    >> *( ch_p('"') >> (*alpha_p) >> ch_p('"') )
	    >> ch_p(']');

	  // Coordinate rules
	  Coordinate_r = str_p("Coordinate")
	    >> ch_p('{') 
	    >> str_p("point")
	    >> str_p('[')
	    >> MFVec3f_r 
	    >> str_p(']')
	    >> ch_p('}');
	  
	  // Fields to link a name and field 
	  scaleMultiple_r = str_p("scale")>> !str_p("Orientation");
	  NameToField_r = str_p("center") | 
	    str_p("children")             |
	    str_p("rotation")             | 
	    scaleMultiple_r               |
	    str_p("translation")          | 
	    str_p("bboxCenter")           | 
	    str_p("bboxSize")             |
	    str_p("addChildren")          | 
	    str_p("removeChildren")       | 
	    str_p("viewpoints")           | 
	    str_p("humanoidBody")         |
	    str_p("transmitter")          | 
	    str_p("receiver");
	  
	  TransformToField_r = (NameToField_r) 
	    >> *(blank_p) 
	    >> (str_p("IS")) 
	    >> (NameToField_r);
	  
	  TransformInstanceRotation_r = str_p("rotation") >>
	    (real_p) >> 
	    (real_p) >> 
	    (real_p) >>
	    (real_p);
	  
	  TransformInstanceTranslation_r = (str_p("translation")) >>
	    real_p>> real_p>> real_p;
	  
	  // Fields for group
	  GroupBlock_r = str_p("Group") 
	    >> ch_p('{') 
	    >> *(TransformToField_r) 
	    >> ch_p('}');
	  
	  Route_r = str_p("ROUTE")
	    >> lexeme_d[+(alnum_p|'.'|'_')]
	    >> str_p("TO")
	    >> lexeme_d[+(alnum_p|'.'|'_')];
	  
	  // Fields of Transform block.
	  TCBChildrenBlock_r = ch_p('[') 
	    >> *(GroupBlock_r     |
		 TransformBlock_r | 
		 ShapeInline_r    | 
		 Sensors_r        | 
		 Shape_r )
	    >> ch_p(']');

	  TCBChildren_r = (str_p("children"))
	    >> Shape_r          | 
	    GroupBlock_r        | 
	    ShapeInline_r       | 
	    TransformBlock_r    |
	    Sensors_r           | 
	    TCBChildrenBlock_r ;
	  
	  TransformChildrenBlock_r = ch_p('[') 
	    >> *(GroupBlock_r | TCBChildren_r) 
	    >> ch_p(']');
	  
	  TransformChildren_r= str_p("children")
	    >> ((str_p("IS")  >> str_p("children")) |
		TransformChildrenBlock_r);
	  
	  TransformLine_r = (TransformToField_r) |
	    TransformChildren_r | TCBChildren_r  ;
	  
	  TransformBlock_r = ((str_p("DEF")>> lexeme_d[+alnum_p] >> 
			       (str_p("Transform"))
			       |(str_p("Transform"))))
	    >> ch_p('{') 
	    >> *(TransformLine_r | TransformInstanceRotation_r | TransformInstanceTranslation_r) 
	    >> ch_p('}');
	  
	  // Fields of Proto []block.
	  
	  SFVec3fD_r= str_p("SFVec3f") 
	    >> (+alpha_p)
	    >> SFVec3f_r; 
	  
	  MF_brackets_r= ch_p('[') >> ch_p(']');
	  MFNodeD_r= str_p("MFNode") 
	    >> (+alpha_p)
	    >> !MF_brackets_r; 
	  
	  
	  SFNodeD_r= str_p("SFNode") 
	    >> (+alpha_p)
	    >> !((ch_p('[') >> ch_p(']')) | str_p("NULL")); 
	  
	  MFFloatD_r= str_p("MFFloat") 
	    >> (+alpha_p)
	    >>  ch_p('[') >> *(real_p) >> ch_p(']'); 
      
	  SFRotationD_r= str_p("SFRotation") 
	    >> (+alpha_p)
	    >> real_p >> real_p >> real_p >> real_p; 

	  SFStringD_r = str_p("SFString") 
	    >> (+alpha_p)
	    >>  ch_p('"') 
	    >> *(alnum_p|ch_p('.')) 
	    >> ch_p('"');

	  MFStringD_r= str_p("MFString") 
	    >> (+alpha_p) 
	    >> MFString_r ;
	  
	  SFFloatD_r = str_p("SFFloat") >> (+alpha_p) >> real_p;
	  
	  SFInt32D_r = str_p("SFInt32") >> (+alpha_p) >> *(int_p); 
	  
	  ProtoLineTitle_r = 
	    str_p("exposedField") | 
	    str_p("field") | 
	    str_p("eventIn") |
	    str_p("eventOut");
	  
	  ProtoLine_r=  ProtoLineTitle_r >> 
	    (SFVec3fD_r    | 
	     SFInt32D_r    | 
	     SFNodeD_r     | 
	     MFNodeD_r     | 
	     MFFloatD_r    | 
	     SFRotationD_r | 
	     SFStringD_r   | 
	     MFStringD_r   | 
	     SFFloatD_r);

	  ProtoBlock_r = ch_p('[') >> *((ProtoLine_r)[self.actions.fDisplay]) >> ch_p(']');
	  
	  ProtoSndBlock_r = ch_p('{') >> *(TransformBlock_r| GroupBlock_r|Route_r )>> ch_p('}');
	  
	  Proto_r= str_p("PROTO") >> (+alpha_p)[self.actions.fProtoName] 
				  >> (ProtoBlock_r)
				  >> (ProtoSndBlock_r);
	
	  // Part of the joints.

	  // Read the translation of the joint.
	  JointTranslation_r = str_p("translation") >> 
	    (real_p)[self.actions.fJointTranslationX] >> 
	    (real_p)[self.actions.fJointTranslationY] >> 
	    (real_p)[self.actions.fJointTranslationZ];

	  // Read the rotation of the joint.
	  JointRotation_r = str_p("rotation") >> 
	    (real_p)[self.actions.fJointRotationX] >> 
	    (real_p)[self.actions.fJointRotationY] >> 
	    (real_p)[self.actions.fJointRotationZ] >>
	    (real_p)[self.actions.fJointRotationAngle];
    
	  // Type of the joint.
	  JointType_r = str_p("jointType") >> ch_p('"')
					   >> (+alpha_p)[self.actions.fJointType]
					   >> ch_p('"');

	  // Identifient of the joint.
	  JointID_r = str_p("jointId") >> (int_p)[self.actions.fJointID];

	  // Specify the axis along which the rotation take place for this joint.
	  JointAxis_r = str_p("jointAxis") >> ch_p('"') 
					   >> ( (ch_p('X'))[self.actions.fJointXAxis] | 
						(ch_p('Y'))[self.actions.fJointYAxis] | 
						(ch_p('Z'))[self.actions.fJointZAxis] )
					   >> ch_p('"');
	  // Not used
	  Jointdh_r = str_p("dh") >> ch_p('[') >> *(real_p)
				  >> ch_p(']'); // not used.

	  // Lower Position Limit for the joint.
	  Jointllimit_r = str_p("llimit") >> ch_p('[') >> (real_p)[self.actions.fJointLLimit]
					  >> ch_p(']'); 

	  // Upper Position Limit for the joint.
	  Jointulimit_r = str_p("ulimit") >> ch_p('[') >> (real_p)[self.actions.fJointULimit]
					  >> ch_p(']'); 
 
	  // Lower Speed Limit for the joint 
	  Jointlvlimit_r = str_p("lvlimit") >> ch_p('[') >> (real_p)[self.actions.fJointLVLimit]
					    >> ch_p(']'); 

	  // Upper Speed Limit for the joint.
	  Jointuvlimit_r = str_p("uvlimit") >> ch_p('[') >> (real_p)[self.actions.fJointUVLimit]
					    >> ch_p(']'); 

	  // Upper Speed Limit for the joint.
	  Jointequivalentinertia_r = str_p("equivalentInertia") >> 
	    (real_p)[self.actions.fEquivalentInertia];
					    
 
	  JointField_r = JointType_r | 
	    JointTranslation_r       | 
	    JointAxis_r              |  
	    JointRotation_r          | 
	    JointID_r                |
	    Jointdh_r                |
	    Jointllimit_r            |
	    Jointulimit_r            |
	    Jointlvlimit_r           |
	    Jointuvlimit_r           |
            Jointequivalentinertia_r ;
	
	  // Parts of the force sensor
	  FSTranslation_r = str_p("translation") >> 
	    (real_p) >> 
	    (real_p) >> 
	    (real_p);

	  // Read the rotation of the force sensor.
	  FSRotation_r = str_p("rotation") >> 
	    (real_p) >> 
	    (real_p) >> 
	    (real_p) >>
	    (real_p);
	  FSID_r = str_p("sensorId") >> (int_p);
	  ForceSensorBlock_r = *(FSTranslation_r | FSRotation_r | FSID_r );
	  ForceSensor_r = str_p("ForceSensor") >> ch_p('{') >> ForceSensorBlock_r >> ch_p('}');

	  // Parts of the Gyroscope sensor
	  GyroTranslation_r = str_p("translation") >> 
	    (real_p) >> 
	    (real_p) >> 
	    (real_p);

	  // Read the rotation of the Gyroscope sensor.
	  GyroRotation_r = str_p("rotation") >> 
	    (real_p) >> 
	    (real_p) >> 
	    (real_p) >>
	    (real_p);
	  GyroID_r = str_p("sensorId") >> (int_p);
	  GyrometerSensorBlock_r = *(GyroTranslation_r | GyroRotation_r | GyroID_r);
	  GyrometerSensor_r =  str_p("Gyro") >> ch_p('{') >> GyrometerSensorBlock_r >> ch_p('}');
  

	  // Parts of the Acceleration sensor
	  ASTranslation_r = str_p("translation") >> 
	    (real_p) >> 
	    (real_p) >> 
	    (real_p);

	  // Read the rotation of the Acceleration sensor.
	  ASRotation_r = str_p("rotation") >> 
	    (real_p) >> 
	    (real_p) >> 
	    (real_p) >>
	    (real_p);
	  ASID_r = str_p("sensorId") >> (int_p);
	  AccelerationSensorBlock_r = *(ASTranslation_r | ASRotation_r | ASID_r);
	  AccelerationSensor_r =  str_p("AccelerationSensor") >> ch_p('{') 
							      >> AccelerationSensorBlock_r >> ch_p('}');
  
	  // Read the vision sensor.
  
	  // Translation 
	  VSTranslation_r = str_p("translation") >> 
	    (real_p) >> 
	    (real_p) >> 
	    (real_p);

	  // Read the rotation of the Acceleration sensor.
	  VSRotation_r = str_p("rotation") >> 
	    (real_p) >> 
	    (real_p) >> 
	    (real_p) >>
	    (real_p);

	  VSID_r = str_p("sensorId") >> (int_p);
  
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
  
	  CSAngle_r = (lexeme_d[+alnum_p])
	    >> str_p("IS") >> (lexeme_d[+alnum_p]);
	  CylinderSensorBlock_r = *(CSAngle_r);

	  CylinderSensor_r = (str_p("CylinderSensor"))
	    >> ch_p('{') >> CylinderSensorBlock_r >> ch_p('}');

	  ListSensors_r =  VisionSensor_r |
	    AccelerationSensor_r |
	    ForceSensor_r |
	    GyrometerSensor_r |
	    CylinderSensor_r;

	  Sensors_r =  str_p("DEF") >> (lexeme_d[+(alpha_p|ch_p('_'))] )
				    >> ListSensors_r ;

	  // Parts of the body
	  CenterOfMass_r = str_p("centerOfMass") >> 
	    (real_p)[self.actions.fBodyCenterOfMassX] >> 
	    (real_p)[self.actions.fBodyCenterOfMassY] >> 
	    (real_p)[self.actions.fBodyCenterOfMassZ];

	  Mass_r = str_p("mass") >> (real_p)[self.actions.fBodyMass];
	  MomentsOfInertia_r = str_p("momentsOfInertia") >> ch_p('[') 
							 >> *((real_p)[self.actions.fFillMomentsOfInertia])
							 >> ch_p(']') ;
	
	  // Material block
	  DiffuseColor_r = str_p("diffuseColor") >> 
	    real_p >> real_p >> real_p ;
	  SpecularColor_r = str_p("specularColor") >>
	    real_p >> real_p >> real_p;
	  EmissiveColor_r = str_p("emissiveColor") >>
	    real_p >> real_p >> real_p;
	  Shininess_r = str_p("shininess") >> real_p;
	  Transparency_r = str_p("transparency") >> real_p;
	  AmbientIntensity_r = str_p("ambientIntensity") >> real_p;
	
	  MaterialBlock_r = *( DiffuseColor_r  | 
			       SpecularColor_r |
			       EmissiveColor_r |
			       Shininess_r     |
			       Transparency_r  |
			       AmbientIntensity_r );

	  // Appearance block
	  AppearanceBlock_r = (str_p("material"))
	    >> str_p("Material")
	    >> ch_p('{') >> *(MaterialBlock_r) >> ch_p('}');
	  
	  AppearanceUse_r = str_p("USE")  >>  lexeme_d[+(alnum_p|'_')];
	  AppearanceDef_r = str_p("DEF")  
	    >> ( (lexeme_d[+(alnum_p|'_')] 
		  >> str_p("Appearance")) | str_p("Appearance") )
	    >>  ch_p('{') 
	    >> AppearanceBlock_r
	    >> ch_p('}') ;

	  AppearanceHeader_r = str_p("appearance") >>
	    (AppearanceDef_r | 
	     AppearanceUse_r);
	  
	  // Geometry block

	  // Box
	  GeometryBox_r = str_p("Box")
	    >> ch_p('{') 
	    >> str_p("size") >> real_p >> real_p >>real_p 
	    >> ch_p('}');
	  
	  // Cylinder
	  GeometryCylinder_r = str_p("Cylinder")
	    >> ch_p('{') 
	    >> str_p("radius") >> real_p 
	    >> str_p("height") >> real_p 
	    >> ch_p('}');
	  
	  // IndexedFaceSet

	  IFSccwfield_r = str_p("ccw") >> SFBool_r;
	  IFSconvexfield_r = str_p("convex") >> SFBool_r;
	  IFSsolidfield_r = str_p("solid") >> SFBool_r;
	  IFScreaseAngle_r = str_p("creaseAngle") >> real_p;
	  IFScoord_r = str_p("coord") >> Coordinate_r;

	  IFScoordIndex_r = str_p("coordIndex")
	    >> ch_p('[')
	    >> *( MFInt32_r 
		  >> ((str_p("-1") >> ch_p(','))
		      | str_p("-1")
		      )
		  ) 
	    >> ch_p(']');
	
	  IndexedFaceSet_r = str_p("IndexedFaceSet")
	    >> ch_p ('{')
	    >> IFSccwfield_r |
	    IFSconvexfield_r |
	    IFSsolidfield_r  |
	    IFScreaseAngle_r |
	    IFScoord_r 
	    >> ch_p ('{');
	  // Header
	  GeometryHeader_r = str_p("geometry")
	    >>  GeometryBox_r |  
	    GeometryCylinder_r;
	
	  // Shape block
	  ShapeBlock_r = AppearanceHeader_r | 
	    GeometryHeader_r;
	  
	  Shape_r = (str_p("Shape"))
	    >> ch_p('{') 
	    >> *ShapeBlock_r 
	    >> ch_p('}');

	  BodySubBlock_r =CenterOfMass_r | Mass_r | MomentsOfInertia_r ;
	  ShapeInlineUrl_r = str_p("url") 
	    >> ch_p('"') 
	    >> (lexeme_d[+(alnum_p|ch_p('_')|ch_p('.')|ch_p('/'))])
	    >> ch_p('"');

	  ShapeBlockInline_r = ch_p('{') >> 
	    *(ShapeInlineUrl_r)>> ch_p('}');
  
	  ShapeInline_r = (str_p("Inline") >> ShapeBlockInline_r );

	  BodyChildrenField_r=  ShapeInline_r | Sensors_r | Shape_r | TransformBlock_r;
	
	  BodyChildren_r = str_p("children") >> (ch_p('[') 
						 >> *BodyChildrenField_r
						 >> ch_p(']')) |
	        Shape_r;
	  
	  // Define the entry rules for body and hint
	  BodyBlock_r =  (str_p("Segment"))
	    >> ch_p('{')
	    >> *((BodySubBlock_r) |
		 (BodyChildren_r) )
	    >> ch_p('}');
	  
	  JointChildrenDEFBlocks_r = str_p("DEF") >> (lexeme_d[+(alnum_p|ch_p('_'))])
	    [self.actions.fDEFName]
	     |
	    ( (BodyBlock_r)[self.actions.fUpdateAndAddBody] | 
	      JointBlock_r |
	      ListSensors_r);

	  JointChildren_r = str_p("children") >> (ch_p('['))
					      >> *( JointChildrenDEFBlocks_r ) 
					      >> ch_p(']');

	  JointBlock_r = (str_p("Joint"))
	    >> ch_p('{')[self.actions.fIncreaseDepth]
	    >> *(JointField_r | JointChildren_r ) 
	    >> ch_p('}')[self.actions.fDecreaseDepth];

	  DEFBlock_r = str_p("DEF") 
	    >> (lexeme_d[+(alnum_p|ch_p('_'))])[self.actions.fDEFName]
	    >> JointBlock_r;

	  HumanoidVersion_r = str_p("version") >> ch_p('"') 
					       >> (lexeme_d[+(alnum_p|'.')])
					       >> ch_p('"');
          HumanoidName_r = str_p("name") >> ch_p('"') >> (lexeme_d[+(alnum_p)])
					 >> ch_p('"');
	  HumanoidInfoLine_r = ch_p('"')
	    >> *((lexeme_d[+(alnum_p|':'|'.'|',')]))
	    >> ch_p('"');
	    
				

	  HumanoidInfo_r = (str_p("info"))
	    >> ch_p('[') >> *(HumanoidInfoLine_r) >> ch_p(']');
	  
	  // Define the entry rules for huanoid.
	  HumanoidBlock_r =  *(str_p("humanoidBody") 
			       >> ch_p('[') >> *DEFBlock_r >> ch_p(']') |
			       HumanoidName_r | HumanoidVersion_r |
			       HumanoidInfo_r );
  
	  HumanoidTrail_r = str_p("Humanoid")
	    >> ch_p('{') >> HumanoidBlock_r >> ch_p('}');

	  Humanoid_r = (str_p("DEF"))
	    >> (lexeme_d[+alnum_p]) 
	    >> HumanoidTrail_r;

	  // Navigation Info
	  NIavatarSize_r = str_p("avatarSize") >> (real_p);
	  NIHeadlight_r = str_p("headlight") >> SFBool_r;
	  NIType_r = str_p("type") >> ch_p('[')
				   >> ch_p('"') >> str_p("EXAMINE") >> ch_p('"')
				   >> ch_p(',')
				   >> ch_p('"') >> str_p("ANY")  >> ch_p('"')
				   >> ch_p(']');
	  NavigationInfo_r = str_p("NavigationInfo") >> ch_p('{') >> 
	    *((NIType_r) | 
	      (NIHeadlight_r) | 
	      (NIavatarSize_r))>> ch_p('}');
  
	  // Background 
	  skyColor_r = str_p("skyColor") >> real_p >> real_p >> real_p;
	  Background_r = str_p("Background") 
	    >> ch_p('{') >> *( (skyColor_r) )
	    >> ch_p('}');

	  // Viewpoint
	  ViewpointOri_r = str_p("orientation")>> real_p >> real_p >> real_p >> real_p;
	  ViewpointPos_r = str_p("position") >> real_p >> real_p >> real_p;
	  Viewpoint_r = str_p("Viewpoint") >> ch_p('{') >> *(
							     (ViewpointPos_r)|
							     (ViewpointOri_r)
							     ) >> ch_p('}');
	
	  EntryPoint = *(Proto_r| 
			 Humanoid_r| 
			 Background_r | 
			 NavigationInfo_r | 
			 Viewpoint_r );  
	

	  BOOST_SPIRIT_DEBUG_RULE(scaleMultiple_r);
	  BOOST_SPIRIT_DEBUG_RULE(NameToField_r);
	  BOOST_SPIRIT_DEBUG_RULE(TransformToField_r);
	  BOOST_SPIRIT_DEBUG_RULE(TransformInstanceRotation_r);
	  BOOST_SPIRIT_DEBUG_RULE(TransformInstanceTranslation_r);
	  BOOST_SPIRIT_DEBUG_RULE(GroupBlock_r);
	  BOOST_SPIRIT_DEBUG_RULE(Route_r);
	  BOOST_SPIRIT_DEBUG_RULE(TransformBlock_r);
	  BOOST_SPIRIT_DEBUG_RULE(TCBChildren_r);
	  BOOST_SPIRIT_DEBUG_RULE(TCBChildrenBlock_r);
	  BOOST_SPIRIT_DEBUG_RULE(TransformChildrenBlock_r);
	  BOOST_SPIRIT_DEBUG_RULE(TransformChildren_r);
	  BOOST_SPIRIT_DEBUG_RULE(TransformLine_r);
	  BOOST_SPIRIT_DEBUG_RULE(SFVec3f_r);
	  BOOST_SPIRIT_DEBUG_RULE(MF_brackets_r);
	  BOOST_SPIRIT_DEBUG_RULE(MFNodeD_r);
	  BOOST_SPIRIT_DEBUG_RULE(SFNodeD_r);
	  BOOST_SPIRIT_DEBUG_RULE(MFFloatD_r);
	  BOOST_SPIRIT_DEBUG_RULE(SFRotationD_r);
	  BOOST_SPIRIT_DEBUG_RULE(SFStringD_r);
	  BOOST_SPIRIT_DEBUG_RULE(MFStringD_r);
	  BOOST_SPIRIT_DEBUG_RULE(SFFloatD_r);
	  BOOST_SPIRIT_DEBUG_RULE(SFInt32D_r);
	  
	  BOOST_SPIRIT_DEBUG_RULE(ProtoLineTitle_r);
	  BOOST_SPIRIT_DEBUG_RULE(ProtoLine_r);
	  BOOST_SPIRIT_DEBUG_RULE(ProtoBlock_r);
	  BOOST_SPIRIT_DEBUG_RULE(ProtoSndBlock_r);
	  BOOST_SPIRIT_DEBUG_RULE(Proto_r);

	  BOOST_SPIRIT_DEBUG_RULE(JointTranslation_r); 
	  BOOST_SPIRIT_DEBUG_RULE(JointRotation_r); 
	  BOOST_SPIRIT_DEBUG_RULE(JointType_r);
	  BOOST_SPIRIT_DEBUG_RULE(JointID_r); 
	  BOOST_SPIRIT_DEBUG_RULE(JointAxis_r); 
	  BOOST_SPIRIT_DEBUG_RULE(Jointdh_r); 
	  BOOST_SPIRIT_DEBUG_RULE(Jointllimit_r);
	  BOOST_SPIRIT_DEBUG_RULE(Jointulimit_r);
	  BOOST_SPIRIT_DEBUG_RULE(Jointlvlimit_r); 
	  BOOST_SPIRIT_DEBUG_RULE(Jointuvlimit_r); 
          BOOST_SPIRIT_DEBUG_RULE(Jointequivalentinertia_r); 
          BOOST_SPIRIT_DEBUG_RULE(JointField_r); 
	  BOOST_SPIRIT_DEBUG_RULE(DEFBlock_r); 
	  BOOST_SPIRIT_DEBUG_RULE(JointBlock_r);
	  
	  BOOST_SPIRIT_DEBUG_RULE(FSTranslation_r); 
	  BOOST_SPIRIT_DEBUG_RULE(FSRotation_r); 
	  BOOST_SPIRIT_DEBUG_RULE(FSID_r); 
	  BOOST_SPIRIT_DEBUG_RULE(ForceSensorBlock_r); 
	  BOOST_SPIRIT_DEBUG_RULE(ForceSensor_r);
	  
	  BOOST_SPIRIT_DEBUG_RULE(BodyChildren_r);
	  BOOST_SPIRIT_DEBUG_RULE(BodyBlock_r);

	  BOOST_SPIRIT_DEBUG_RULE(GyroRotation_r);  
	  BOOST_SPIRIT_DEBUG_RULE(GyroTranslation_r); 
	  BOOST_SPIRIT_DEBUG_RULE(GyroID_r); 
	  BOOST_SPIRIT_DEBUG_RULE(GyrometerSensorBlock_r); 
	  BOOST_SPIRIT_DEBUG_RULE(GyrometerSensor_r);

	  BOOST_SPIRIT_DEBUG_RULE(ASTranslation_r); 
	  BOOST_SPIRIT_DEBUG_RULE(ASRotation_r); 
	  BOOST_SPIRIT_DEBUG_RULE(ASID_r);
	  BOOST_SPIRIT_DEBUG_RULE(AccelerationSensorBlock_r); 
	  BOOST_SPIRIT_DEBUG_RULE(AccelerationSensor_r);

	  BOOST_SPIRIT_DEBUG_RULE(VSTranslation_r); 
	  BOOST_SPIRIT_DEBUG_RULE(VSRotation_r); 
	  BOOST_SPIRIT_DEBUG_RULE(VSID_r); 
	  BOOST_SPIRIT_DEBUG_RULE(VSFrontClipDistance_r); 
	  BOOST_SPIRIT_DEBUG_RULE(VSBackClipDistance_r);
	  BOOST_SPIRIT_DEBUG_RULE(VSwidth_r);  
	  BOOST_SPIRIT_DEBUG_RULE(VSheight_r); 
	  BOOST_SPIRIT_DEBUG_RULE(VStype_r); 
	  BOOST_SPIRIT_DEBUG_RULE(VSFieldOfView_r); 
	  BOOST_SPIRIT_DEBUG_RULE(VSName_r);
	  BOOST_SPIRIT_DEBUG_RULE(VisionSensorBlock_r); 
	  BOOST_SPIRIT_DEBUG_RULE(VisionSensor_r);
	  BOOST_SPIRIT_DEBUG_RULE(CylinderSensorBlock_r); 
	  BOOST_SPIRIT_DEBUG_RULE(CylinderSensor_r); 
	  BOOST_SPIRIT_DEBUG_RULE(CSAngle_r);

	  BOOST_SPIRIT_DEBUG_RULE(ListSensors_r); 
	  BOOST_SPIRIT_DEBUG_RULE( Sensors_r); 
	  BOOST_SPIRIT_DEBUG_RULE( CenterOfMass_r); 
	  BOOST_SPIRIT_DEBUG_RULE(Mass_r); 
	  BOOST_SPIRIT_DEBUG_RULE( MomentsOfInertia_r);
	  
	  BOOST_SPIRIT_DEBUG_RULE(BodySubBlock_r);
	  BOOST_SPIRIT_DEBUG_RULE( ShapeInlineUrl_r);
	  BOOST_SPIRIT_DEBUG_RULE( ShapeBlockInline_r);
	  BOOST_SPIRIT_DEBUG_RULE( ShapeInline_r);
	  BOOST_SPIRIT_DEBUG_RULE( BodyChildrenField_r);
	  BOOST_SPIRIT_DEBUG_RULE( BodyChildren_r);
	  BOOST_SPIRIT_DEBUG_RULE( BodyBlock_r);
	  BOOST_SPIRIT_DEBUG_RULE( GeometryHeader_r);
	  BOOST_SPIRIT_DEBUG_RULE( GeometryBox_r);
	  BOOST_SPIRIT_DEBUG_RULE( GeometryCylinder_r);
	  BOOST_SPIRIT_DEBUG_RULE( Shape_r);
	  BOOST_SPIRIT_DEBUG_RULE( ShapeBlock_r);
	  BOOST_SPIRIT_DEBUG_RULE(AppearanceBlock_r);
	  BOOST_SPIRIT_DEBUG_RULE( AppearanceHeader_r);
	  BOOST_SPIRIT_DEBUG_RULE(  AppearanceUse_r);
	  BOOST_SPIRIT_DEBUG_RULE( AppearanceDef_r);
	  BOOST_SPIRIT_DEBUG_RULE( MaterialBlock_r);
	  BOOST_SPIRIT_DEBUG_RULE( JointChildrenDEFBlocks_r);
	  BOOST_SPIRIT_DEBUG_RULE( JointChildren_r);
	  BOOST_SPIRIT_DEBUG_RULE(  HumanoidBlock_r);
	  BOOST_SPIRIT_DEBUG_RULE( HumanoidTrail_r);
	  BOOST_SPIRIT_DEBUG_RULE( Humanoid_r);
	  BOOST_SPIRIT_DEBUG_RULE(HumanoidVersion_r);
	  BOOST_SPIRIT_DEBUG_RULE( HumanoidName_r);
	  BOOST_SPIRIT_DEBUG_RULE( HumanoidInfo_r);
	  BOOST_SPIRIT_DEBUG_RULE(HumanoidInfoLine_r);
	  BOOST_SPIRIT_DEBUG_RULE( NIavatarSize_r);
	  BOOST_SPIRIT_DEBUG_RULE( NIHeadlight_r);
	  BOOST_SPIRIT_DEBUG_RULE( NIType_r);
	  BOOST_SPIRIT_DEBUG_RULE( NavigationInfo_r);
	  BOOST_SPIRIT_DEBUG_RULE(skyColor_r);
	  BOOST_SPIRIT_DEBUG_RULE( Background_r);
	  BOOST_SPIRIT_DEBUG_RULE( ViewpointOri_r);
	  BOOST_SPIRIT_DEBUG_RULE( ViewpointPos_r);
	  BOOST_SPIRIT_DEBUG_RULE( Viewpoint_r);
	  BOOST_SPIRIT_DEBUG_RULE( EntryPoint);
	  
	}
	
	// Tree of the robot.
	rule<ScannerT> scaleMultiple_r, NameToField_r, TransformToField_r,
	  TransformInstanceRotation_r, TransformInstanceTranslation_r,
	  GroupBlock_r, Route_r,
	  TransformBlock_r, TCBChildren_r, TCBChildrenBlock_r, TransformChildrenBlock_r,
	  TransformChildren_r, 
	  TransformLine_r, SFVec3f_r, MF_brackets_r, MFNodeD_r, SFNodeD_r, MFFloatD_r,
	  SFRotationD_r, SFStringD_r, MFStringD_r, SFFloatD_r, SFInt32D_r,SFVec3fD_r,
	  ProtoLineTitle_r, ProtoLine_r, ProtoBlock_r, ProtoSndBlock_r, Proto_r;

	// Definitions of variables.
	rule<ScannerT>  MFString_r;
	 
	rule<ScannerT> JointTranslation_r, JointRotation_r, JointType_r,
	  JointID_r, JointAxis_r, Jointdh_r, 
	  Jointllimit_r, Jointulimit_r,
	  Jointlvlimit_r, Jointuvlimit_r, 
          Jointequivalentinertia_r, 
          JointField_r, DEFBlock_r, JointBlock_r;

	rule<ScannerT> SFBool_r,MFVec3f_r,MFInt32_r;

	rule<ScannerT> Coordinate_r;

	rule<ScannerT> FSTranslation_r, FSRotation_r, FSID_r, 
	  ForceSensorBlock_r, ForceSensor_r;

	rule<ScannerT> GyroRotation_r,  GyroTranslation_r, GyroID_r, 
	  GyrometerSensorBlock_r, GyrometerSensor_r;

	rule<ScannerT> ASTranslation_r, ASRotation_r, ASID_r,
	  AccelerationSensorBlock_r, AccelerationSensor_r;

	rule<ScannerT> VSTranslation_r, VSRotation_r, VSID_r, 
	  VSFrontClipDistance_r, VSBackClipDistance_r,
	  VSwidth_r,  VSheight_r, VStype_r, VSFieldOfView_r, VSName_r,
	  VisionSensorBlock_r, VisionSensor_r,
	  CylinderSensorBlock_r, CylinderSensor_r, CSAngle_r;

	rule<ScannerT> ListSensors_r, Sensors_r, CenterOfMass_r,Mass_r, MomentsOfInertia_r;

	rule<ScannerT> BodySubBlock_r, ShapeInlineUrl_r, ShapeBlockInline_r, ShapeInline_r,
	  BodyChildrenField_r, BodyChildren_r, BodyBlock_r;

	// IndexedFaceSet
	rule<ScannerT> IndexedFaceSet_r, IFSccwfield_r,
	  IFSconvexfield_r, IFSsolidfield_r, IFScreaseAngle_r,
	  IFScoord_r, IFScoordIndex_r;

	rule<ScannerT> GeometryHeader_r, GeometryBox_r, GeometryCylinder_r;

	rule<ScannerT> Transparency_r,AmbientIntensity_r;

	rule<ScannerT> Shape_r, ShapeBlock_r,AppearanceBlock_r, AppearanceHeader_r, 
	  AppearanceUse_r, AppearanceDef_r, MaterialBlock_r,
	  DiffuseColor_r, SpecularColor_r, EmissiveColor_r, Shininess_r;

	rule<ScannerT> JointChildrenDEFBlocks_r, JointChildren_r, 
	  HumanoidBlock_r, HumanoidTrail_r, Humanoid_r,
	  HumanoidVersion_r, HumanoidName_r, HumanoidInfo_r,HumanoidInfoLine_r;
      
	rule<ScannerT> NIavatarSize_r, NIHeadlight_r, NIType_r, NavigationInfo_r;

	rule<ScannerT> skyColor_r, Background_r, ViewpointOri_r, 
	  ViewpointPos_r, Viewpoint_r, EntryPoint;

	rule<ScannerT> const& start() const {return EntryPoint;}

      }; // end of definition.
      
      explicit SpiritOpenHRP(const tActions &lactions = tActions()):
	actions(lactions)
      {
      };
      
      const tActions & actions;

    };

  };
};
