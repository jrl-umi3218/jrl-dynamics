/* @doc Object used to parse Specificities Information

   Copyright (c) 2010, 

   @author : 
   Olivier Stasse
   
   JRL-Japan, CNRS/AIST

   All rights reserved.
   
   Please refers to file License.txt for details on the license.

*/
#ifndef _HSHUMANOIDNODEPARSER_HPP_
#define _HSHUMANOIDNODEPARSER_HPP_

/*! Framework specfic includes */
#include <Debug.h>

/*! Boost specific includes */
#include <boost/config/warning_disable.hpp>
#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/include/phoenix_core.hpp>
#include <boost/spirit/include/phoenix_operator.hpp>
#include <boost/spirit/include/phoenix_fusion.hpp>
#include <boost/spirit/include/phoenix_stl.hpp>
#include <boost/fusion/include/adapt_struct.hpp>
#include <boost/variant/recursive_variant.hpp>
#include <boost/foreach.hpp>

/*! System Includes */
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include "HumanoidSpecificitiesData.h"

#include "HSArmNodeParser.hpp"
#include "HSFootNodeParser.hpp"
#include "HSLegNodeParser.hpp"
#include "HSFootNodeParser.hpp"
#include "HSHandNodeParser.hpp"
#include "HSWaistNodeParser.hpp"


namespace dynamicsJRLJapan {
  namespace HumanoidSpecificitiesData {
    
    namespace fusion = boost::fusion;
    namespace phoenix = boost::phoenix;
    namespace qi = boost::spirit::qi;
    namespace ascii = boost::spirit::ascii;

    // Foot parser.

    template <typename Iterator>
    struct HumanoidNode_parser : 
      qi::grammar<Iterator, HumanoidNode(), ascii::space_type>
    {
     HumanoidNode_parser() : HumanoidNode_parser::base_type(start)
      {
        using qi::double_;
	using qi::lit;
	using qi::lexeme;
	using qi::char_;
	
	quoted_string %= lexeme['"' >> +(char_ - '"') >> '"'];

	starthn_tag %= '<' >> lit("Humanoid") 
			   >> lit("name")
			   >> '=' 
			   >> quoted_string >> '>';

	feet_parser %=
	  '<' >>  lit("Feet") >>  '>'  >>
	  '<' >>  lit("Right") >>  '>'  >>
	  FootNode_parser >> 
	  "</" >>  lit("Right") >>  '>'  >>
	  '<' >>  lit("Left") >>  '>'  >>
	  FootNode_parser >> 
	  "</" >>  lit("Left") >>  '>'  >>
	  "</" >>  lit("Feet") >>  '>';

	legs_parser %=
	  '<' >>  lit("Legs") >>  '>'  >>
	  '<' >>  lit("Right") >>  '>'  >>
	  LegNode_parser >> 
	  "</" >>  lit("Right") >>  '>'  >>
	  '<' >>  lit("Left") >>  '>'  >>
	  LegNode_parser >> 
	  "</" >>  lit("Left") >>  '>'  >>
	  "</" >>  lit("Legs") >>  '>';
	
	hands_parser %=
	  '<' >>  lit("Hands") >>  '>'  >>
	  '<' >>  lit("Right") >>  '>'  >>
	  HandNode_parser >> 
	  "</" >>  lit("Right") >>  '>'  >>
	  '<' >>  lit("Left") >>  '>'  >>
	  HandNode_parser >> 
	  "</" >>  lit("Left") >>  '>'  >>
	  "</" >>  lit("Hands") >>  '>';

	arms_parser %=
	  '<' >>  lit("Arms") >>  '>'  >>
	  '<' >>  lit("Right") >>  '>'  >>
	  ArmNode_parser >> 
	  "</" >>  lit("Right") >>  '>'  >>
	  '<' >>  lit("Left") >>  '>'  >>
	  ArmNode_parser >> 
	  "</" >>  lit("Left") >>  '>'  >>
	  "</" >>  lit("Arms") >>  '>';
	
	head_parser %= 
	  "</" >>  lit("Head") >>  '>'  >>
	  serialchain_parser >> 
	  "</" >>  lit("Head") >>  '>';

	chest_parser %= 
	  "</" >>  lit("Chest") >>  '>'  >>
	  serialchain_parser >> 
	  "</" >>  lit("Chest") >>  '>';
	  
	
	start %= starthn_tag >> 
	  feet_parser >> 
	  waist_parser >> 
	  legs_parser >>
	  hands_parser >>
	  arms_parser >> 
	  head_parser >>
	  chest_parser >>
	  "</" >>  lit("Humanoid") >>  '>' ;
	  
      }

      qi::rule<Iterator, HumanoidNode(), ascii::space_type> start;
      qi::rule<Iterator, FootNode(), ascii::space_type> feet_parser;
      qi::rule<Iterator, LegNode(), ascii::space_type> legs_parser;
      qi::rule<Iterator, HandNode(), ascii::space_type> hands_parser;
      qi::rule<Iterator, ArmNode(), ascii::space_type> arms_parser;
      qi::rule<Iterator, SerialChain(), ascii::space_type> head_parser;
      qi::rule<Iterator, SerialChain(), ascii::space_type> chest_parser;
      
      qi::rule<Iterator, std::string(), ascii::space_type> quoted_string;
      qi::rule<Iterator, std::string(), ascii::space_type> starthn_tag;
      struct SerialChain_parser<Iterator> serialchain_parser;
      struct FootNode_parser<Iterator> FootNode_parser;
      struct LegNode_parser<Iterator> LegNode_parser;
      struct HandNode_parser<Iterator> HandNode_parser;
      struct ArmNode_parser<Iterator> ArmNode_parser;
      struct WaistNode_parser<Iterator> waist_parser;
    };



  };
};

#endif /* _HSHUMANOIDNODEPARSER_HPP_ */
