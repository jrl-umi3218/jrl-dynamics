/*
 * Copyright 2009, 2010, 
 *
 * Olivier Stasse
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
/* @doc Object used to parse Specificities Information
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

//#include "HSArmNodeParser.hpp"
//#include "HSFootNodeParser.hpp"
//#include "HSLegNodeParser.hpp"
//#include "HSFootNodeParser.hpp"
//#include "HSHandNodeParser.hpp"
//#include "HSWaistNodeParser.hpp"

namespace dhs=dynamicsJRLJapan::HumanoidSpecificitiesData;


// We need to tell fusion about our mini_xml struct
    // to make it a first-class fusion citizen
    BOOST_FUSION_ADAPT_STRUCT(
			      dhs::SerialChain,
			      (unsigned int, nbOfJoints)
			      (std::vector<int>, jointID)
			      )

    BOOST_FUSION_ADAPT_STRUCT(
			      dhs::FootNode,
			      (double, sizeX)
			      (double, sizeY)
			      (double, sizeZ)
			      (std::vector<double>, anklePosition)
			      (dhs::SerialChain, sChain)
			      )

    BOOST_FUSION_ADAPT_STRUCT(dhs::WaistNode,
			      (std::vector<double>, leftWaistToHip)
			      (std::vector<double>, rightWaistToHip)
			      (dhs::SerialChain, sChain)
			      )
    BOOST_FUSION_ADAPT_STRUCT(dhs::LegNode,
			      (std::vector<double>,hipLength)
			      (double, femurLength)
			      (double, tibiaLength)
			      (dhs::SerialChain, sChain)
			      )
    BOOST_FUSION_ADAPT_STRUCT(dhs::HandNode,
			      (std::vector<double>, center)
			      (std::vector<double>, okayAxis)
			      (std::vector<double>, showingAxis)
			      (std::vector<double>, palmAxis)
			      )
    BOOST_FUSION_ADAPT_STRUCT(dhs::ArmNode,
			      (double, upperArmLength)
			      (double, foreArmLength)
			      (dhs::SerialChain, sChain)
			      )
    BOOST_FUSION_ADAPT_STRUCT(dhs::HumanoidNode,
			      (std::string, Name)
			      (dhs::FootNode, rightFoot)
			      (dhs::FootNode, leftFoot)
			      (dhs::WaistNode, waist)
			      (dhs::LegNode, rightLeg)
			      (dhs::LegNode, leftLeg)
			      (dhs::HandNode, rightHand)
			      (dhs::HandNode, leftHand)
			      (dhs::ArmNode, rightArm)
			      (dhs::ArmNode, leftArm)
			      (dhs::SerialChain, head)
			      (dhs::SerialChain, chest)
			      )

namespace dynamicsJRLJapan {
  namespace HumanoidSpecificitiesData {
    
    namespace fusion = boost::fusion;
    namespace phoenix = boost::phoenix;
    namespace qi = boost::spirit::qi;
    namespace ascii = boost::spirit::ascii;
    
    template <typename Expr, typename Iterator = boost::spirit::unused_type>
    struct attribute_of_parser
    {
      typedef typename boost::spirit::result_of::compile<
        boost::spirit::qi::domain, Expr
	>::type parser_expression_type;
      
      typedef typename boost::spirit::traits::attribute_of<
        parser_expression_type, boost::spirit::unused_type, Iterator
	>::type type;
    };
    
    template <typename T>
    void display_attribute_of_parser(T const&)
    {
      typedef typename attribute_of_parser<T>::type attribute_type;
      std::cout << typeid(attribute_type).name() << std::endl;
    }


    // Serial chain parser.
    
    template <typename Iterator>
    struct SerialChain_parser : 
      qi::grammar<Iterator, SerialChain(), ascii::space_type>
    {
      SerialChain_parser() : SerialChain_parser::base_type(start)
      {
        using qi::uint_;
        using qi::int_;
        using qi::lit;
        using ascii::char_;
	
	start%= '<' >>  lit("JointNb") >>  '>' 
		    >>  uint_ // Implicit rule to fill in nbOfJoints.
		    >>  "</" >>  lit("JointNb") >>  '>' 
		    >> '<' >>  lit("JointsID") >>  '>' 
		    >> *int_ // Implicit rule to fill in JointsID.
		    >>  "</" >>  lit("JointsID") >>  '>' ;;
      }
      
      qi::rule<Iterator, SerialChain(), ascii::space_type> start;
      qi::rule<Iterator, unsigned int , ascii::space_type> jointnb_tag;
      qi::rule<Iterator, std::vector<unsigned int>, ascii::space_type> jointid_tag;

    };


    // Foot parser.

    template <typename Iterator>
    struct FootNode_parser : 
      qi::grammar<Iterator, FootNode(), ascii::space_type>
    {
      FootNode_parser() : FootNode_parser::base_type(start)
      {
        using qi::double_;
	using qi::lit;
        using qi::uint_;
        using ascii::char_;

	start %= '<' >>  lit("SizeX") >>  '>' 
		     >> double_  
		     >>  lit("</") >>  lit("SizeX") >>  '>' 
		     >> '<' >>  lit("SizeY") >>  '>' 
		     >> double_  
		     >>  lit("</") >>  lit("SizeY") >>  '>'
		     >> '<' >>  lit("SizeZ") >>  '>' 
		     >> double_  
		     >>  lit("</") >>  lit("SizeZ") >>  '>'
		     >> '<' >>  lit("AnklePosition") >>  '>' 
		     >> *double_
		     >> lit("</") >>  lit("AnklePosition") >>  '>' 
		     >> serialchain_parser;

      }

      qi::rule<Iterator, FootNode(), ascii::space_type> start;
      SerialChain_parser<Iterator> serialchain_parser;

    };

    // Leg Node Parser
    template <typename Iterator>
    struct LegNode_parser : 
      qi::grammar<Iterator, LegNode(), ascii::space_type>
    {
      LegNode_parser() : LegNode_parser::base_type(start)
      {
        using qi::double_;
	using qi::lit;


	start %= '<' >>  lit("HipLength") >>  '>' 
		     >> *double_  // Implicit rule to fill HipLength.
		     >>  lit("</") >>  lit("HipLength") >>  '>' 
		     >> '<' >>  lit("FemurLength") >>  '>' 
		     >> double_ >> lit("</") >>  lit("FemurLength") >>  '>' 
		     >>  '<' >>  lit("TibiaLength") >>  '>'
		     >>  double_ >> lit("</") >>  lit("TibiaLength") >>  '>' 
		     >> serialchain_parser;
      }

      qi::rule<Iterator, LegNode(), ascii::space_type> start;
      SerialChain_parser<Iterator> serialchain_parser;
    };

    // Hand Node parser.

    template <typename Iterator>
    struct HandNode_parser : 
      qi::grammar<Iterator, HandNode(), ascii::space_type>
    {
      HandNode_parser() : HandNode_parser::base_type(start)
      {
        using qi::double_;
	using qi::lit;


	start %= '<' >> lit("Center") >>  '>' 
		     >> *double_ // Implicit rule to fill center.
		     >> lit("</") >>  lit("Center") >>  '>'
		     >> '<' >>  lit("okayAxis") >>  '>' 
		     >> *double_ // Implicit rule to fill okay.
		     >> lit("</") >>  lit("okayAxis") >>  '>'
		     >> '<' >>  lit("showingAxis") >>  '>' 
		     >> *double_ // Implicit rule to fill showing.
		     >> lit("</") >>  lit("showingAxis") >>  '>'
		     >> '<' >>  lit("palmAxis") >>  '>' 
		     >> *double_ // Implicit rule to fill palm.
		     >> lit("</") >>  lit("palmAxis") >>  '>' ;
	
      }

      qi::rule<Iterator, HandNode(), ascii::space_type> start;
    };

    // Arm Node Parser
    template <typename Iterator>
    struct ArmNode_parser : 
      qi::grammar<Iterator, ArmNode(), ascii::space_type>
    {
      ArmNode_parser() : ArmNode_parser::base_type(start)
      {
        using qi::double_;
	using qi::lit;

	start %= '<' >> lit("UpperArmLength") >>  '>' 
		     >> double_ // Implicit rule to fill upper arm length.
		     >> lit("</") >>  lit("UpperArmLength") >>  '>' 
		     >> '<' >> lit("ForeArmLength") >>  '>' 
		     >> double_ // Implicit rule to fill fore Arm length.
		     >> lit("</") >>  lit("ForeArmLength") >>  '>' 
		     >> serialchain_parser;
	  
      }

      qi::rule<Iterator, ArmNode(), ascii::space_type> start;
      SerialChain_parser<Iterator> serialchain_parser;
    };

    // Waist Node parser

    template <typename Iterator>
    struct WaistNode_parser : 
      qi::grammar<Iterator, WaistNode(), ascii::space_type>
    {
      WaistNode_parser() : WaistNode_parser::base_type(start)
      {
        using qi::int_;
        using qi::lit;
        using qi::double_;
        using qi::lexeme;
        using ascii::char_;

	start %= '<' >>  lit("Waist") >>  '>' 

		     >> '<' >>  lit("Right") >>  '>' 
		     >> '<' >>  lit("WaistToHip") >>  '>'  
		     >> *double_ // Implicit rule to fill in position
		     >>  lit("</") >>  lit("WaistToHip") >>  '>'
		     >> lit("</") >>  lit("Right") >>  '>' 

		     >> '<' >>  lit("Left") >>  '>'  
		     >> '<' >>  lit("WaistToHip") >>  '>'  
		     >> *double_ // Implicit rule to fill in position
		     >>  lit("</") >>  lit("WaistToHip") >>  '>'
		     >> lit("</") >>  lit("Left") >>  '>' 

		     >> serialchain_parser 
		     >> lit("</") >>  lit("Waist") >>  '>' ;
      }

      qi::rule<Iterator, WaistNode(), ascii::space_type> start;
      SerialChain_parser<Iterator> serialchain_parser;
    };

    // Arm Node Parser
    template <typename Iterator>
    struct HeadNode_parser : 
      qi::grammar<Iterator, SerialChain(), ascii::space_type>
    {
      HeadNode_parser() : HeadNode_parser::base_type(start)
      {
        using qi::double_;
	using qi::lit;

	start %=  '<' >>  lit("Head") >>  '>'   
		      >> serialchain_parser
		      >> "</" >>  lit("Head") >>  '>';
	  
      }

      qi::rule<Iterator, SerialChain(), ascii::space_type> start;
      SerialChain_parser<Iterator> serialchain_parser;
    };

    // Humanoid parser.

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
        using qi::uint_;
	
	quoted_string %= lexeme['"' >> +(char_ - '"') >> '"'];

	starthn_tag %= '<' >> lit("Humanoid") 
			   >> lit("name")
			   >> '=' 
			   >> quoted_string >> '>';

	rfeet_parser %= '<' >> lit("Right") >>  '>'  
			    >> aFootNode_parser 
			    >> "</" >>  lit("Right") >>  '>' ;
	lfeet_parser %= '<' >> lit("Left") >>  '>'  
			    >> aFootNode_parser 
			    >> "</" >>  lit("Left") >>  '>';

	rleg_parser %= '<' >>  lit("Right") >>  '>'  
			   >> aLegNode_parser 
			   >>  "</" >>  lit("Right") >>  '>';

	lleg_parser %= '<' >>  lit("Left") >>  '>'  
			   >> aLegNode_parser 
			   >> "</" >>  lit("Left") >>  '>';

	rhand_parser %= '<' >>  lit("Right") >>  '>'  
			    >> aHandNode_parser 
			    >> "</" >>  lit("Right") >>  '>' ;

	lhand_parser %= '<' >>  lit("Left") >>  '>'  
			    >> aHandNode_parser
			    >> "</" >>  lit("Left") >>  '>' ;

	rarm_parser %= '<' >>  lit("Right") >>  '>'  
			   >> aArmNode_parser 
			   >> "</" >>  lit("Right") >>  '>' ;

	larm_parser %= '<' >>  lit("Left") >>  '>'  
			   >> aArmNode_parser 
			   >> "</" >>  lit("Left") >>  '>';
	chest_parser %=  '<' >>  lit("Chest") >>  '>'

			      >> serialchain_parser  
			      >> "</" >>  lit("Chest") >>  '>' ;

	start %= starthn_tag 
	  >>  '<' >>  lit("Feet") >>  '>'  
	  >> rfeet_parser 
	  >> lfeet_parser 
	  >> "</" >>  lit("Feet") >>  '>' 
	  >> aWaist_parser 
	  >> '<' >>  lit("Legs") >>  '>'  
	  >> rleg_parser
	  >> lleg_parser
	  >> "</" >>  lit("Legs") >>  '>'  

	  >> '<' >>  lit("Hands") >>  '>' 
	  >> '<' >>  lit("Right") >>  '>'  
	  >> aHandNode_parser 
	  >> "</" >>  lit("Right") >>  '>' 
	  >> '<' >>  lit("Left") >>  '>'  
	  >> aHandNode_parser 
	  >> "</" >>  lit("Left") >>  '>'   
	  >> "</" >>  lit("Hands") >>  '>' 

	  >> '<' >>  lit("Wrists") >>  '>' 
	  >> '<' >>  lit("Right") >>  '>'  
	  >> aHandNode_parser 
	  >> "</" >>  lit("Right") >>  '>' 
	  >> '<' >>  lit("Left") >>  '>'  
	  >> aHandNode_parser 
	  >> "</" >>  lit("Left") >>  '>'   
	  >> "</" >>  lit("Wrists") >>  '>' 

	  >> '<' >>  lit("Arms") >>  '>'  
	  >> rarm_parser 
	  >> larm_parser
	  >> "</" >>  lit("Humanoid") >>  '>' ; 

	  
      }

      SerialChain_parser<Iterator> serialchain_parser;
      FootNode_parser<Iterator> aFootNode_parser;
      LegNode_parser<Iterator> aLegNode_parser;
      HandNode_parser<Iterator> aHandNode_parser;
      ArmNode_parser<Iterator> aArmNode_parser;
      WaistNode_parser<Iterator> aWaist_parser; 
      HeadNode_parser<Iterator> aHeadNode_parser; 

      qi::rule<Iterator, HumanoidNode(), ascii::space_type> start;
      qi::rule<Iterator, FootNode(), ascii::space_type> rfeet_parser,lfeet_parser;
      qi::rule<Iterator, LegNode(), ascii::space_type> lleg_parser,rleg_parser;
      qi::rule<Iterator, HandNode(), ascii::space_type> rhand_parser,lhand_parser;
      qi::rule<Iterator, ArmNode(), ascii::space_type> larm_parser,rarm_parser;
      // qi::rule<Iterator, SerialChain(), ascii::space_type> head_parser;
      qi::rule<Iterator, SerialChain(), ascii::space_type> chest_parser;
      
      qi::rule<Iterator, std::string(), ascii::space_type> quoted_string;
      qi::rule<Iterator, std::string(), ascii::space_type> starthn_tag;
      
    };

    // Intermediat Humanoid parser.

    template <typename Iterator>
    struct IHumanoidNode_parser : 
      qi::grammar<Iterator, HumanoidNode(), ascii::space_type>
    {
     IHumanoidNode_parser() : IHumanoidNode_parser::base_type(start)
      {
        using qi::double_;
	using qi::lit;
	using qi::lexeme;
	using qi::char_;
        using qi::uint_;
	
	quoted_string %= lexeme['"' >> +(char_ - '"') >> '"'];


	rfeet_parser %= '<' >> lit("Right") >>  '>'  
			    >> aFootNode_parser 
			    >> "</" >>  lit("Right") >>  '>' ;
	lfeet_parser %= '<' >> lit("Left") >>  '>'  
			    >> aFootNode_parser 
			    >> "</" >>  lit("Left") >>  '>';
	rleg_parser %= '<' >>  lit("Right") >>  '>'  
			   >> aLegNode_parser 
			   >>  "</" >>  lit("Right") >>  '>';

	lleg_parser %= '<' >>  lit("Left") >>  '>'  
			   >> aLegNode_parser 
			   >> "</" >>  lit("Left") >>  '>';

	rhand_parser %= '<' >>  lit("Right") >>  '>'  
			    >> aHandNode_parser 
			    >> "</" >>  lit("Right") >>  '>' ;

	lhand_parser %=  '<' >>  lit("Left") >>  '>' 
			     >> aHandNode_parser 
			     >> "</" >>  lit("Left") >>  '>' ;


	start %= '<' >> lit("Humanoid") 
		     >> lit("name")
		     >> '=' 
		     >> quoted_string >> '>'
		     >> '<' >>  lit("Feet") >>  '>'  
		     >> rfeet_parser 
		     >> lfeet_parser 
		     >> "</" >>  lit("Feet") >>  '>' 
		     >> aWaist_parser
		     >> '<' >>  lit("Legs") >>  '>'  
		     >> rleg_parser
		     >> lleg_parser
		     >> "</" >>  lit("Legs") >>  '>'  
		     >> '<' >>  lit("Hands") >>  '>' 
		     >> rhand_parser
		     >> lhand_parser
		     >> "</" >>  lit("Hands") >>  '>' ;

      }

      SerialChain_parser<Iterator> serialchain_parser;
      FootNode_parser<Iterator> aFootNode_parser;
      LegNode_parser<Iterator> aLegNode_parser;
      HandNode_parser<Iterator> aHandNode_parser;
      ArmNode_parser<Iterator> aArmNode_parser;
      WaistNode_parser<Iterator> aWaist_parser; 
      HeadNode_parser<Iterator> aHeadNode_parser; 

      qi::rule<Iterator, HumanoidNode(), ascii::space_type> start;
      qi::rule<Iterator, FootNode(), ascii::space_type> rfeet_parser,lfeet_parser;
      qi::rule<Iterator, LegNode(), ascii::space_type> lleg_parser,rleg_parser;
      qi::rule<Iterator, HandNode(), ascii::space_type> rhand_parser,lhand_parser;
      qi::rule<Iterator, ArmNode(), ascii::space_type> larm_parser,rarm_parser;
      // qi::rule<Iterator, SerialChain(), ascii::space_type> head_parser;
      qi::rule<Iterator, SerialChain(), ascii::space_type> chest_parser;
      
      qi::rule<Iterator, std::string(), ascii::space_type> quoted_string;
      qi::rule<Iterator, std::string(), ascii::space_type> starthn_tag;
      
    };



  };
};

#endif /* _HSHUMANOIDNODEPARSER_HPP_ */
