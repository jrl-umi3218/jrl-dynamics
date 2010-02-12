/* @doc Object used to parse Specificities Information

   Copyright (c) 2010, 

   @author : 
   Olivier Stasse
   
   JRL-Japan, CNRS/AIST

   All rights reserved.
   
   Please refers to file License.txt for details on the license.

*/
#ifndef _HSHANDNODEPARSER_HPP_
#define _HSHANDNODEPARSER_HPP_

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

namespace dynamicsJRLJapan {
  namespace HumanoidSpecificitiesData {
    
    namespace fusion = boost::fusion;
    namespace phoenix = boost::phoenix;
    namespace qi = boost::spirit::qi;
    namespace ascii = boost::spirit::ascii;

    // Foot parser.

    template <typename Iterator>
    struct HandNode_parser : 
      qi::grammar<Iterator, HandNode(), ascii::space_type>
    {
      HandNode_parser() : HandNode_parser::base_type(start)
      {
        using qi::double_;
	using qi::lit;

	starthl_tag %= '<' >>  lit("Center") >>  '>' ;
	endhl_tag %= lit("</") >>  lit("Center") >>  '>' ;

	center_parser %= starthl_tag >> 
	  double_ >> // Implicit rule to fill center.
	  double_ >>
	  double_ >>
	  endhl_tag;

	okay_parser %= '<' >>  lit("okayAxis") >>  '>' >>
	  double_ >> // Implicit rule to fill okay.
	  double_ >>
	  double_ >>
	  double_ >> lit("</") >>  lit("okayAxis") >>  '>' ;
	  
	showing_parser %= '<' >>  lit("showingAxis") >>  '>' >>
	  double_ >> // Implicit rule to fill showing.
	  double_ >>
	  double_ >>
	  double_ >> lit("</") >>  lit("showingAxis") >>  '>' ;

	palm_parser %= '<' >>  lit("palmAxis") >>  '>' >>
	  double_ >> // Implicit rule to fill palm.
	  double_ >>
	  double_ >>
	  double_ >> lit("</") >>  lit("palmAxis'") >>  '>' ;
	
        start %= center_parser >> 
	  okay_parser >>
	  showing_parser >>
	  palm_parser;
      }

      qi::rule<Iterator, HandNode(), ascii::space_type> start;
      qi::rule<Iterator, double[3], ascii::space_type> center_parser,
	okay_parser, showing_parser, palm_parser;
      qi::rule<Iterator, std::string(), ascii::space_type>
        starthl_tag, endhl_tag;
    };



  };
};

#endif /* _HSHANDNODEPARSER_HPP_ */
