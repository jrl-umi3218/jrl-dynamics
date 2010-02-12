/* @doc Object used to parse Specificities Information

   Copyright (c) 2010, 

   @author : 
   Olivier Stasse
   
   JRL-Japan, CNRS/AIST

   All rights reserved.
   
   Please refers to file License.txt for details on the license.

*/
#ifndef _HSFOOTNODEPARSER_HPP_
#define _HSFOOTNODEPARSER_HPP_

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
#include "HSSerialChainParser.hpp"

namespace dynamicsJRLJapan {
  namespace HumanoidSpecificitiesData {
    
    namespace fusion = boost::fusion;
    namespace phoenix = boost::phoenix;
    namespace qi = boost::spirit::qi;
    namespace ascii = boost::spirit::ascii;

    // Foot parser.

    template <typename Iterator>
    struct FootNode_parser : 
      qi::grammar<Iterator, FootNode(), ascii::space_type>
    {
      FootNode_parser() : FootNode_parser::base_type(start)
      {
        using qi::int_;
        using qi::lit;
        using qi::double_;
        using qi::lexeme;
        using ascii::char_;

	starts_tag %= '<' >>  lit("SizeX")|
	  lit("SizeY")|
	  lit("SizeZ") >>  '>' ;
	endjs_tag %= lit("</") >>  lit("SizeX")|
	  lit("SizeY")|
	  lit("SizeZ") >>  '>' ;
	
	size_parser %= starts_tag >>
	  double_ >> // Implicit rule to fill in sizeX, sizeY, sizeZ.
	  endjs_tag ;
	
	startap_tag %= '<' >>  lit("AnklePosition") >>  '>' ;
	endap_tag %= lit("</") >>  lit("AnklePosition") >>  '>' ;

	ankleposition_parser %= 
	  startap_tag >>
	  double_ >> // Implicit rule to fill in anklePosition
	  double_ >>
	  double_ >> 
	  endap_tag;

	
        start %= size_parser >> 
	  ankleposition_parser >>
	  serialchain_parser;
      }

      qi::rule<Iterator, FootNode(), ascii::space_type> start;
      qi::rule<Iterator, std::string(), ascii::space_type> starts_tag,endjs_tag,
	startap_tag,endap_tag;
      qi::rule<Iterator, double, ascii::space_type> size_parser;
      qi::rule<Iterator, std::vector<double>, ascii::space_type> ankleposition_parser;

      struct SerialChain_parser<Iterator> serialchain_parser;
    };



  };
};

#endif /* _HSFOOTNODEPARSER_HPP_ */
