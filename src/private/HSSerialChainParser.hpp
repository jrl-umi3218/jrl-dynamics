/* @doc Object used to parse Specificities Information

   Copyright (c) 2010, 

   @author : 
   Olivier Stasse
   
   JRL-Japan, CNRS/AIST

   All rights reserved.
   
   Please refers to file License.txt for details on the license.

*/
#ifndef _HSSERIALCHAINPARSER_HPP_
#define _HSSERIALCHAINPARSER_HPP_

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



    // Serial chain parser.

    template <typename Iterator>
    struct SerialChain_parser : 
      qi::grammar<Iterator, SerialChain(), ascii::space_type>
    {
      SerialChain_parser() : SerialChain_parser::base_type(start)
      {
        using qi::uint_;
        using qi::lit;
        using ascii::char_;
	
	jointnb_tag %= '<' >>  lit("JointNb") >>  '>' >>
	  uint_ >> // Implicit rule to fill in nbOfJoints.
	  lit("</") >>  lit("JointNb") >>  '>' ;
	

	
	jointid_tag %= '<' >>  lit("JointsID") >>  '>' >>
	  *uint_ >> // Implicit rule to fill in JointsID.
	  lit("</") >>  lit("JointsID") >>  '>' ;;
	
        start %= jointnb_tag >> jointid_tag;
      }
      
      qi::rule<Iterator, SerialChain(), ascii::space_type> start;
      qi::rule<Iterator, unsigned int , ascii::space_type> jointnb_tag;
      qi::rule<Iterator, unsigned int, ascii::space_type> jointid_tag;

    };
  };
};

#endif /* _HSSERIALCHAINPARSER_HPP_ */
