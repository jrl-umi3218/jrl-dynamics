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
#ifndef _HSARMNODEPARSER_HPP_
#define _HSARMNODEPARSER_HPP_

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
    struct ArmNode_parser :
      qi::grammar<Iterator, ArmNode(), ascii::space_type>
    {
      ArmNode_parser() : ArmNode_parser::base_type(start)
      {
        using qi::double_;
	using qi::lit;

	UpperArmLength_parser %= '<' >>  lit("UpperArmLength") >>  '>'
				     >>  double_ // Implicit rule to fill upper arm length.
				     >>  lit("</") >>  lit("UpperArmLength") >>  '>' ;

	ForeArmLength_parser %= '<' >> lit("ForeArmLength") >>  '>'
				    >> double_ // Implicit rule to fill fore Arm length.
				    >> lit("</") >>  lit("ForeArmLength") >>  '>' ;

	start %= UpperArmLength_parser >>
	  ForeArmLength_parser >>
	  serialchain_parser;

      }

      qi::rule<Iterator, ArmNode(), ascii::space_type> start;
      qi::rule<Iterator, double, ascii::space_type> UpperArmLength_parser, ForeArmLength_parser;
      struct SerialChain_parser<Iterator> serialchain_parser;
    };



  };
};

#endif /* _HSARMNODEPARSER_HPP_ */
