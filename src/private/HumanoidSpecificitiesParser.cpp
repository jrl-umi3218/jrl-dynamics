/*
 * Copyright 2010,
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


#include "HSHumanoidNodeParser.hpp"

namespace dynamicsJRLJapan {
  namespace HumanoidSpecificitiesData {

    namespace fusion = boost::fusion;
    namespace phoenix = boost::phoenix;
    namespace qi = boost::spirit::qi;
    namespace ascii = boost::spirit::ascii;

    int ReadXMLData2(std::string &aFileName)
    {
      std::ifstream in((char *)aFileName.c_str(), std::ios_base::in);

      if (!in)
	{
	  std::cerr << "Error: Could not open input file: "
		    << aFileName << std::endl;
	  return 1;
	}

      std::string storage; // We will read the contents here.
      in.unsetf(std::ios::skipws); // No white space skipping!
      std::copy(
		std::istream_iterator<char>(in),
		std::istream_iterator<char>(),
		std::back_inserter(storage));

      struct HumanoidNode_parser<std::string::const_iterator>
	hsxml; // Our grammar
      HumanoidNode ast; // Our tree

      using boost::spirit::ascii::space;
      std::string::const_iterator iter = storage.begin();
      std::string::const_iterator end = storage.end();

      // this will print something like: boost::fusion::vector2<int, double>
      display_attribute_of_parser(hsxml);

      bool r = phrase_parse(iter, end, hsxml, space, ast);

      if (r && iter == end)
	{
	  std::cout << "-------------------------\n";
	  std::cout << "Parsing succeeded\n";
	  std::cout << "-------------------------\n";
	  return 0;
	}
      else
	{
	  std::string::const_iterator some = iter+30;
	  std::string context(iter, (some>end)?end:some);
	  std::cout << "-------------------------\n";
	  std::cout << "Parsing failed\n";
	  std::cout << "stopped at: \": " << context << "...\"\n";
	  std::cout << "-------------------------\n";
	  return 1;
	}
    }
  }
}
