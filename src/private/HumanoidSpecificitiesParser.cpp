/* @doc Object used to parse Specificities Information

   Copyright (c) 2010, 

   @author : 
   Olivier Stasse
   
   JRL-Japan, CNRS/AIST

   All rights reserved.
   
   Please refers to file License.txt for details on the license.

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
