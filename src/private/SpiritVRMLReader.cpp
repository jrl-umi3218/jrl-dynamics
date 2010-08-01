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
#include <map>
#include <string.h>
#include <boost/spirit/include/classic.hpp>
#include <boost/spirit/include/classic_actor.hpp>
#include <boost/spirit/include/classic_dynamic.hpp>
#include <boost/spirit/include/phoenix1.hpp>
#include <boost/spirit/include/classic_core.hpp>
#include <boost/spirit/include/classic_position_iterator.hpp>
#include <boost/spirit/include/classic_functor_parser.hpp>

#include "SpiritVRMLGrammar.hpp"
#include "SpiritVRMLActions.hpp"

namespace dynamicsJRLJapan 
{
  namespace VRMLReader
  {
  
    int ParseVRMLFile(MultiBody *aMB, 
		      std::string aFileName,
		      vector<BodyGeometricalData> &aListOfURLs)
    {
      if (aFileName == std::string("")) {
	std::cerr << "SpiritVRMLReader: Filename is empty." << std::endl;
	return 0;
      }
      ifstream aif;
      struct s_DataForParsing DataForParsing;

      SkipGrammar aSkipGrammar;
      SpiritOpenHRP<Actions> aSpiritOpenHRP;
      
      aif.open(aFileName.c_str(),ifstream::in|ifstream::binary);
      if (!aif.is_open())
	{
	  cout << "Unable to open file "<< aFileName<< endl;
	  return 0;
	}
      else {
	/*	if (aSpiritOpenHRP.getVerbose()>10)
	  {
	    cout << "Succeeded in opening " << aFileName <<endl;
	    } */
      }
      unsigned int length;
  
      // get length of file:
      aif.seekg (0, ios::end);
      length = aif.tellg();

      aif.seekg (0, ios::beg);
  
      // allocate memory:
      char * buffer = new char [length+1];
      // read data as a block:
      aif.read(buffer,length);
      aif.close();
  
      aif.open(aFileName.c_str(),ifstream::in|ifstream::binary);

      /*      aSpiritOpenHRP.Init(aMB,
			  &DataForParsing,
			  &aListOfURLs);
			  aSpiritOpenHRP.setVerbose(0); */

      typedef multi_pass<istreambuf_iterator<char> > multi_pass_iterator_t;
      typedef istream::char_type char_t;
      
      multi_pass_iterator_t
        in_begin(make_multi_pass(istreambuf_iterator<char_t>(aif))),
        in_end(make_multi_pass(istreambuf_iterator<char_t>()));
      
      typedef position_iterator<multi_pass_iterator_t> iterator_t;
      
      iterator_t first(in_begin, in_end, aFileName), last;

      parse(first,last,aSpiritOpenHRP,aSkipGrammar);
      
      /*
      if (0){
	using namespace boost::spirit;
	using namespace std;
	
	ifstream in(aFileName.c_str()); // we get our input from this file
	
	typedef char char_t;
	typedef multi_pass<istreambuf_iterator<char_t> > iterator_t;
	
	typedef skip_parser_iteration_policy<space_parser> iter_policy_t;
	typedef scanner_policies<iter_policy_t> scanner_policies_t;
	typedef scanner<iterator_t, scanner_policies_t> scanner_t;
	
	typedef rule<scanner_t> rule_t;
	
	iter_policy_t iter_policy(space_p);
	scanner_policies_t policies(iter_policy);
	iterator_t first(make_multi_pass(std::istreambuf_iterator<char_t>(in)));
	
	scanner_t scan(first, 
		       make_multi_pass(std::istreambuf_iterator<char_t>()),
		       policies);
	
	match<>     m = aSpiritOpenHRP.parse(scan);
	}*/
      /*
      aif.open(aFileName.c_str(),ifstream::in|ifstream::binary);      
      vector<char> vec;
      std::copy(istream_iterator<char>(aif),
		istream_iterator<char>(),
		std::back_inserter(vec));
      aif.close();
      */
     /*

      vector<char>::const_iterator first=vec.begin();
      vector<char>::const_iterator last=vec.end();
      parse_info<vector<char>::const_iterator> info 
	= parse(first,last,aSpiritOpenHRP,aSkipGrammar);
     */
      //      parse(buffer,aSpiritOpenHRP,aSkipGrammar).full;
      
      delete [] buffer;
      return 1;

    };
  };
};


