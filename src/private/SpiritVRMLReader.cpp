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

#include <dynamicsJRLJapan/dynamicsJRLJapanFactory.h>

#include "SpiritVRMLGrammar.hpp"
#include "SpiritVRMLActions.hpp"

//#include "DynamicMultiBodyCopy.h"

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

      SkipGrammar aSkipGrammar;
      SpiritOpenHRP< Actions > aSpiritOpenHRP;
      
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

      typedef multi_pass<istreambuf_iterator<char> > multi_pass_iterator_t;
      typedef istream::char_type char_t;
      
      multi_pass_iterator_t
        in_begin(make_multi_pass(istreambuf_iterator<char_t>(aif))),
        in_end(make_multi_pass(istreambuf_iterator<char_t>()));
      
      typedef position_iterator<multi_pass_iterator_t> iterator_t;
      
      iterator_t first(in_begin, in_end, aFileName), last;
      
      parse(first,last,aSpiritOpenHRP,aSkipGrammar);
      
      delete [] buffer;

      MultiBodyCopy aDMBC;
      aDMBC.PerformCopyFromJointsTree(&aSpiritOpenHRP.actions.m_DataForParsing.m_MultiBody,
				      aMB);
      
      return 1;

    };
  };
};


