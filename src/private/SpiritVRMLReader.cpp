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

#include "SpiritVRMLGrammar.hpp"
#include "SpiritVRMLActions.hpp"

#include <boost/spirit/include/classic.hpp>
#include <boost/spirit/include/classic_actor.hpp>
#include <boost/spirit/include/classic_dynamic.hpp>
#include <boost/spirit/include/phoenix1.hpp>
#include <boost/spirit/include/classic_core.hpp>
#include <boost/spirit/include/classic_position_iterator.hpp>
#include <boost/spirit/include/classic_functor_parser.hpp>

#include <dynamicsJRLJapan/dynamicsJRLJapanFactory.h>

//#include "DynamicMultiBodyCopy.h"

namespace dynamicsJRLJapan 
{
  namespace VRMLReader
  {
    
    int ParseVRMLFile(MultiBody *aMB, 
		      std::string aFileName,
		      vector<BodyGeometricalData> &aListOfURLs,
		      bool ReadGeometry)
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
    
      aif.open(aFileName.c_str(),ifstream::in|ifstream::binary);

      typedef multi_pass<istreambuf_iterator<char> > multi_pass_iterator_t;
      typedef istream::char_type char_t;
      
      multi_pass_iterator_t
        in_begin(make_multi_pass(istreambuf_iterator<char_t>(aif))),
        in_end(make_multi_pass(istreambuf_iterator<char_t>()));
      
      typedef position_iterator<multi_pass_iterator_t> iterator_t;
      
      iterator_t first(in_begin, in_end, aFileName), last;
      
      parse(first,last,aSpiritOpenHRP,aSkipGrammar);
      aif.close();

      if (ReadGeometry)
	{
	  // Iterate over the included files if there is some. 
	  vector<BodyGeometricalData*> aLOU = aSpiritOpenHRP.actions.m_DataForParsing.m_ListOfURLs;
	  
	  string Path;
	  unsigned int npos = aFileName.find_last_of('/');
	  Path = aFileName.substr(0,npos+1);
	  
	  ODEBUG( "Path: " << Path 
		  << " Size of m_ListOfURLs: " 
		  << aSpiritOpenHRP.actions.m_DataForParsing.m_ListOfURLs.size());

	  for(unsigned int iIndexBDG=0;
	      iIndexBDG<aLOU.size();
	      iIndexBDG++)
	    {
	      const vector<string > URLs = aLOU[iIndexBDG]->getURLs();
	      ODEBUG(" i: " << iIndexBDG << " URLs.size():" << URLs.size());
	      for(unsigned int j=0;j<URLs.size();j++)
		{
		  string GeomFileName = Path + URLs[j];
		  aif.open(GeomFileName.c_str(),ifstream::in|ifstream::binary);

		  if (!aif.is_open())
		    {
		      ODEBUG(" Unable to open :" << GeomFileName );
		    }
		  else
		    {
		      ODEBUG3( "Open :" << GeomFileName );
		      multi_pass_iterator_t
			lin_begin(make_multi_pass(istreambuf_iterator<char_t>(aif))),
			lin_end(make_multi_pass(istreambuf_iterator<char_t>()));
		  
		      iterator_t lfirst(lin_begin, lin_end, URLs[j]), llast;
		      *aSpiritOpenHRP.actions.m_DataForParsing.m_LOUIndex = 
			iIndexBDG;
		      parse(lfirst,llast,aSpiritOpenHRP,aSkipGrammar);
		    }
		  aif.close();
		}
	    }
	  
	  // Copy the list of URLS.
	  aListOfURLs.resize(aLOU.size());
	  vector<BodyGeometricalData *>::iterator it_BGD = aLOU.begin();
	  unsigned int i=0;
	  while (it_BGD!= aLOU.end())
	    {
	      aListOfURLs[i] = *(*it_BGD);
	      i++;
	      it_BGD++;
	    }
	};      

      // Copy multibody structure.
      *aMB = aSpiritOpenHRP.actions.m_DataForParsing.m_MultiBody;

      
      return 1;
    };
  };
};


