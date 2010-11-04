/*
 * Copyright 2010, 
 *
 * Olivier Stasse,
 * 
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
/* @doc Object used to parse a VRML file describing a robot.*/

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

#include <jrl/dynamics/dynamicsfactory.hh>

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
      
      if (!parse(first,last,aSpiritOpenHRP,aSkipGrammar).full)
	{
	  file_position fp_cur;
	  
	  // Store the current file position
	  fp_cur = last.get_position();
	  ODEBUG("Display - Current file: " << fp_cur.file );
	  ODEBUG( "Line   : " << fp_cur.line  
		  << " Column : " << fp_cur.column 
		  << endl);

	}
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
		      if (!parse(lfirst,llast,aSpiritOpenHRP,aSkipGrammar).full)
			{
			  file_position fp_cur;
			  
			  // Store the current file position
			  fp_cur = lfirst.get_position();
			  ODEBUG("Display - Current file: " << fp_cur.file );
			  ODEBUG( "Line   : " << fp_cur.line  
				   << " Column : " << fp_cur.column 
				   << endl);
			}

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


