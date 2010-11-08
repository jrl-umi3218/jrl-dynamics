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

/* @doc Object used to parse a VRML file describing a robot. */
#if BOOST_VERSION < 104000
#include <boost/spirit.hpp>
#include <boost/spirit/phoenix/binders.hpp>
#include <boost/spirit/utility/chset.hpp>
#else
#include <boost/spirit/include/classic.hpp>
#include <boost/spirit/include/phoenix1_binders.hpp>
#include <boost/spirit/include/classic_chset.hpp>
#endif

#include <boost/lambda/lambda.hpp>
#include <boost/lambda/bind.hpp>

//#include "SpiritVRMLReader.t.cpp"
using namespace std;
using namespace boost::spirit;

#if BOOST_VERSION < 104000
using namespace boost::spirit::utility;
#else
using namespace boost::spirit::classic;
using namespace boost::spirit::classic::utility;
#endif

using namespace phoenix;

namespace dynamicsJRLJapan
{
  namespace VRMLReader
  {

    std::ostream& operator<<(std::ostream& out, file_position const& lc)
    {
      return out <<
	"\nFile:\t" << lc.file <<
	"\nLine:\t" << lc.line <<
	"\nCol:\t" << lc.column << endl;
    }

    struct error_report_parser {
      char const* eol_msg;
      char const* msg;

      error_report_parser(char const* eol_msg_, char const* msg_):
        eol_msg(eol_msg_),
        msg    (msg_)
      {}

      typedef struct boost::spirit::classic::nil_t result_t;

      template <typename ScannerT>
      int
      operator()(ScannerT const& scan, result_t& /*result*/) const
      {
        if (scan.at_end()) {
	  if (eol_msg) {
	    file_position fpos = scan.first.get_position();
	    cerr << fpos << eol_msg << endl;
	  }
        } else {
	  if (msg) {
	    file_position fpos = scan.first.get_position();
	    cerr << fpos << msg << endl;
	  }
        }

        return -1; // Fail.
      }

    };
    typedef functor_parser<error_report_parser> error_report_p;

    error_report_p
    error_badnumber_or_eol =
		    error_report_parser("Expecting a number, but found the end of the file\n",
					"Expecting a number, but found something else\n");

    error_report_p error_real =
		    error_report_parser(0,"Expecting a real, but found something else\n");

    error_report_p error_badnumber =
		    error_report_parser(0,"Expecting a number, but found something else\n");

    error_report_p error_comma =
		    error_report_parser(0,"Expecting a comma, but found something else\n");
  };
};
