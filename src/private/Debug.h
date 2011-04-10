/*
 * Copyright 2009, 2010,
 *
 * Florent Lamiraux
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

/* @doc \file Debug information macros

*/
#include <string>
#include <exception>


#define LTHROW(x) \
  { \
    class Exception: public std::exception \
    { \
      virtual const char * what() const throw() \
      {  \
        return x; \
      } \
    }; \
  \
  Exception almsg; \
  throw almsg; }

#ifdef RESETDEBUG5
#undef RESETDEBUG5
#endif

#ifdef ODEBUG5
#undef ODEBUG5
#endif

#ifdef ODEBUG2
#undef ODEBUG2
#endif

#ifdef ODEBUG3
#undef ODEBUG3
#endif

#ifdef ODEBUG4
#undef ODEBUG4
#endif

#ifdef ODEBUG
#undef ODEBUG
#endif

#define RESETDEBUG5(y) { ofstream DebugFile;	\
    DebugFile.open(y,ofstream::out);		\
    DebugFile.close();}

#define ODEBUG5(x,y) { ofstream DebugFile;	\
    DebugFile.open(y,ofstream::app);		\
    DebugFile << __FILE__ << ":" << __LINE__ << ":" << x << endl;	\
    DebugFile.close();}

#define ODEBUG2(x)
#define ODEBUG3(x) cerr << __FILE__ << ":" << __LINE__ << x << endl
#define ODEBUG3Q(x) cerr << x << endl

#ifdef DEBUG_MODE

#define ODEBUG(x) cerr << __FILE__ << ":" << __LINE__ <<  x << endl
#define ODEBUGL(x,level) {\
  if (DEBUG_MODE>level) \
    cerr << __FILE__ << ":" << __LINE__ <<  x << endl; }
#else
#define ODEBUG(x)
#define ODEBUGL(x)
#endif

#ifdef DEBUG_MODE
#define ODEBUG4(x,y) { ofstream DebugFile; \
    DebugFile.open(y,ofstream::app); \
    DebugFile << __FILE__ << ":" << __LINE__ \
	      << x << endl; DebugFile.close();}
#define _DEBUG_4_ACTIVATED_ 1

#else

#define ODEBUG4(x,y)

#endif

#ifdef DEBUG_MODE

#define RESETDEBUG4(y) { \
ofstream DebugFile; \
DebugFile.open(y,ofstream::out); \
DebugFile.close();}

#define ODEBUG4(x,y) { \
ofstream DebugFile; \
DebugFile.open(y,ofstream::app); \
DebugFile <<  __FILE__ << ":" << __LINE__ << x << endl; \
DebugFile.close();}
#define _DEBUG_4_ACTIVATED_ 1

#else

#define RESETDEBUG4(y)
#define ODEBUG4(x,y)

#endif
