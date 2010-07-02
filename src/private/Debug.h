/* @doc \file Debug information macros

   Copyright (c) 2009
   @author Olivier Stasse
   JRL-Japan, CNRS/AIST
 
   All rights reserved.

   Please refers to file License.txt for details on the license.

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
    cerr << __FILE << ":" << __LINE__ <<  x << endl; }
#else
#define ODEBUG(x)
#define ODEBUGL(x)
#endif

#ifdef DEBUG_MODE
#define ODEBUG4(x,y) { ofstream DebugFile; \
    DebugFile.open(y,ofstream::app); \
    DebugFile << __FILE << ":" << __LINE__ \
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
DebugFile <<  __FILE << ":" << __LINE__ << x << endl; \
DebugFile.close();}
#define _DEBUG_4_ACTIVATED_ 1

#else

#define RESETDEBUG4(y)
#define ODEBUG4(x,y)

#endif
