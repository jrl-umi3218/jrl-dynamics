/* Computation of the dynamic aspect for a humanoid robot.
  
   Copyright (c) 2007-2009
   @author Olivier Stasse.
   
   JRL-Japan, CNRS/AIST
   
   All rights reserved.

   Please refers to file License.txt for details on the license.   
*/
#ifndef _SPIRIT_VRML_READER_H_
#define _SPIRIT_VRML_READER_H_

#include <string>
#include "MultiBody.h"


namespace dynamicsJRLJapan
{
  namespace VRMLReader 
  {
    int ParseVRMLFile(MultiBody *aMB, std::string FileName);
  };
};

#endif /* _SPIRIT_VRML_READER_H_ */
