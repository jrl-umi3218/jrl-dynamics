/* @doc Functions to parse a VRML file.
   OS: Small modification to read other parameters such as
   joint name, id, and make it a bit more model versatile.

   Copyright (c) 2005-2006, 
   @author Adrien Escande, Francois Keith, Abderrahmane Kheddar, Olivier Stasse, Ramzi Sellouati
   
   JRL-Japan, CNRS/AIST

   Please refers to file License.txt for details on the license.

*/
#ifndef _DYN_JRLJAPAN_FILE_READER_H_
#define _DYN_JRLJAPAN_FILE_READER_H_

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
using namespace std;

namespace dynamicsJRLJapan
{
  const static int CROCHET_OUVRANT=0;
  const static int CROCHET_FERMANT=1;
  const static int CHILDREN=2;
  const static int JOINT=3;
  const static int DEF=4;
  
  const static int AXE_X = 0;
  const static int AXE_Y = 1;
  const static int AXE_Z = 2;
  const static int JOINT_TRANSLATION=3;
  const static int JOINT_ROTATION=4;
  const static int JOINT_ID=5;
  const static int JOINT_LLIMIT = 6;
  const static int JOINT_ULIMIT = 7;

  /// Function to look for an object.
  char look_for(FILE* fichier, const char *str);
  
  /// Give back the word currently at the position of the file fichier in str.
  bool immediatlyAppears(FILE* fichier, const char *str) ;

  /// Look for the next keyword inside fich.
  int nextKeyWord(FILE* fich);
  
  /// Look for the next Joint keyword inside fich.
  int nextJointKeyWord(FILE* fichier);
  
  /// Returns the type of the next joint inside the file.
  int typeOfJoint(FILE* fichier);
};
#endif /* _DYN_JRLJAPAN_FILE_READER_H_ */
