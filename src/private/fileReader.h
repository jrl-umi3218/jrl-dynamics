/*
 * Copyright 2009, 2010,
 *
 * Abderrahmane Kheddar
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

/* @doc Functions to parse a VRML file.
   OS: Small modification to read other parameters such as
   joint name, id, and make it a bit more model versatile.

   Copyright (c) 2005-2006,

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

  /// Function to read a character str from fichier
  void fscanfc(FILE *fichier, char *str);

  /// Function to read a double from fichier
  void fscanfd(FILE *fichier, double *d);

  /// Function to read an integer from fichier
  void fscanfi(FILE *fichier, int *d);

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
}
#endif /* _DYN_JRLJAPAN_FILE_READER_H_ */
