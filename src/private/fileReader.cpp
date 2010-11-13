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

/* @doc Object used to parse small config
   @author :
   Abderrahmane Kheddar

*/

#ifndef WIN32
#include <err.h>
#endif /*WIN32*/

#include <string.h>
#include <errno.h>
#include "fileReader.h"
#include "dynamicsGetLine.h"
#include <Debug.h>

namespace dynamicsJRLJapan
{

  void fscanfc(FILE *fichier, char *c)
  {
    errno=0;
    int n = fscanf(fichier, "%c", c);

    if ((n!=1) && (errno!=0))
      LTHROW("fscanf unable to read a character" );
  }

  void fscanfd(FILE *fichier, double *c)
  {
    errno=0;
    int n = fscanf(fichier, "%lf", c);

    if ((n!=1) && (errno!=0))
      LTHROW("fscanf unable to read a double");
  }

  void fscanfi(FILE *fichier, int *c)
  {
    errno=0;
    int n = fscanf(fichier, "%d", c);

    if ((n!=1) && (errno!=0))
      LTHROW("fscanf unable to read a double");
  }

  // Search for string
  char look_for(FILE *fichier, const char *str)
  {
    int i = 0;
    char c=0;
    bool Cont=false;

    do
      {

	fscanfc(fichier, &c);

	if (Cont && c=='\n')
	  Cont=false;
	else if (c=='#')
	  Cont=true;

	if (!Cont)
	  {
	    if (c == str[i] && i < (int)(strlen(str)))
	      i++;
	    else
	      {
		if (i == (int)strlen(str))
		  return 1;
		else
		  i = 0;
	      }
	  }
      }
    while (!feof(fichier));

    return 0;
  }

  bool immediatlyAppears(FILE* fichier, const char *str)
  {
    bool b = true;
    char c;
    fpos_t afpos_t;

    fgetpos(fichier,&afpos_t);
    for (int i=0; i<(int)(strlen(str)) && b; i++) {
      if(!feof(fichier))
	{
	  fscanfc(fichier, &c);
	  b = (c==str[i]);
	}
      else
	{
	  fsetpos(fichier,&afpos_t);
	  return 0;
	}
    }
    if (b==false)
      fsetpos(fichier,&afpos_t);

    if ((!strcmp(str,"ointId")) && (b==false))
      {
	char Buffer[124];
	int r=fread(Buffer,strlen(str),1,fichier);
	if (r<0)
	  perror("fread");
	
	cout << "Refused by immediatlyAppears" << Buffer << " ";
	fsetpos(fichier,&afpos_t);

      }

    return b;
  }

  int nextKeyWord(FILE* fich)
  {
    char c;

    while (!feof(fich))
      {
	fscanfc(fich, &c);

	switch (c) {
	case '[' :
	  return CROCHET_OUVRANT;
	  break;
	case ']' :
	  return CROCHET_FERMANT;
	  break;
	case 'c' :
	  if (immediatlyAppears(fich, "hildren")) {
	    return CHILDREN;
	  }
	  break;
	case 'J' :
	  if (immediatlyAppears(fich, "oint {")) {
	    return JOINT;
	  }
	  break;
	case 'D' :
	  if (immediatlyAppears(fich, "EF")) {
	    return DEF;
	  }
	  break;

	}
      }
    return -2;
  }

  int nextJointKeyWord(FILE* fichier)
  {
    char c;
    do {

      fscanfc(fichier, &c);
      switch (c) {
      case 'j' :
	if (immediatlyAppears(fichier, "ointAxis \"")) {
	  fscanfc(fichier, &c);
	  switch (c) {
	  case 'X' :
	    return AXE_X;
	  case 'Y' :
	    return AXE_Y;
	  case 'Z' :
	    return AXE_Z;
	  }
	}
	else if (immediatlyAppears(fichier, "ointId ")) {
	  return JOINT_ID;
	}
	break;
      case 't' :
	if (immediatlyAppears(fichier, "ranslation")) {
	  return JOINT_TRANSLATION;
	}
	break;
      case 'r' :
	if (immediatlyAppears(fichier, "otation")) {
	  return JOINT_ROTATION;
	}
	break;
      case 'u' :
	if (immediatlyAppears(fichier, "limit"))
	  {
	    //cout << "nextJointKeyWord : Detect ulimit " << endl;
	    return JOINT_ULIMIT;
	  }
	break;
      case 'l' :
	if (immediatlyAppears(fichier, "limit"))
	  {
	    //cout << "nextJointKeyWord : Detect llimit " << endl;
	    return JOINT_LLIMIT;
	  }
	break;
      case '#':
	{
	  char *buf=NULL;
	  size_t n=256;
#ifdef HAVE_GETLINE
	  getline(&buf, &n, fichier);
#else
	  buf = (char *)malloc(n);
	  if (!buf)
#ifndef WIN32
	    warnx("cannot allocate memory");
#else
	  {}
#endif /* WIN32 */
	  else
	    {
	      char * lnull=fgets(buf, n, fichier);
	      if (!lnull)
		std::cerr << "fgets error in fileReader.cpp" <<endl;
	    }
#endif /* HAVE_GETLINE */
	  if (buf) free(buf);
	}
	break;
      }

    } while (!feof(fichier));
    return 0;
  }

  int typeOfJoint(FILE* fichier)
  {
    look_for(fichier,"jointType");
    char c;
    fscanfc(fichier, &c);
    fscanfc(fichier, &c);
    switch (c) {
    case 'f' :
      if (immediatlyAppears(fichier,"ree")) {
	return -1;
      }
      break;
    case 'r' :
      if (immediatlyAppears(fichier,"otate")) {
	return 1;
      }
    }
    return 0;
  }
}
