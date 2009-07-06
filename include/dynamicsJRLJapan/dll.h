/*
 *   Copyright (c) 2009 CNRS-AIST
 *
 *   Research carried out within the scope of the Associated
 *   International Laboratory: Joint Japanese-French Robotics
 *   Laboratory (JRL)
 *
 *   Author: Florent Lamiraux
 *
 */

#ifndef DYNAMICSJRLJAPAN_DLL_H
#define DYNAMICSJRLJAPAN_DLL_H

#if defined (WIN32)
#  ifdef dynamicsJRLJapan_EXPORTS 
#    define DYN_JRL_JAPAN_EXPORT __declspec(dllexport)
#  else  
#    define DYN_JRL_JAPAN_EXPORT __declspec(dllimport)
#  endif 
#else
#  define DYN_JRL_JAPAN_EXPORT
#endif

#endif /* DYNAMICSJRLJAPAN_DLL_H */
