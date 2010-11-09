/*
 * Copyright 2008, 2009, 2010,
 *
 * Francois Keith
 * Florent Lamiraux
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
 */

#ifndef DYNAMICSJRLJAPAN_DLL_H
#define DYNAMICSJRLJAPAN_DLL_H

#if defined (WIN32)
#  ifdef jrl_dynamics_EXPORTS
#    define DYN_JRL_JAPAN_EXPORT __declspec(dllexport)
#  else
#    define DYN_JRL_JAPAN_EXPORT __declspec(dllimport)
#  endif
#else
#  define DYN_JRL_JAPAN_EXPORT
#endif

#endif /* DYNAMICSJRLJAPAN_DLL_H */
