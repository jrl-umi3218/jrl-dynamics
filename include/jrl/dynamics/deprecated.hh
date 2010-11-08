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
 *
 */

#ifndef JRL_DEPRECATED_H
#define JRL_DEPRECATED_H

// This macro allow a generic deprecation of the methods
// It is based on the code found at
// http://stackoverflow.com/questions/295120/c-mark-as-deprecated/295154
#ifdef __GNUC__
#define JRLDEPRECATED(func) func __attribute__ ((deprecated))
#elif defined(_MSC_VER)
#define JRLDEPRECATED(func) __declspec(deprecated) func
#else
#pragma message("WARNING: You need to implement JRLDEPRECATED for this compiler")
#define JRLDEPRECATED(func) func
#endif
//

#endif
