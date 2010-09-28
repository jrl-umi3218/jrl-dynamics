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
#ifndef _SPIRIT_VRML_READER_H_
#define _SPIRIT_VRML_READER_H_

#include <string>
#include <vector>
#include "MultiBody.h"
#include "dynamicsJRLJapan/dynamicsJRLJapanFactory.h"

namespace dynamicsJRLJapan
{
  namespace VRMLReader 
  {
    int ParseVRMLFile(MultiBody *aMB, 
		      std::string FileName, 
		      std::vector<BodyGeometricalData> & VectorOfURLs,
		      bool ReadGeometry=false);
  };
};

#endif /* _SPIRIT_VRML_READER_H_ */
