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
/*! \page pcomputingquantities Computing Physical Quantities

For a given state vector \f$ {\bf x} = [{\bf r} \; {\bf q} \; \dot{\bf r} \; \dot{\bf q} \; \ddot{\bf r} \; \ddot{\bf q}]\f$, 
it is possible to compute the following quantities for a body:
<ul>
   <li>Its position in the world reference frame</li>
   <li>Its linear and angular velocity in the world 
   reference frame and in its own reference frame</li>
   <li></li>
</ul>
