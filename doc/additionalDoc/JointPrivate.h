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
/*! \class JointPrivate

  \section Rationale
      This class implements an abstract Joint class with the following functionnalities:
      <ul>
        <li>Normalization around the x-axis for the following joints:
	  <ul>
	    <li> Rotation</li>
	    <li> Slide </li>
	  </ul>
        </li>
	<li> A default Spatial based implementation of the Recursive Newton-Euler Algorithm (RNEA).</li>
        <li> An implementation of the abstractRobotDynamics interface. </li>
      </ul>
      
      An optimized derivation of the RNEA is possible by overloading the following methods:
      <ul> 
        <li> updateTransformation() </li>
	<li> updateVelocity() </li>
	<li> updateAcceleration() </li>
	<li> updateWorlCoMPosition() </li>
	<li> updateAccelerationCoM() </li>
	<li> updateMomentum() </li>
	<li> updateTorqueAndForce() </li>
      </ul>
      Each of this method is called only if appropriate flags are set inside
      the library. This allows partial computation of the quantities.

      
*/
