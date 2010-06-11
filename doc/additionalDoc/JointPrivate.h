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
