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

  \section The Recursive Newton-Euler Algorithm algorithm using spatial vectors
      
      We recall here briefly this algorithm for a better understanding.

      \f$inputs: {\bf q},\dot{\bf q}, \ddot{\bf q}, \textit{model}, {^0{\bf f}{^e}{_i}}\f$ <br>
      \f$output: \tau, {\bf f} \f$ <br>
      \f$\textit{model data}: N_B, jtype(i), p(i), {\bf X}_L(i), I_i \f$ <br>
      <br>
      \f${\bf v}_0 = {\bf 0}\f$ <br>
      \f${\bf a}_o = -{\bf a}_g\f$<br>
      \f$for \; i=1\; to \; N_b \; do\f$<br>
      \f$\;\; {\bf X}_J(i) = xjcalc(jtype(i),{\bf q}_i)\f$<br>
      
*/
