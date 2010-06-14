/*! \page pagerneaspatial The Recursive Newton-Euler Algorithm algorithm using spatial vectors
      
      We recall here briefly this algorithm for a better understanding.

      \f$inputs: {\bf q},\dot{\bf q}, \ddot{\bf q}, \textit{model}, {^0{\bf f}{^e}{_i}}\f$ <br>
      \f$output: \tau, {\bf f} \f$ <br>
      \f$\textit{model data}: N_B, jtype(i), p(i), {\bf X}_L(i), I_i \f$ <br>
      <br>
      <ol>
        <li>\f${\bf v}_0 = {\bf 0}\f$ </li>
	<li>\f${\bf a}_o = -{\bf a}_g\f$</li>
	<li>\f$for \; i=1\; to \; N_b \; do\f$<br>
	  <ol>
	    <li>\f${\bf X}_J(i) = xjcalc(jtype(i),{\bf q}_i)\f$</li>
	    <li>\f${^i{\bf X}}_{p(i)} = {\bf X}_J(i){\bf X}_L(i)\f$</li>
  	    <li> \f$\text{if } p(i)\neq 0 \text{ then }\f$
	       <ol>
	         <li> \f${^i{\bf X}_0} = {^i{\bf X}_{p(i)}} {^{p(i)}{\bf X}_0} \f$</li>
	       </ol>
            </li>
	    <li> \f$\text{end}\f$</li>
	    <li> \f${\bf \Phi}_i = pcalc(jtype(i),{\bf q}_i)\f$</li>
	    <li> \f${\mathring{\bf \Phi}}c_i = pdcalc(jtype(i),{\bf q}_i)\f$</li>
	    <li> \f${\bf v}_i = {^i{\bf X}_{p(i)}}{\bf v}_{p(i)} + \Phi_i\dot{q}_i\f$</li>
	  </ol> 
      </ol>
	

      
*/
