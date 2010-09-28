/*! \page pagerneaspatial The Recursive Newton-Euler Algorithm algorithm using spatial vectors
      
      We recall here briefly this algorithm for a better understanding.

      \f$inputs: {\bf q},\dot{\bf q}, \ddot{\bf q}, \textit{model}, {^0{\bf f}{^e}{_i}}\f$ <br>
      \f$output: \tau, {\bf f} \f$ <br>
      \f$\textit{model data}: N_B, jtype(i), p(i), {\bf X}_L(i), I_i \f$ <br>
      <br>
      <ol>
        <li>\f${\bf v}_0 = {\bf 0}\f$ </li>
	<li>\f${\bf a}_o = -{\bf a}_g\f$</li>
	<li>\f$for \; i=1\; to \; N_b \; then\f$<br>
	  <ol>
	    <li>\f${\bf X}_J(i) = xjcalc(jtype(i),{\bf q}_i)\f$</li>
	    <li>\f${^i{\bf X}}_{p(i)} = {\bf X}_J(i){\bf X}_L(i)\f$</li>
  	    <li> \f$\text{if } p(i)\neq 0 \text{ then }\f$
	       <ol>
	         <li> \f${^i{\bf X}_0} = {^i{\bf X}_{p(i)}} {^{p(i)}{\bf X}_0} \f$</li>
	       </ol>
	    </li>
	    <li> \f$ \text{end}\f$</li>
	    <li> \f$ {\bf \Phi}_i = pcalc(jtype(i),{\bf q}_i)\f$</li>
	    <li> \f$ {\mathring{\bf \Phi}}c_i = pdcalc(jtype(i),{\bf q}_i)\f$</li>
	    <li> \f$ {\bf v}_i = {^i{\bf X}_{p(i)}}{\bf v}_{p(i)} + \Phi_i\dot{\bf q}_i\f$</li>
	    <li> \f$ {\bf \zeta}_i = {\mathring{\bf \Phi}}c_i \dot{\bf q}_i + {\bf v}_i \times \Phi_i\dot{\bf q}_i \f$</li> 
	    <li> \f$ {\bf a}_i = {^i{\bf X}_{p(i)}} {\bf a}_{p(i)} + \Phi_i \ddot{\bf q}_i + {\bf \zeta}_i \f$</li>
	    <li> \f$ {\bf f}_i = {\bf I}_i {\bf a}_i + {\bf v}_i \times {\bf I}_i {\bf v}_i \f$</li>
	  </ol> 
	</li>
	<li> \f$ for \; i=N_b\; to \; 1 \; do \f$
	  <ol> 
	    <li>\f$ {\bf \tau}_i = \Phi_i^T {\bf f}_i \f$ </li>
            <li> \f$ \text{if } p(i)\neq 0 \text{ then } \f$
               <ol>
	         <li> \f${\bf f}_p(i) = {\bf f}_{p(i)} + {^i}{\bf X}_{p(i)}^T} \f$ </li>
	       </ol>
	    </li>
	    <li> \f$ \text{end} \f$</li>
          </ol>
	</li>

     </ol>

	
      
*/
