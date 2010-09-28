// This script implements the Inverse Dynamics
// as described by Featherstone in p.52 of the
// Handbook of Robotics.
//
// Copyright 2010, 
//
// Olivier Stasse,
//
// JRL/LAAS, CNRS/AIST
//
// This file is part of dynamicsJRLJapan.
// dynamicsJRLJapan is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// dynamicsJRLJapan is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Lesser Public License for more details.
// You should have received a copy of the GNU Lesser General Public License
// along with dynamicsJRLJapan.  If not, see <http://www.gnu.org/licenses/>.
//
//  Research carried out within the scope of the Associated
//  International Laboratory: Joint Japanese-French Robotics
//  Laboratory (JRL)
//

mode(0)

// Create rotation 
function [rotM] = rotx(x)
rotM=[ [ 1 0 0 ]; ...
       [ 0 cos(x) -sin(x)]; ...
       [ 0 sin(x) cos(x) ]; ];
endfunction

// Skew operator
function sk=skew(x)
sk=[[0 -x(3) x(2)];[x(3) 0 -x(1)];[-x(2) x(1) 0];];
endfunction

// Load model information
exec("legs.sci");

// Load functions
exec("Fforward.sci");
exec("Fbackward.sci");
// end Load functions

// Initialize forces and torques.
f=zeros(3,6);
t=zeros(3,6);

// Initialize robot state, velocities and accelerations.
q   = [ 0.0; 0.0; 0.0; 0.0; 0.0; 0.0;];
dq  = [ 1.0; 1.0; 1.0; 1.0; 0.0; 0.0;];
ddq = [ 0.0; 2.0; 0.0; 0.0; 2.0; 0.0;];

// Initialize angular velocities (local frame)
mw = zeros(3,6);
// Initialize linear velocities (local frame)
mdv = zeros(3,6);
// Initialize angular acceleration (local frame)
mdw = zeros(3,6);
// Initialize rotations (local frame)
mR = zeros(6*3,3);
// Initialize rotations (absolute/world frame)
mRA = zeros(6*3,3);

nextstep ="Forward recursion"

// Loop for forward recursion
for i=1:6, 
  s1 =(i-1)*3+1;
  s2 = i*3;
  [lRi,lRA, lwi,ldwi,ldvi] = ...
   forwardit( i,...
  	      q, dq, ddq, ...
	      mw,mdv,mdw, ...
	      mR, mRA, RS, p);
   mR([s1:s2],:) = lRi;
   mRA([s1:s2],:) = lRA;
   mw(:,i)  =  lwi;
   mdw(:,i) = ldwi;
   mdv(:,i) = ldvi;

end

nextstep ="Backward recursion"
// Loop for backward recursion
for i=6:-1:1, 

  [f(:,i),t(:,i)] = backwardit( i,m(i), ...
	       	    I([(i-1)*3+1:i*3],:), ...
		    lc(i,:)', ...
		    mw(:,i),mdv(:,i), ...
		    mdw(:,i),p, ...
		    mR,...
		    mRA,...
		    f,t);
end

