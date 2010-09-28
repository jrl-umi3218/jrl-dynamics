// Backward iteration to compute dynamics.
// Input Variables:
// ================
// i : number of the body.
// lm : vector of masses.
// lI : inertia matrices.
// llc : vector of local center of mass (local reference frame).
// lw : angular velocities (local reference frame).
// ldv : linear velocities (local reference frame).
// ldw : angular velocities (local reference frame).
// lp : position of the joint in their parent reference frame.
// lR : rotation of the joint in their parent reference frame.
// lRA : rotation of the joint in the World/Absolute reference frame.
// mf : Forces in local reference frame.
// mt : Torques in local reference frame.
// Output Variables:
// =================
// lf : forces.
// lt : torques.
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

function [lf,lt]=backwardit(i,lm, lI,llc ,lw,ldv,ldw,lp,lR,lRA,mf,mt)

// Compute acceleration of the center of mass.
ac = ldv + skew(ldw)*llc + skew(lw)*skew(lw)*llc

// Compute part of the force related to the body itself
lf = (ac - lRA([(i-1)*3+1:i*3],:)'*[0.0; 0.0; -9.81;]) *lm;

// Compute part of the torque related to the body itself 
lt = lI * ldw + skew(lw) * lI * lw + skew(llc) * lf;

if i < size(f,'c') then
  // Add force from the son.
  lf = lR([i*3+1:(i+1)*3],:)' * mf(:,i+1) + lf;
  // Add torque from the son.
  lt = lR([i*3+1:(i+1)*3],:)' * mt(:,i+1) + skew(lp(i+1,:)) * lR([i*3+1:(i+1)*3],:)' * mf(:,i+1) + lt;
end,

endfunction
