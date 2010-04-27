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
// (c) Olivier STASSE, CNRS/AIST, JRL

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
