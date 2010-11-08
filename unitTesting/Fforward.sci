// Forward iteration to compute dynamics.
// Input Variables:
// ================
// i : number of the body.
// mq : articular values of the robot.
// mdq: velocities of the robot joints.
// mddq: acceleration of the robot joints.
// mw : angular velocities (local reference frame).
// mdv : linear velocities (local reference frame).
// mdw : angular velocities (local reference frame).
// mR : rotation of the joint in their parent reference frame
//      this include the static rotation and the joint rotation.
// mRA : rotation of the joint in the World/Absolute reference frame.
// mRS : static rotation of the joint in their parent reference frame
// mp : position of the joint in their parent reference frame.
// Output variables:
// =================
// lR : Rotation of joint i including static and joint rotation in local reference frame.
// lRA : Rotation of joint i in the world/absolute reference frame.
// lw: angular velocity in local reference frame.
// ldw: angular acceleration in local reference frame.
// ldv: linear acceleration in local reference frame.
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

function [lR,lRA,lw,ldw,ldv] = forwardit(i,mq,mdq,mddq,mw,mdv,mdw,mR,mRA, mRS,mp)

// Relative rotation
lR = zeros(3,3);
// Compute the rotation matrix  iRp(i)
lR([1:3],:) =  mRS([(i-1)*3+1:i*3],:) * rotx(mq(i));

// Absolute rotation
lRA =zeros(3,3);
if i > 1  then
  lRA =  mRA([(i-2)*3+1:(i-1)*3],:) * lR;
else
  lRA = lR;
end,

lR=lR';


// Compute the angular velocity.
lw = zeros(3,1);
lw = [ 1 0 0]'* mdq(i);
// Term from parent.
if i > 1  then
  lw = lw + lR * mw(:,i-1);
end,

lw

// Compute the angular acceleration
ldw = zeros(3,1);
ldw =  [ 1 0 0]'* mddq(i);
// Term from parent.
if i > 1
  ldw = ldw + lR * mdw(:,i-1) ...
        + skew ( lR * mw(:,i-1)) * ...
	[ 1 0 0]'*mdq(i);
end,

ldw

// Compute the linear acceleration
ldv = zeros(3,1);
// Term from parent.
if i > 1
  ldv =lR * ( mdv(:,i-1) + skew(mdw(:,i-1))* mp(i,:)' ...
	     + skew(mw(:,i-1))*skew(mw(:,i-1))* mp(i,:)') ;
end,

endfunction
