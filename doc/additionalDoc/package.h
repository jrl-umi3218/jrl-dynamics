/** \mainpage

\section intro Introduction

This package implements the abstract interface abstractRobotDynamics.
More specifically the algorithm used in this library is
the well known Newton-Euler which provides the inverse dynamics of a robot.
Given a robot with a state vector 
\f$ {\bf x} = [{\bf r} \; {\bf q} \; \dot{\bf r} \; \dot{\bf q} \; \ddot{\bf r} \; \ddot{\bf q}]\f$,
where  
\f$ {\bf r} \; \dot{\bf r} \; \ddot{\bf r}\f$ are the free flyer position, velocity and acceleration;
\f$ {\bf q} \; \dot{\bf q} \; \ddot{\bf q}\f$ are the articular position  velocity and acceleration,
the algorithm computes the position, velocities, acceleration, force and torques
for each bodies of the robot. <br>

To make sure that the code developped with this library is compatible with 
other implementation of the abstract interface, the specific implementation details are hidden to the user.

The robot model used in this library is a kinematic tree which can be build
using the abstract interface or by parsing a VRML file following the  
<a href="http://www.openrtp.jp/openhrp3/">OpenHRP</a> format.
  
*/
