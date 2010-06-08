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

\section Installation

\subsection robotpkg
In order to have a coherent installation it is strongly recommended to install 
dynamicsJRLJapan using the tool named robotpkg. 
For instructions in how to install it please look
<a href="http://homepages.laas.fr/mallet/robotpkg">here</a>.

\subsection Dependencies
This package depends upon the following packages:
<ul>
  <li> System Packages: </li>
  <ul>
    <li> boost (last tested on 1.41)</li>
    <li> pkgconfig </li>
    <li> cmake </li>
  </ul>
  <li> JRL packages: </li>
  <ul>
   <li> Matrix Abstract Layer</li>
   <li> abstractRobotDynamics</li>
  </ul
</ul>

\subsection Compiling

<ol>
<li> Uncompress the source package<br>
tar zxvf dynamicsJRLJapan.tgz </li>
<li> Create a build directory and go inside it: <br>
mkdir build<br></li>
<li> Setup the installation with :<br>
cmake -DCMAKE_INSTALL_PREFIX=/your/prefix -DCMAKE_BUILD_TYPE=RELEASE ..<br></li>
<li> Compile with: <br>
make <br></li>
<li> The installation is realized with: <br>
make install <br></li>
</ol>


\section author Authors
The package has been strongly modified since its inception 
but the people mentionned here participated in a significant way:
<ul>
<li> Jean-Remy Chardonnet </li>
<li> Adrien Escande </li>
<li> Fumio Kanehiro </li>
<li> Oussama Kanoun </li>
<li> Francois Keith </li>
<li> Florent Lamiraux </li>
<li> Ramzi Sellaouti </li>
<li> Olivier Stasse </li>
</ul>

*/

