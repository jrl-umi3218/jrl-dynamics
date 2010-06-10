/** \page Installing Installing

\section robotpkg
In order to have a coherent installation it is strongly recommended to install 
dynamicsJRLJapan using the tool named robotpkg. 
For instructions on robotpkg please look 
<a href="http://homepages.laas.fr/mallet/robotpkg">here</a>.

\section Dependencies
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
  </ul>
</ul>

\section Compiling

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

*/
