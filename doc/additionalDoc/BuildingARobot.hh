/*
 * Copyright 2010, 
 *
 * Olivier Stasse,
 * 
 *
 * JRL/LAAS, CNRS/AIST
 *
 * This file is part of dynamicsJRLJapan.
 * dynamicsJRLJapan is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * dynamicsJRLJapan is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Lesser Public License for more details.
 * You should have received a copy of the GNU Lesser General Public License
 * along with dynamicsJRLJapan.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  Research carried out within the scope of the Associated
 *  International Laboratory: Joint Japanese-French Robotics
 *  Laboratory (JRL)
 *
 */
/** \page pbuildingarobot Building a robot
    
    \section secobjectfactory Object Factory

    A robot model is handled using the class dynamicRobot,
    and it is build using the following components:
    <ul>
      <li> Bodies</li>
      <li> Joints</li>
    </ul>
    
    A humanoid robot (humanoidDynamicRobot) is a particular robot which has
    in addition the following components:
    <ul>
      <li> Hand </li>
      <li> Foot </li>
    </ul>

    All those components are created using an object factory.
    The class ObjectFactory provides such a factory.<br>

    \subsection subsecbhr Building a humanoid robot
    Creating an empty humanoid robot is therefore very simple:

\code
dynamicsJRLJapan::ObjectFactory robotDynamicsObjectConstructor; 
CjrlHumanoidDynamicRobot * aHDR = robotDynamicsObjectConstructor.createHumanoidDynamicRobot();
\endcode

    \subsection subsecbj Building a revolute joint
The same way a joint can be created by specifying the pose of the joint
when the robot is at state \f$ {\bf x} = [{\bf r} \; {\bf q} \; \dot{\bf r} \; \dot{\bf q} \; \ddot{\bf r} \; \ddot{\bf q}]=
[{\bf 0} \; {\bf 0} \; {\bf 0} \; {\bf 0} \; {\bf 0} \; {\bf 0}]\f$.
Therefore if the first joint is at the origin of the reference frame, the
following code snippet can be used:
\code
matrix4d pose;
MAL_S4x4_MATRIX_SET_IDENTITY(pose);

// Create Joint 1
CjrlJoint* j1=0;
j1 = anObjectFactory->createJointRotation(pose);
\endcode

IMPORTANT: It is assumed that the revolute joint axis is aligned with the \f$x\f$-axis
of its own frame. This normalization is very similar to the one of Davit-Hartenberg,
and is compatible with KineoWorks. The position of the joint and the kinematic
tree specified below allow the library to derive all the needed parameters automatically.

\subsection subsecbj Building a body
The same way a body can be build 
\code
// Create Link 1
CjrlBody* l1=0;
l1= anObjectFactory->createBody();
\endcode

The body mass is specified with:
\code
l1->mass(m_SetOfParameters.m[0]);
\endcode

The inertia matrix can be specified using:
\code
matrix3d I;
MAL_S3x3_MATRIX_SET_IDENTITY(I);
l1->inertiaMatrix(I);
\endcode

Finally the center of mass for the body l1 is written:
\code
vector3d lc ;
lc(0) = 0.0;
lc(1) = 0.0;
lc(2) = 0.25;
l1->localCenterOfMass(lc);
\endcode

A body is related with the joint which 
is the parent of all the other joints on the body.
It is specified with:
\code
// Relate joint1 and link1
j1->setLinkedBody(*l1);
\endcode

\section secspeckintree Specifying the kinematic tree

   The library handles robots as kinematic trees.
   Therefore each joint has a father or is the root of the robot kinematic tree. 

\subsection subsecspecroot Specifying the root

To specify the root of the kinematic for the humanoid robot created above:
\code
// Joint 1 is root of the tree.
aHDR->rootJoint(*j1);
\endcode

\subsection subsecspecfather Adding a child in the kinematic chain
A joint is set as the son of another joint with:
\code
CjrlJoint* j2=0;
j2 = anObjectFactory->createJointRotation(pose);
j1->addChildJoint(*j2);
\endcode

\subsection subsecinitkt Initialization
In order to compute quantities, it is important to initialize
the internal structure of the library by calling the following method:
\code
aHDR->initialize();
\endcode    

 */

