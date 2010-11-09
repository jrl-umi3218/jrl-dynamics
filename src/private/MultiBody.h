/*
 * Copyright 2010,
 *
 * Adrien Escande,
 * Oussama Kanoun
 * Francois Keith
 * Abderrahmane Kheddar,
 * Florent Lamiraux
 * Olivier Stasse,
 * Ramzi Sellouati
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

/*! Multibody object used to compute :
   - Center Of Mass,
   - Zero Momentum Point.

   This version has been modified to get rid of all the OpenGL code.

   OS: Updated the names of the contributors, the documentation
   and added a sample file for WalkPlugin
   OS (21/12/2006): removed any reference to non-homogeneous matrix
   library.
*/

#ifndef MULTIBODY_H
#define MULTIBODY_H

#include "JointPrivate.h"
#include "Body.h"
#include <vector>
#include <algorithm>	//pour utiliser la fonction find
#include <iostream>
#include <string>

using namespace std;

namespace dynamicsJRLJapan
{


  /** @ingroup forwardynamics
       Internallink structure:
      Define a link between two bodys:
      Supported types:
      - Free joint (6 DoFs) (type = FREE_JOINT)
      - Fixed joint (0 DoF) (type = FIX_JOINT)
      - Prismatic joint (1 DoF) (type = PRISMATIC_JOINT)
      - Revolute joint (1 DoF) (type = REVOLUTE_JOINT)

      The sequence for the transformation is
      - translation (translationStatique
      - rotation (angleRotationStatique)
      - the list of transformation (listeTransformation).
  */
  struct internalLink {
    int label;
    JointPrivate * aJoint;
    int indexCorps1;
    int indexCorps2;
  };

  /** @ingroup forwardynamics

  Binding structure:
  - \a corps is the body to which this structure belongs. This should be equal
  to one of the field \a indexCorps1 or \a indexCorps of the internalLink's instance indicated
  by the field \a liaison.
  - \a liaison is the corresponding link.

  The equality operator is used with the find function of \<algorithm\>.
  */
  struct matching {
    int body;
    int link;
  };

  bool operator==(const matching a1, const matching a2);

  /** @ingroup forwardynamics

      This class implements a non-oriented graph for which the nodes are
      bodies (Body) and the edges are joints (internalLink).

      The description is:
      There is a list of nodes (listeBodies) and a list of edges (listeInternalLinks).
      The edges starting from an i-th node of listeBodies are stored in vector
      links[i], and store an instance of matching for which the body
      Body is the second node of the edge.

  */
  class MultiBody
  {
  protected:

    /*!  Robot's mass. */
    double m_mass;
    /*!  CoM of the robot */
    vector3d positionCoMPondere;			//CoM*mass
    /*!  List of links. */
    std::vector<internalLink> listInternalLinks;
    /*!  List Of Bodies. */
    std::vector<Body *> listBodies;
    /*!  Links with other bodies. */
    std::vector<std::vector<matching> > links;

  public:

    /*!  Constructor */
    MultiBody(void);
    /*!  Destructor */
    virtual ~MultiBody(void);

    /*!  Adds a body. */
    void addBody(Body &b);

    /*!  Adds a link. */
    void addLink(Body &corps1, Body &corps2, internalLink & l);

    /*! Return last body. */
    Body * lastBody();

    /*!  Remove a link between body corps1 and body corps. */
    void deleteLinkBetween(Body &corps1, Body &corps2);

    /*!  Remove a link by index. */
    void removeLink(int index);

    /*!  Remove a link by label. */
    void removeLinkLabel(int label);

    /*!  Remove the body b. */
    void removeBody(Body &b);

    /*!  Remove a body by index. */
    void removeBody(int index);

    /*!  Remove body by label. */
    void removeBodyLabel(int label);

    /*!  Returns the CoM position. */
    vector3d getPositionCoM(void);


    /*!  Display bodies */
    void displayBodies(void);

    /*!  Display links */
    void displayLinks(void);

    /*!  Display everything.  */
    void display(void);

    /*!  Returns the number of links. */
    int NbOfLinks() const;

    /*!  Returns the number of Joints. */
    int NbOfJoints() const;

    /*! Operator = */
    MultiBody & operator=(const MultiBody &rhs);

  };

  // Create a matrix from an axis and a rotation around
  // this axis.
  void AxisAngle2Matrix(const vector3d &AnAxis, double aQuantity, matrix3d &R);

}

#endif
