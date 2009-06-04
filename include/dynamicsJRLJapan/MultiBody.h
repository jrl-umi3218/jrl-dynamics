/* Multibody object used to compute :
   - Center Of Mass,
   - Zero Momentum Point.

   This version has been modified to get rid of all the OpenGL code.

   OS: Updated the names of the contributors, the documentation
   and added a sample file for WalkPlugin
   OS (21/12/2006): removed any reference to non-homogeneous matrix
   library.


   Copyright (c) 2005-2006, 
   @author Adrien Escande, Francois Keith, Abderrahmane Kheddar, Olivier Stasse, Ramzi Sellouati
   
   JRL-Japan, CNRS/AIST

   All rights reserved.

   Please refers to file License.txt for details on the license.
*/

#ifndef MULTIBODY_H
#define MULTIBODY_H

#define INTERFACE
#include "dynamics-config.h"
#include "dynamicsJRLJapan/Joint.h"
#include "dynamicsJRLJapan/Body.h"
#include "dynamicsJRLJapan/fileReader.h"
#include <vector>
#include <algorithm>	//pour utiliser la fonction find
#include <iostream>
#include <string>

using namespace std;


//static int cptLiaison= 0;	//pour labeliser les liaisons


namespace dynamicsJRLJapan
{
    
    
  static const int PROFONDEUR_MAX=30;


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
    Joint * aJoint;
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
  struct appariement {
    int corps;
    int liaison;
  };

  bool operator==(const appariement a1, const appariement a2);

  /** @ingroup forwardynamics
       
      This class implements a non-oriented graph for which the nodes are 
      bodies (Body) and the edges are joints (internalLink).
      
      The description is:
      There is a list of nodes (listeCorps) and a list of edges (listeLiaisons).
      The edges starting from an i-th node of listeCorps are stored in vector
      liaisons[i], and store an instance of appariement for which the body 
      Body is the second node of the edge.

      
      La suppresion d'un noeud n'entraine pas la suppression effective du noeud et
      des arcs correspondants dans les vecteurs concernes, mais seulement le
      marquage des instances en questions comme "supprimees" par la mise a -1 de 
      leur label.
      
      Les methodes necessitant un parcours du graphe se base sur l'hypothese qu'il
      ne contienne pas de cycle.   
  */
  class DYN_JRL_JAPAN_EXPORT MultiBody
  {
  protected:
    
    /*!  Robot's mass. */
    double masse;
    /*!  CoM of the robot */
    vector3d positionCoMPondere;			//CoM*masse
    /*!  List of links. */
    std::vector<internalLink> listeLiaisons;
    /*!  List Of Bodies. */
    std::vector<Body *> listeCorps;
    /*!  Links with other bodies. */
    std::vector<std::vector<appariement> > liaisons;
    /*!  Counter for links. */
    int cptLiaison;

  public:

    /*!  Constructor */
    MultiBody(void);
    /*!  Destructor */
    virtual ~MultiBody(void);
    
    /*! Returns the masse. */
    double getMasse();

    /*!  Adds a body. */
    void ajouterCorps(Body &b);

    /*!  Adds a link. */
    void ajouterLiaison(Body &corps1, Body &corps2, internalLink & l);

    /*!  Adds a fixed link. */
    void ajouterLiaisonFixe(Body &corps1, Body &corps2, 
			    vector3d translationStat, 
			    vector3d axeRotationStat, 
			    double angleRotationStat = 0);
    /*!  Adds a link with rotation. */
    void ajouterLiaisonRotation(Body &corps1, Body &corps2, 
				vector3d axe , 
				vector3d translationStat, 
				vector3d axeRotationStat, 
				double angleRotationStat = 0);
    /*! Return last body. */
    Body * dernierCorps();

    /*!  Remove a link between body corps1 and body corps. */
    void supprimerLiaisonEntre(Body &corps1, Body &corps2);

    /*!  Remove a link by index. */
    void supprimerLiaison(int index);

    /*!  Remove a link by label. */
    void supprimerLiaisonLabel(int label);

    /*!  Remove the body b. */
    void supprimerCorps(Body &b);

    /*!  Remove a body by index. */
    void supprimerCorps(int index);

    /*!  Remove body by label. */
    void supprimerCorpsLabel(int label);

    /*!  Inverse the link i. */
    void inverserLiaison(int i);

    /*!  Returns the CoM position. */ 
    vector3d getPositionCoM(void);

    //Construction a partir d'un fichier VRML
    virtual void parserVRML(string path, string nom, const char* option);
  
    /*!  Display bodies */
    void afficherCorps(void);
    /*!  Display links */
    void afficherLiaisons(void);
    /*!  Display everything.  */
    void afficher(void);

    /*!  Returns the number of links. */
    int NbOfLinks() const;

    /*!  Returns the number of Joints. */
    int NbOfJoints() const;
    
  };

  // Create a matrix from an axis and a rotation around
  // this axis.
  void AxeAngle2Matrix(const vector3d &AnAxis, double aQuantity, matrix3d &R);
  
};

#endif
