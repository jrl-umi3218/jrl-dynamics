/*
 * Copyright 2009, 2010, 
 *
 * Francois Keith
 * Florent Lamiraux
 * Olivier Stasse
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


/*! System includes */
#include <iostream>
#include <sstream>
#include <fstream>
#include <string.h>

#include "Debug.h"

/*! Local library includes. */
#include "MatrixAbstractLayer/MatrixAbstractLayer.h"
#include "dynamicsJRLJapan/DynamicBody.h"
#include "DynMultiBodyPrivate.h"
#include "robotDynamics/jrlBody.h"

#include "fileReader.h"

using namespace dynamicsJRLJapan;

void DynMultiBodyPrivate::SpecifyTheRootLabel(int ID)
{
  labelTheRoot = ID;
  m_listOfBodies[ID]->setLabelMother(-1);

  // Right now it is assume that the first body is the world,
  // and that this body is related to another body.
  // Thus links[ID].size() should be equal to one.


  if (links[ID].size()!=1)
    {
      cout << "Wrong assumption concerning the initial body: " << links[ID].size()<< endl;
      return;
    }
  int ld = links[ID][0].link;
  m_RootOfTheJointsTree = listInternalLinks[ld].aJoint;
  m_RootOfTheJointsTree->setLinkedBody(*m_listOfBodies[ID]);
  ODEBUG(" m_RootOfTheJointsTree->m_globalConfiguration"<<
	  m_RootOfTheJointsTree->initialPosition());
  //specify the type of the root joint

  // Start the vector of joints.
  m_JointVector.clear();
  m_JointVector.insert(m_JointVector.end(),m_RootOfTheJointsTree);
  int lVRMLID = m_RootOfTheJointsTree->getIDinActuated();
  if (m_NbOfVRMLIDs < lVRMLID)
    m_NbOfVRMLIDs = lVRMLID;

  // Find out the next body to be examine.
  ReLabelling(ID,ld);

  // Once finished we initialize the child and the sister.
  for(unsigned int i=0;i<m_listOfBodies.size();i++)
    {
      int lMother,lElderSister;
      if ((lMother=m_listOfBodies[i]->getLabelMother()) != -1)
        {

	  if ((lElderSister=m_listOfBodies[lMother]->child) == -1)
            {
	      // Mother, I am your daughter !
	      m_listOfBodies[lMother]->child = i;
            }
	  else
            {
	      // I have an elder sister !
	      while (m_listOfBodies[lElderSister]->sister != -1)
		// I have another elder sister !
		lElderSister = m_listOfBodies[lElderSister]->sister;
	      // I am your younger sister !
	      m_listOfBodies[lElderSister]->sister = i;
            }
        }
    }

  for (unsigned int i = 0; i< m_listOfBodies.size();i++)
    m_listOfBodies[i]->massCoef(m_listOfBodies[i]->mass()/mass());
}

void DynMultiBodyPrivate::UpdateBodyParametersFromJoint(int BodyID, int JointID, int LiaisonForFatherJoint)
// cID : corps identifier
// lD : liaison destination
{

  ODEBUG( "Update body :" << BodyID << " from Joint " << JointID);
  // Update the rotation axis.
  m_listOfBodies[BodyID]->a =  listInternalLinks[JointID].aJoint->axis();
  ODEBUG(" axis: " << m_listOfBodies[BodyID]->a);
  // Update the translation vector
  listInternalLinks[JointID].aJoint->getStaticTranslation(m_listOfBodies[BodyID]->b);
  ODEBUG(" JointID: " << JointID << "BodyID: " << BodyID << ", static translation: " << m_listOfBodies[BodyID]->b);
  // Update the rotation matrix
  listInternalLinks[JointID].aJoint->getStaticRotation(m_listOfBodies[BodyID]->R_static);
  ODEBUG(" Rotation matrix: " << endl << m_listOfBodies[BodyID]->R_static);
  listInternalLinks[JointID].aJoint->setLinkedBody(*m_listOfBodies[BodyID]);
  m_listOfBodies[BodyID]->joint( listInternalLinks[JointID].aJoint);

}

void DynMultiBodyPrivate::ReLabelling(int currentBody, int sourceLink)
{
  // This one has been nicely clean-up
  for (unsigned int i=0; i<links[currentBody].size(); i++)
    {
      int destinationLink = links[currentBody][i].link;
      if ((destinationLink == sourceLink) &&
	  (currentBody!=labelTheRoot))
	continue;

      int corps1 = listInternalLinks[destinationLink].indexCorps1;
      int corps2 = listInternalLinks[destinationLink].indexCorps2;
      int corpsMother,corpsSon;

      if ((currentBody == corps1) && (currentBody != corps2))
        {
	  corpsSon = corps2;
	  corpsMother = corps1;
        }
      else
        {
	  corpsSon = corps1;
	  corpsMother = corps2;
        }
      m_listOfBodies[corpsSon]->setLabelMother(corpsMother);

      if(listInternalLinks[destinationLink].aJoint->getIDinActuated()!=-1)
        {


	  // Update the connections between the Joints.
	  int lIDinActuated = listInternalLinks[destinationLink].aJoint->getIDinActuated();
	  if (lIDinActuated!=-1)
            {
	      ConvertIDInActuatedToBodyID[lIDinActuated] = corpsSon;
            }


	  listInternalLinks[sourceLink].aJoint->addChildJoint(*listInternalLinks[destinationLink].aJoint);

	  listInternalLinks[destinationLink].aJoint->SetFatherJoint(listInternalLinks[sourceLink].aJoint);


	  // Update the vector of joints.
	  ODEBUG("Inside Relabelling :" << listInternalLinks[destinationLink].aJoint->getName().c_str());
	  ODEBUG("JointRankFromName :" << JointRankFromName(listInternalLinks[destinationLink].aJoint));
	  if (JointRankFromName(listInternalLinks[destinationLink].aJoint)!=-1)
	    m_JointVector.insert(m_JointVector.end(),listInternalLinks[destinationLink].aJoint);

	  int lVRMLID = listInternalLinks[destinationLink].aJoint->getIDinActuated();
	  if (m_NbOfVRMLIDs < lVRMLID)
	    m_NbOfVRMLIDs = lVRMLID;


        }

      // TODO : It is important to do the relabelling after the
      // TODO : recursive call to the relabelling as set father build
      // TODO : up the JointFromRootToThis vector. Should be fixed at the Joint object level.
      ReLabelling(corpsSon, destinationLink);
      UpdateBodyParametersFromJoint(corpsSon,destinationLink,sourceLink);

    }

}

void DynMultiBodyPrivate::CreatesTreeStructure(const char * option)
{
  m_listOfBodies.resize(listBodies.size());
  DynamicBodyPrivate *dbody;
  for(unsigned int i=0;i<listBodies.size();i++)
    {
      dbody = dynamic_cast<DynamicBodyPrivate *>(listBodies[i]);
      if (!dbody)
        {
	  dbody = new DynamicBodyPrivate();
	  *dbody = *listBodies[i];
        }
      m_listOfBodies[i] = dbody;
    }

  if (option!=0)
    {
      ReadSpecificities(option);
    }

  ConvertIDInActuatedToBodyID.resize(m_listOfBodies.size());
  SpecifyTheRootLabel(0);
  ComputeNumberOfJoints();
  BuildStateVectorToJointAndDOFs();
  BuildLinkFromActuatedIDs();
  UpdateTheSizeOfJointsJacobian();

  for(unsigned int i=0;i<m_JointVector.size();i++)
    {
      JointPrivate *aJP = dynamic_cast<JointPrivate *>(m_JointVector[i]);
      if (aJP!=0)
	aJP->computeLocalAndGlobalPose();
    }

  MAL_VECTOR_RESIZE(m_Configuration,m_NbDofs);	MAL_VECTOR_FILL(m_Configuration, 0);
  MAL_VECTOR_RESIZE(m_Velocity,m_NbDofs);		MAL_VECTOR_FILL(m_Velocity, 0);
  MAL_VECTOR_RESIZE(m_Acceleration,m_NbDofs);	MAL_VECTOR_FILL(m_Acceleration, 0);

  MAL_MATRIX_RESIZE(m_Forces ,m_NbDofs,3);		MAL_MATRIX_FILL(m_Forces, 0);
  MAL_MATRIX_RESIZE(m_Torques,m_NbDofs,3);		MAL_MATRIX_FILL(m_Torques, 0);

  MAL_VECTOR_RESIZE(m_pastConfiguration,m_NbDofs);	MAL_VECTOR_FILL(m_pastConfiguration, 0);
  MAL_VECTOR_RESIZE(m_pastVelocity,m_NbDofs);	MAL_VECTOR_FILL(m_pastVelocity, 0);
}

void DynMultiBodyPrivate::InitializeFromJointsTree()
{
  /* The goal of this method is to recreate the undirected
     graph, to restart the sequence of initialization. */
  vector<Body *> vectorOfBodies;
  int Depth=0;
  int NbOfBodies=0;
  internalLink CurrentLink ;

  /* Initialize the reading. */

  // Default initialization to 30 bodies for one branch.
  vectorOfBodies.resize(30);
  vectorOfBodies[0] = new DynamicBodyPrivate();
  vectorOfBodies[0]->setLabel(NbOfBodies++);
  addBody(*vectorOfBodies[0]);
  Depth++;

  // Go through the Joints tree.
  JointPrivate *CurrentJoint = m_RootOfTheJointsTree;
  JointPrivate *NextCurrentJoint=0,*FatherJoint;
  double mi[9]={ 1.0,1.0,1.0, 1.0,1.0,1.0, 1.0,1.0,1.0};

  CurrentLink.label = 0;
  CurrentLink.aJoint = m_RootOfTheJointsTree;
  CurrentLink.indexCorps1 = 0;
  CurrentLink.indexCorps2 = 0;
  int lIDinActuated=-1;
  unsigned int lRank=0;

  while(CurrentJoint!=0)
    {
      // Deal with the current joint.

      // Update the joint value of the current link.
      CurrentLink.aJoint = CurrentJoint;


      // Update the joint ID value in case there is none.
      if (CurrentLink.aJoint->getIDinActuated()==-1)
	CurrentLink.aJoint->setIDinActuated(lIDinActuated);

      lIDinActuated++;

      if (m_FileLinkJointRank.size()==0)
        // Create a relation between the name and the rank.
        {
	  NameAndRank_t aNameAndRank;

	  if (CurrentLink.aJoint->getName() == "")
            {
	      char buf[64];
	      if (CurrentLink.aJoint->type() == JointPrivate::FIX_JOINT)
                {
		  sprintf(buf, "FIXED_%02d", lRank);
                }
	      else
                {
		  sprintf(buf, "JOINT_%02d", lRank);
                }
	      string name(buf);
	      CurrentLink.aJoint->setName(name);
            }
	  strcpy(aNameAndRank.LinkName,
		 (char *)CurrentLink.aJoint->getName().c_str());

	  aNameAndRank.RankInConfiguration = lRank;
	  m_LinksBetweenJointNamesAndRank.insert(m_LinksBetweenJointNamesAndRank.end(),
						 aNameAndRank);

	  lRank+= CurrentLink.aJoint->numberDof();
        }


      // Take care of the body.
      // extend the size of vectorOfBodies if necessary.
      if (Depth>(int)vectorOfBodies.size())
        {
	  Body *aBody=NULL;
	  vectorOfBodies.push_back(aBody);
        }

      /*
        If a body has already been attached to the joint,
        keep inertial information
      */
      DynamicBodyPrivate* lCurrentBody = NULL;
      CjrlBody* jrlBody = CurrentJoint->linkedBody();
      if (jrlBody != NULL) {
	lCurrentBody = dynamic_cast<DynamicBodyPrivate*>(jrlBody);

	if (lCurrentBody) {
	  lCurrentBody->localCenterOfMass(jrlBody->localCenterOfMass());
	} 
	else if (DynamicBody* dBody = dynamic_cast<DynamicBody*>(jrlBody)) {
	  lCurrentBody = dynamic_cast<DynamicBodyPrivate*>(dBody->m_privateObj.get());
	} else {
	  std::cerr <<
	    "dynamicsJRLJapan: body is not of ab expected type."
		    << std::endl;
	  throw(0);
	}
      }
      else
        {
	  lCurrentBody = new DynamicBodyPrivate();
	  lCurrentBody->setInertie(mi);
	  lCurrentBody->setMass(1.0);// 1 kg per default.
	  vector3d cm;cm[0] = cm[1] =cm[2]=0.0;
	  lCurrentBody->localCenterOfMass(cm);
        }
      vectorOfBodies[Depth] = lCurrentBody;
      lCurrentBody->setLabel(NbOfBodies++);
      string lCurrentBodyName=CurrentJoint->getName();
      lCurrentBodyName = "BODY_" + lCurrentBodyName;
      lCurrentBody->setName((char *)lCurrentBodyName.c_str());
      lCurrentBody->setLabelMother(vectorOfBodies[Depth-1]->getLabel());


      addBody(*lCurrentBody);
      addLink(*vectorOfBodies[Depth-1],
	      *lCurrentBody,
	      CurrentLink);

      // Find the next one.
      // 1. A children.
      NextCurrentJoint = (dynamicsJRLJapan::JointPrivate *)CurrentJoint->childJoint(0);
      Depth++;

      // No child.
      if (NextCurrentJoint==0)
        {
	  // If a father exist.
	  FatherJoint = (dynamicsJRLJapan::JointPrivate *)CurrentJoint->parentJoint();
	  Depth--;

	  while( (FatherJoint!=0) &&
		 (NextCurrentJoint==0))
            {
	      // Find the location of the current node
	      // in the father tree.
	      int NbOfChildren= FatherJoint->countChildJoints();
	      int CurrentJointPosition=-1;
	      for(int li=0;li<NbOfChildren;li++)
		if (FatherJoint->childJoint(li)==CurrentJoint)
		  {
		    CurrentJointPosition = li;
		    break;
		  }

	      // If a sibling has not been explored
	      if(CurrentJointPosition<NbOfChildren-1)
                {
		  // take it !
		  NextCurrentJoint = (dynamicsJRLJapan::JointPrivate *)FatherJoint->
		    childJoint(CurrentJointPosition+1);
                }
	      else
                {
		  // otherwise move upward.
		  CurrentJoint =FatherJoint;
		  FatherJoint=(dynamicsJRLJapan::JointPrivate *)FatherJoint->parentJoint();
		  Depth--;
                }
            }
	  // If finally FatherJoint==0 then NextCurrentJoint too is equal to 0.

        }

      CurrentJoint=NextCurrentJoint;

    }

  /* Initialize the data structures needed for the Newton-Euler algorithm. */
  if (m_FileLinkJointRank.size()==0)
    CreatesTreeStructure(0);
  else
    CreatesTreeStructure(m_FileLinkJointRank.c_str());

}
