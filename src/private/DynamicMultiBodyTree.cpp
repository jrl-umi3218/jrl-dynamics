void DynMultiBodyPrivate::SpecifyTheRootLabel(int ID)
{
  labelTheRoot = ID;
  m_listOfBodies[ID]->setLabelMother(-1);

  // Right now it is assume that the first body is the world,
  // and that this body is related to another body.
  // Thus liaisons[ID].size() should be equal to one.


  if (liaisons[ID].size()!=1)
    {
      cout << "Wrong assumption concerning the initial body." << endl;
      return;
    }
  int ld = liaisons[ID][0].liaison;
  m_RootOfTheJointsTree = listeLiaisons[ld].aJoint;
  m_RootOfTheJointsTree->setLinkedBody(*m_listOfBodies[ID]);
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
	      m_listOfBodies[lMother]->child = i;					  // Mother, I am your daughter !

            }
	  else
            {
	      // I have an elder sister !

	      while (m_listOfBodies[lElderSister]->sister != -1)
		lElderSister = m_listOfBodies[lElderSister]->sister;  // I have another elder sister !

	      m_listOfBodies[lElderSister]->sister = i;				  // I am your younger sister !
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
  m_listOfBodies[BodyID]->a =  listeLiaisons[JointID].aJoint->axe();
  ODEBUG(" axe: " << m_listOfBodies[BodyID]->a);
  // Update the translation vector
  listeLiaisons[JointID].aJoint->getStaticTranslation(m_listOfBodies[BodyID]->b);
  ODEBUG(" JointID: " << JointID << "BodyID: " << BodyID << ", static translation: " << m_listOfBodies[BodyID]->b);
  // Update the rotation matrix
  listeLiaisons[JointID].aJoint->getStaticRotation(m_listOfBodies[BodyID]->R_static);
  ODEBUG(" Rotation matrix: " << endl << m_listOfBodies[BodyID]->R_static);
  listeLiaisons[JointID].aJoint->setLinkedBody(*m_listOfBodies[BodyID]);
  m_listOfBodies[BodyID]->joint( listeLiaisons[JointID].aJoint);

}

void DynMultiBodyPrivate::ReLabelling(int corpsCourant, int liaisonDeProvenance)
{
  // This one has been nicely clean-up
  for (unsigned int i=0; i<liaisons[corpsCourant].size(); i++)
    {
      int liaisonDestination = liaisons[corpsCourant][i].liaison;
      if ((liaisonDestination == liaisonDeProvenance) &&
	  (corpsCourant!=labelTheRoot))
	continue;

      int corps1 = listeLiaisons[liaisonDestination].indexCorps1;
      int corps2 = listeLiaisons[liaisonDestination].indexCorps2;
      int corpsMother,corpsSon;

      if ((corpsCourant == corps1) && (corpsCourant != corps2))
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

      if(listeLiaisons[liaisonDestination].aJoint->getIDinActuated()!=-1)
        {


	  // Update the connections between the Joints.
	  int lIDinActuated = listeLiaisons[liaisonDestination].aJoint->getIDinActuated();
	  if (lIDinActuated!=-1)
            {
	      ConvertIDInActuatedToBodyID[lIDinActuated] = corpsSon;
            }


	  listeLiaisons[liaisonDeProvenance].aJoint->addChildJoint(*listeLiaisons[liaisonDestination].aJoint);

	  listeLiaisons[liaisonDestination].aJoint->SetFatherJoint(listeLiaisons[liaisonDeProvenance].aJoint);


	  // Update the vector of joints.
	  ODEBUG("Inside Relabelling :" << listeLiaisons[liaisonDestination].aJoint->getName().c_str());
	  ODEBUG("JointRankFromName :" << JointRankFromName(listeLiaisons[liaisonDestination].aJoint));
	  if (JointRankFromName(listeLiaisons[liaisonDestination].aJoint)!=-1)
	    m_JointVector.insert(m_JointVector.end(),listeLiaisons[liaisonDestination].aJoint);

	  int lVRMLID = listeLiaisons[liaisonDestination].aJoint->getIDinActuated();
	  if (m_NbOfVRMLIDs < lVRMLID)
	    m_NbOfVRMLIDs = lVRMLID;


        }

      // TODO : It is important to do the relabelling after the
      // TODO : recursive call to the relabelling as set father build
      // TODO : up the JointFromRootToThis vector. Should be fixed at the Joint object level.
      ReLabelling(corpsSon, liaisonDestination);
      UpdateBodyParametersFromJoint(corpsSon,liaisonDestination,liaisonDeProvenance);

    }

}

void DynMultiBodyPrivate::CreatesTreeStructure(const char * option)
{
  m_listOfBodies.resize(listeCorps.size());
  DynamicBody *dbody;
  for(unsigned int i=0;i<listeCorps.size();i++)
    {
      dbody = dynamic_cast<DynamicBody *>(listeCorps[i]);
      if (!dbody)
        {
	  dbody = new DynamicBody();
	  *dbody = *listeCorps[i];
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
  UpdateTheSizeOfJointsJacobian();

  MAL_VECTOR_RESIZE(m_Configuration,m_NbDofs);
  MAL_VECTOR_RESIZE(m_Velocity,m_NbDofs);
  MAL_VECTOR_RESIZE(m_Acceleration,m_NbDofs);

  MAL_MATRIX_RESIZE(m_Forces ,m_NbDofs,3);
  MAL_MATRIX_RESIZE(m_Torques,m_NbDofs,3);

  MAL_VECTOR_RESIZE(m_pastConfiguration,m_NbDofs);
  MAL_VECTOR_RESIZE(m_pastVelocity,m_NbDofs);
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
  vectorOfBodies[0] = new DynamicBody();
  vectorOfBodies[0]->setLabel(NbOfBodies++);
  ajouterCorps(*vectorOfBodies[0]);
  Depth++;

  MAL_S3_VECTOR(,double) dummy;


  // Go through the Joints tree.
  Joint *CurrentJoint = m_RootOfTheJointsTree;
  Joint *NextCurrentJoint=0,*FatherJoint;
  double mi[9]={ 1.0,1.0,1.0, 1.0,1.0,1.0, 1.0,1.0,1.0};
  double cm[3] = { 0.0,0.0,0.0};

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
	      if (CurrentLink.aJoint->type() == Joint::FIX_JOINT)
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
      CjrlBody* jrlBody = CurrentJoint->linkedBody();
      Body * lCurrentBody;
      if (jrlBody != NULL)
        {
	  lCurrentBody = (Body *)jrlBody;
	  DynamicBody *dbody = dynamic_cast<DynamicBody *>(lCurrentBody);
	  dbody->c = jrlBody->localCenterOfMass();
        }
      else
        {
	  lCurrentBody = new DynamicBody();
	  lCurrentBody->setInertie(mi);
	  lCurrentBody->setMasse(1.0);// 1 kg per default.
	  lCurrentBody->setPositionCoM(cm);
        }
      vectorOfBodies[Depth] = lCurrentBody;
      lCurrentBody->setLabel(NbOfBodies++);
      string lCurrentBodyName=CurrentJoint->getName();
      lCurrentBodyName = "BODY_" + lCurrentBodyName;
      lCurrentBody->setName((char *)lCurrentBodyName.c_str());
      lCurrentBody->setLabelMother(vectorOfBodies[Depth-1]->getLabel());


      ajouterCorps(*lCurrentBody);
      ajouterLiaison(*vectorOfBodies[Depth-1],
		     *lCurrentBody,
		     CurrentLink);

      // Find the next one.
      // 1. A children.
      NextCurrentJoint = (dynamicsJRLJapan::Joint *)CurrentJoint->childJoint(0);
      Depth++;

      // No child.
      if (NextCurrentJoint==0)
        {
	  // If a father exist.
	  FatherJoint = (dynamicsJRLJapan::Joint *)CurrentJoint->parentJoint();
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
		  NextCurrentJoint = (dynamicsJRLJapan::Joint *)FatherJoint->childJoint(CurrentJointPosition+1);
                }
	      else
                {
		  // otherwise move upward.
		  CurrentJoint =FatherJoint;
		  FatherJoint=(dynamicsJRLJapan::Joint *)FatherJoint->parentJoint();
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
