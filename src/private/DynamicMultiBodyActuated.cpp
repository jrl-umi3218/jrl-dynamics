/* Implements Actuated methods of DynamicsMultiBody */
void DynMultiBodyPrivate::setActuatedJoints(std::vector<CjrlJoint *>& lActuatedJoints)
{
  bool Same=true;

  if (m_ActuatedJoints.size()==
      lActuatedJoints.size())
    {
      for(unsigned int i=0;i<m_ActuatedJoints.size();i++)
	if (m_ActuatedJoints[i] != lActuatedJoints[i])
	  Same=false;
      
    }

  if (Same)
    return;

  m_ActuatedJoints = lActuatedJoints;

  m_SynchronizationBetweenActuatedVectorAndJoints = false;
}

const std::vector<CjrlJoint *>& DynMultiBodyPrivate::getActuatedJoints() const
{
  return m_ActuatedJoints;
}


void DynMultiBodyPrivate::GetJointIDInConfigurationFromActuatedID(std::vector<int> &VectorFromActuatedIDToConfigurationID)
{
  VectorFromActuatedIDToConfigurationID.clear();
  VectorFromActuatedIDToConfigurationID = m_ActuatedIDToConfiguration;
}

int DynMultiBodyPrivate::BuildLinkBetweenActuatedVectorAndJoints()
{
  m_ActuatedIDToConfiguration.resize(m_ActuatedJoints.size());

  for(unsigned int IndexInActuatedVector=0;
      IndexInActuatedVector<m_ActuatedJoints.size();
      IndexInActuatedVector++)
    {
      bool FoundActuatedJoint = false;
      unsigned int IndexInJointVector=0;
      for(IndexInJointVector=0;IndexInJointVector<m_JointVector.size();IndexInJointVector++)
	if (m_JointVector[IndexInJointVector]==m_ActuatedJoints[IndexInActuatedVector])
	  {
	    FoundActuatedJoint = true;
	    break;
	  }
      if (FoundActuatedJoint)
	{
	  ((Joint *)(m_JointVector[IndexInJointVector]))->setIDinActuated(IndexInActuatedVector);
	  m_ActuatedIDToConfiguration[IndexInActuatedVector] = IndexInJointVector;
	}
      else
	{
	  cerr << "Unable to find actuated joint: "<< m_ActuatedJoints[IndexInActuatedVector] << 
	    " in the vector of joints." << endl;
	  return -1;
	}
    }
  m_SynchronizationBetweenActuatedVectorAndJoints = true;
  return 0;
}

CjrlJoint* DynMultiBodyPrivate::GetJointFromActuatedID(int JointID)
{

  for(unsigned int i=0;i<m_JointVector.size();i++)
    {
      Joint * r;
      if (((r=(Joint *)m_JointVector[i])->getIDinActuated())==JointID)
        {
	  ODEBUG("Joint : "<< r->getName() << " " << JointID );

	  return r;
        }
    }
  return 0;
}
