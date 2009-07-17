
/*! Implements methods related to propertis for DynamicsMultiBody */
bool DynMultiBodyPrivate::getProperty(const std::string &inProperty,std::string &outValue)
{
  if (inProperty=="ComputeVelocity")
    {
      if (m_ComputeVelocity)
	outValue="true";
      else
	outValue="false";
      return true;
    }
  else if (inProperty=="ComputeAcceleration")
    {
      if (m_ComputeAcceleration)
	outValue="true";
      else
	outValue="false";
      return true;

    }
  else if (inProperty=="ComputeCoM")
    {
      if (m_ComputeCoM)
	outValue="true";
      else
	outValue="false";
      return true;

    }
  else if (inProperty=="ComputeMomentum")
    {
      if (m_ComputeMomentum)
	outValue="true";
      else
	outValue="false";
      return true;
    }
  else if (inProperty=="ComputeAccelerationCoM")
    {
      if (m_ComputeAccCoM)
	outValue="true";
      else
	outValue="false";
      return true;

    }
  else if (inProperty=="ComputeBackwardDynamics")
    {
      if (m_ComputeBackwardDynamics)
	outValue="true";
      else
	outValue="false";
      return true;

    }
  else if (inProperty=="ComputeZMP")
    {
      if (m_ComputeZMP)
	outValue="true";
      else
	outValue="false";
      return true;

    }
  else if (inProperty=="TimeStep")
    {
      ostringstream aos;
      aos << m_TimeStep;
      outValue=aos.str();
    }
  else if (inProperty=="ResetIteration")
    {
      ResetIterationNumber();
    }
  outValue="false";
  return false;
}

bool DynMultiBodyPrivate::setProperty(std::string &inProperty,const std::string &inValue)
{
  if (inProperty=="ComputeVelocity")
    {
      if (inValue=="true")
        {
	  setComputeVelocity(true);
	  return true;
        }
      else if (inValue=="false")
        {
	  setComputeVelocity(false);
	  return true;
        }
    }
  else if (inProperty=="ComputeAcceleration")
    {
      if (inValue=="true")
        {
	  setComputeAcceleration(true);
	  return true;
        }
      else if (inValue=="false")
        {
	  setComputeAcceleration(false);
	  return true;
        }
    }
  else if (inProperty=="ComputeMomentum")
    {
      if (inValue=="true")
        {
	  setComputeMomentum(true);
	  return true;
        }
      else if (inValue=="false")
        {
	  setComputeMomentum(false);
	  return true;
        }
    }
  else if (inProperty=="ComputeCoM")
    {
      if (inValue=="true")
        {
	  setComputeCoM(true);
	  return true;
        }
      else if (inValue=="false")
        {
	  setComputeCoM(false);
	  return true;
        }
    }
  else if (inProperty=="ComputeAccelerationCoM")
    {
      if (inValue=="true")
        {
	  setComputeAccelerationCoM(true);
	  return true;
        }
      else if (inValue=="false")
        {
	  setComputeAccelerationCoM(false);
	  return true;
        }
    }
  else if (inProperty=="ComputeBackwardDynamics")
    {
      if (inValue=="true")
        {
	  setComputeBackwardDynamics(true);
	  return true;
        }
      else if (inValue=="false")
        {
	  setComputeBackwardDynamics(false);
	  return true;
        }
    }
  else if (inProperty=="ComputeZMP")
    {
      if (inValue=="true")
        {
	  setComputeZMP(true);
	  return true;
        }
      else if (inValue=="false")
        {
	  setComputeZMP(false);
	  return true;
        }

    }
  else if (inProperty=="FileJointRank")
    {
      m_FileLinkJointRank = inValue;
    }
  else if (inProperty=="TimeStep")
    {
      istringstream iss(inValue);
      iss >> m_TimeStep;
    }
  return false;
}

bool DynMultiBodyPrivate::isSupported(const std::string &aName)
{
  if (aName=="ComputeVelocity")
    return true;
  else if (aName=="ComputeAcceleration")
    return true;
  else if (aName=="ComputeCoM")
    return true;
  else if (aName=="ComputeAccCoM")
    return true;
  else if (aName=="ComputeBackwardDynamics")
    return true;
  else if (aName=="ComputeZMP")
    return true;
  else if (aName=="TimeStep")
    return true;
  return false;
}
