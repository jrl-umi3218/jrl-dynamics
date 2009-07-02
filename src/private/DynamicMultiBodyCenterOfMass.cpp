
// Compute the Jacobian matrix of the center of Mass. 
// Interaction with the environement not taken into account.
void DynMultiBodyPrivate::computeJacobianCenterOfMass()
{
  m_JacobianOfTheCoM.clear();

  std::vector<CjrlJoint*> routeJoints;
  CjrlBody* body;
  CjrlJoint* joint;

  double weight;
  unsigned int rank,i,k,l;

  const vector3d & comPos =  positionCenterOfMass();
  const matrix4d & rootM = rootJoint()->currentTransformation();

  for (i=0; i < m_JointVector.size(); i++)
    {
      joint = m_JointVector[i];
      if (joint == rootJoint())
	continue;

      body = joint->linkedBody();
      const vector3d& localCOM = body->localCenterOfMass();
      weight = body->mass()/mass();
      joint->getJacobianPointWrtConfig( localCOM, m_attCalcJointJacobian );
      routeJoints = joint->jointsFromRootToThis();
      for (k= 1; k< routeJoints.size(); k++)
        {
	  rank = routeJoints[k]->rankInConfiguration();
	  for (l=0; l<3;l++)
	    m_JacobianOfTheCoM(l,rank) += weight * m_attCalcJointJacobian(l,rank);
        }
    }

  for (k= 0; k<3;k++)
    m_JacobianOfTheCoM(k,k) = 1.0;

  m_JacobianOfTheCoM(1,3) = MAL_S4x4_MATRIX_ACCESS_I_J(rootM,2,3) - comPos[2];
  m_JacobianOfTheCoM(2,3) = comPos[1] - MAL_S4x4_MATRIX_ACCESS_I_J(rootM,1,3);

  m_JacobianOfTheCoM(0,4) = -m_JacobianOfTheCoM(1,3);
  m_JacobianOfTheCoM(2,4) = MAL_S4x4_MATRIX_ACCESS_I_J(rootM,0,3) - comPos[0];

  m_JacobianOfTheCoM(0,5) = MAL_S4x4_MATRIX_ACCESS_I_J(rootM,1,3) - comPos[1];
  m_JacobianOfTheCoM(1,5) = -m_JacobianOfTheCoM(2,4);
}

const MAL_MATRIX(,double) &DynMultiBodyPrivate::jacobianCenterOfMass() const
{
  return m_JacobianOfTheCoM;

}

MAL_MATRIX(,double) &DynMultiBodyPrivate::getJacobianOfTheCoM()
{
  return m_JacobianOfTheCoM;
}

/**
   \brief Get the acceleration of the center of mass.
*/
const MAL_S3_VECTOR(,double)& DynMultiBodyPrivate::accelerationCenterOfMass()
{
  return m_AccelerationCenterOfMass;
};

/**
   \brief Get the position of the center of mass.
*/
const MAL_S3_VECTOR(,double)&  DynMultiBodyPrivate::positionCenterOfMass() const
{
  return positionCoMPondere;
}


/**
   \brief Get the velocity of the center of mass.
*/
const MAL_S3_VECTOR(,double)& DynMultiBodyPrivate::velocityCenterOfMass()
{
  return m_VelocityCenterOfMass;
};


bool DynMultiBodyPrivate::getJacobianCenterOfMass ( const CjrlJoint& inStartJoint, 
						 matrixNxP& outjacobian, 
						 unsigned int outOffset, 
						 bool includeFreeFlyer )
{
  unsigned int valNumberDof = ( includeFreeFlyer==true ) ?numberDof() :numberDof()-6;
  unsigned int lengthJacobian = MAL_MATRIX_NB_COLS(outjacobian);
  if ( ( MAL_MATRIX_NB_ROWS(outjacobian) != 3 ) || ( lengthJacobian < valNumberDof + outOffset ) )
    return false;

  unsigned int i,j;
  double ** outTable;
  outTable = new double* [3];
  for ( i=0; i<3; i++ )
    outTable[i] = new double [valNumberDof];

  for ( i=0; i<3; i++ )
    memset ( outTable[i], 0, valNumberDof*sizeof ( double ) );

  std::vector<int> jointsigns ( numberDof(),1 );

  JointPrivate* StartJoint = ( JointPrivate* ) ( &inStartJoint );
  //determine participating joints
  if ( rootJoint() !=&inStartJoint )
    {
      std::vector<JointPrivate *> robotRoot2StartJoint = StartJoint->jointsFromRootToThisJoint();

      for ( i = 1; i<robotRoot2StartJoint.size();i++ )
	jointsigns[robotRoot2StartJoint[i]->rankInConfiguration() ] = -1;

      std::vector<vector3d> tempoS;
      std::vector<double> liftedS;

      for ( i = 0; i<robotRoot2StartJoint.size()-1;i++ )
        {
	  robotRoot2StartJoint[i]->computeSubTreeMComExceptChild ( robotRoot2StartJoint[i+1] );
	  tempoS.push_back ( robotRoot2StartJoint[i]->subTreeMCom() );
	  liftedS.push_back ( robotRoot2StartJoint[i]->subTreeCoef() );
        }

      robotRoot2StartJoint[1]->subTreeMCom ( tempoS[0] );
      robotRoot2StartJoint[1]->subTreeCoef ( liftedS[0] );

      for ( i = 2; i<robotRoot2StartJoint.size();i++ )
        {
	  robotRoot2StartJoint[i]->subTreeMCom ( robotRoot2StartJoint[i-1]->subTreeMCom() +tempoS[i-1] );
	  robotRoot2StartJoint[i]->subTreeCoef ( robotRoot2StartJoint[i-1]->subTreeCoef() +liftedS[i-1] );
        }
    }
  else
    StartJoint->computeSubTreeMCom();

  unsigned int rank;
  JointPrivate* aJoint;
  DynamicBodyPrivate* aBody;

  for ( i=0;i<m_ConfigurationToJoints.size();i++ )
    {
      if ( m_ConfigurationToJoints[i] == rootJoint() )
	continue;

      aJoint = m_ConfigurationToJoints[i];
      aBody=  aJoint->linkedDBody();
      if ( includeFreeFlyer )
	rank = aJoint->rankInConfiguration();
      else
	rank = aJoint->rankInConfiguration()-6;

      switch ( aJoint->type() )
        {
	case JointPrivate::REVOLUTE_JOINT:
	  for ( j=0;j<3;j++ )
	    tempDP[j] = aJoint->subTreeMCom() [j]- aJoint->subTreeCoef() *aBody->p[j];
	  MAL_S3_VECTOR_CROSS_PRODUCT ( tempLV,aBody->w_a,tempDP );
	  for ( j=0;j<3;j++ )
	    if ( jointsigns[m_ConfigurationToJoints[i]->rankInConfiguration() ]>0 )
	      outTable[j][rank] =  tempLV[j];
	    else
	      outTable[j][rank] =  -tempLV[j];
	  break;
	case JointPrivate::PRISMATIC_JOINT:
	  for ( j=0;j<3;j++ )
	    {
	      if ( jointsigns[m_ConfigurationToJoints[i]->rankInConfiguration() ]>0 )
		outTable[j][rank] = aBody->w_a[j];
	      else
		outTable[j][rank] = -aBody->w_a[j];
	    }
	  break;
	case JointPrivate::FREE_JOINT:
	  for ( j=0;j<3;j++ )
	    {
	      if ( jointsigns[m_ConfigurationToJoints[i]->rankInConfiguration() ]>0 )
		outTable[j][rank+j] =  1.0;
	      else
		outTable[j][rank+j] =  -1.0;
	    }
	  for ( j=0;j<3;j++ )
	    tempDP[j] = aJoint->subTreeMCom() [j]- aJoint->subTreeCoef() *aBody->p[j];
	  if ( jointsigns[m_ConfigurationToJoints[i]->rankInConfiguration() ]>0 )
	    {
	      outTable[1][rank+3] =  -tempDP[2];
	      outTable[2][rank+3] =  tempDP[1];
	      outTable[0][rank+4] =  tempDP[2];
	      outTable[2][rank+4] =  -tempDP[0];
	      outTable[0][rank+5] =  -tempDP[1];
	      outTable[1][rank+5] =  tempDP[0];
	    }
	  else
	    {
	      outTable[1][rank+3] =  tempDP[2];
	      outTable[2][rank+3] =  -tempDP[1];
	      outTable[0][rank+4] =  -tempDP[2];
	      outTable[2][rank+4] =  tempDP[0];
	      outTable[0][rank+5] =  tempDP[1];
	      outTable[1][rank+5] =  -tempDP[0];
	    }

	  break;
        }

    }
  if ( includeFreeFlyer )
    {

      for ( j=0;j<3;j++ )
	outTable[j][j] =  1.0;
      tempDP = positionCoMPondere - StartJoint->linkedDBody()->p;
      outTable[1][3] =  -tempDP[2];
      outTable[2][3] =  tempDP[1];
      outTable[0][4] =  tempDP[2];
      outTable[2][4] =  -tempDP[0];
      outTable[0][5] =  -tempDP[1];
      outTable[1][5] =  tempDP[0];
    }
  for ( i=0; i<3; i++ )
    memcpy ( ( &outjacobian.data() [i*lengthJacobian+outOffset] ),outTable[i],valNumberDof *sizeof ( double ) );

  //clean
  for ( i=0; i<3; i++ )
    delete ( outTable[i] );
  delete ( outTable );
  return true;
}

void DynMultiBodyPrivate::getJacobianLinearMomentumWrtCoM(matrixNxP &outjacobian)
{
  matrixNxP JCoM;
  getJacobianCenterOfMass(*rootJoint(),JCoM);
  outjacobian = masse * JCoM;
}

/**
   @}
*/
/*
  \brief Compute the dynamics of the center of mass.
  
  Compute the linear and  angular momentum and their time derivatives, at the center of mass.
*/
bool DynMultiBodyPrivate::computeCenterOfMassDynamics()
{
  computeForwardKinematics();
  return true;
};


/**
   \brief Get the linear momentum of the robot.
*/
const MAL_S3_VECTOR(,double)& DynMultiBodyPrivate::linearMomentumRobot()
{
  return m_P;
};

/**
   \brief Get the time-derivative of the linear momentum.
*/
const MAL_S3_VECTOR(,double)& DynMultiBodyPrivate::derivativeLinearMomentum()
{
  return m_dP;
};

MAL_S3_VECTOR(,double) DynMultiBodyPrivate::getPositionCoM(void)
{
  return (positionCoMPondere);
}

void DynMultiBodyPrivate::GetPandL(MAL_S3_VECTOR(,double) &aP, MAL_S3_VECTOR(,double) &aL)
{
  aP = m_P;
  aL = m_L;
}
