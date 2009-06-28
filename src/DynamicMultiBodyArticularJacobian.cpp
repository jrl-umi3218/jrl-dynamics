bool DynamicMultiBody::getJacobian ( const CjrlJoint& inStartJoint, 
				     const CjrlJoint& inEndJoint, 
				     const vector3d& inFrameLocalPosition, 
				     matrixNxP& outjacobian, 
				     unsigned int outOffset, 
				     bool includeFreeFlyer )
{
  unsigned int valNumberDof = ( includeFreeFlyer==true ) ?numberDof() :numberDof()-6;
  unsigned int lengthJacobian = MAL_MATRIX_NB_COLS(outjacobian);
  if ( ( MAL_MATRIX_NB_ROWS(outjacobian) != 6 ) || 
       ( lengthJacobian < valNumberDof + outOffset ) )
    return false;
  
  unsigned int i,j;
  double ** outTable;
  outTable = new double* [6];
  for ( i=0; i<6; i++ )
    outTable[i] = new double [valNumberDof];
  for ( i=0; i<6; i++ )
    memset ( outTable[i], 0, valNumberDof*sizeof ( double ) );
  
  //determine participating joints
  std::vector<Joint *> robotRoot2StartJoint, robotRoot2EndJoint;
  Joint* StartJoint = ( Joint* ) ( &inStartJoint );
  Joint* EndJoint = ( Joint* ) ( &inEndJoint );
  robotRoot2StartJoint = StartJoint->jointsFromRootToThisJoint();
  robotRoot2EndJoint = EndJoint->jointsFromRootToThisJoint();
  
  unsigned int offset = 1;
  unsigned int minChain = ( robotRoot2StartJoint.size() <robotRoot2EndJoint.size() ) 
    ?robotRoot2StartJoint.size() :robotRoot2EndJoint.size();
  
  for ( i=1; i< minChain; i++ )
    {
      if ( ( robotRoot2StartJoint[i]==robotRoot2EndJoint[i] ) )
	offset++;
    }
  
  unsigned int rank;
  Joint* aJoint;
  DynamicBody* aBody = EndJoint->linkedDBody();
  tempP = aBody->p + MAL_S3x3_RET_A_by_B ( aBody->R, inFrameLocalPosition );

  for ( i=offset;i<robotRoot2EndJoint.size();i++ )
    {
      aJoint=  robotRoot2EndJoint[i];
      aBody=  aJoint->linkedDBody();
      if ( includeFreeFlyer )
	rank = aJoint->rankInConfiguration();
      else
	rank = aJoint->rankInConfiguration()-6;

      tempDP = tempP - aBody->p;

      switch ( aJoint->type() )
        {
	case Joint::REVOLUTE_JOINT:
	  MAL_S3_VECTOR_CROSS_PRODUCT ( tempLV,aBody->w_a,tempDP );
	  for ( j=0;j<3;j++ )
	    {
	      outTable[j][rank] =  tempLV[j];
	      outTable[j+3][rank] = aBody->w_a[j];
	    }
	  break;
	case Joint::PRISMATIC_JOINT:
	  for ( j=0;j<3;j++ )
	    {
	      outTable[j][rank] =  aBody->w_a[j];
	    }
	  break;
	case Joint::FREE_JOINT:
	  for ( j=0;j<3;j++ )
	    {
	      outTable[j][rank+j] =  1.0;
	      outTable[j+3][rank+j+3] =  1.0;
	    }
	  outTable[1][rank+3] =  -tempDP[2];
	  outTable[2][rank+3] =  tempDP[1];
	  outTable[0][rank+4] =  tempDP[2];
	  outTable[2][rank+4] =  -tempDP[0];
	  outTable[0][rank+5] =  -tempDP[1];
	  outTable[1][rank+5] =  tempDP[0];
	  break;
        }
    }

  for ( i=offset;i<robotRoot2StartJoint.size();i++ )
    {
      aJoint = robotRoot2StartJoint[i];
      aBody=  aJoint->linkedDBody();
      if ( includeFreeFlyer )
	rank = aJoint->rankInConfiguration();
      else
	rank = aJoint->rankInConfiguration()-6;

      tempDP = tempP - aBody->p;

      switch ( aJoint->type() )
        {
	case Joint::REVOLUTE_JOINT:
	  MAL_S3_VECTOR_CROSS_PRODUCT ( tempLV,aBody->w_a,tempDP );
	  for ( j=0;j<3;j++ )
	    {
	      outTable[j][rank] = -tempLV[j];
	      outTable[j+3][rank] = -aBody->w_a[j];
	    }
	  break;
	case Joint::PRISMATIC_JOINT:
	  for ( j=0;j<3;j++ )
	    {
	      outTable[j][rank] = -aBody->w_a[j];
	    }
	  break;
	case Joint::FREE_JOINT:
	  for ( j=0;j<3;j++ )
	    {
	      outTable[j][rank+j] =  -1.0;
	      outTable[j+3][rank+j+3] =  -1.0;
	    }
	  outTable[1][rank+3] =  tempDP[2];
	  outTable[2][rank+3] =  -tempDP[1];
	  outTable[0][rank+4] =  -tempDP[2];
	  outTable[2][rank+4] =  tempDP[0];
	  outTable[0][rank+5] =  tempDP[1];
	  outTable[1][rank+5] =  -tempDP[0];
	  break;
        }
    }

  if ( includeFreeFlyer )
    {
      tempDP = tempP - StartJoint->linkedDBody()->p;

      for ( j=0;j<6;j++ )
	outTable[j][j] =  1.0;
      outTable[1][3] =  -tempDP[2];
      outTable[2][3] =  tempDP[1];
      outTable[0][4] =  tempDP[2];
      outTable[2][4] =  -tempDP[0];
      outTable[0][5] =  -tempDP[1];
      outTable[1][5] =  tempDP[0];
    }

  for ( i=0; i<6; i++ )
    memcpy ( ( &outjacobian.data() [i*lengthJacobian+outOffset] ),outTable[i],
	     valNumberDof*sizeof ( double ) );

  //clean
  for ( i=0; i<6; i++ )
    delete [] outTable[i];
  delete [] outTable ;

  return true;
}

bool DynamicMultiBody::getPositionJacobian ( const CjrlJoint& inStartJoint, 
					     const CjrlJoint& inEndJoint, 
					     const vector3d& inFrameLocalPosition, 
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

  //determine participating joints
  std::vector<Joint *> robotRoot2StartJoint, robotRoot2EndJoint;
  Joint* StartJoint = ( Joint* ) ( &inStartJoint );
  Joint* EndJoint = ( Joint* ) ( &inEndJoint );
  robotRoot2StartJoint = StartJoint->jointsFromRootToThisJoint();
  robotRoot2EndJoint = EndJoint->jointsFromRootToThisJoint();

  unsigned int offset = 1;
  unsigned int minChain = ( robotRoot2StartJoint.size() <robotRoot2EndJoint.size() ) 
    ?robotRoot2StartJoint.size() :robotRoot2EndJoint.size();

  for ( i=1; i< minChain; i++ )
    {
      if ( ( robotRoot2StartJoint[i]==robotRoot2EndJoint[i] ) )
	offset++;
    }

  unsigned int rank;
  Joint* aJoint;
  DynamicBody* aBody = EndJoint->linkedDBody();
  tempP = aBody->p + MAL_S3x3_RET_A_by_B ( aBody->R, inFrameLocalPosition );

  for ( i=offset;i<robotRoot2EndJoint.size();i++ )
    {
      aJoint=  robotRoot2EndJoint[i];
      aBody=  aJoint->linkedDBody();
      if ( includeFreeFlyer )
	rank = aJoint->rankInConfiguration();
      else
	rank = aJoint->rankInConfiguration()-6;

      tempDP = tempP - aBody->p;

      switch ( aJoint->type() )
        {
	case Joint::REVOLUTE_JOINT:
	  MAL_S3_VECTOR_CROSS_PRODUCT ( tempLV,aBody->w_a,tempDP );
	  for ( j=0;j<3;j++ )
	    outTable[j][rank] =  tempLV[j];
	  break;
	case Joint::PRISMATIC_JOINT:
	  for ( j=0;j<3;j++ )
	    outTable[j][rank] =  aBody->w_a[j];
	  break;
	case Joint::FREE_JOINT:
	  for ( j=0;j<3;j++ )
	    outTable[j][rank+j] =  1.0;
	  outTable[1][rank+3] =  -tempDP[2];
	  outTable[2][rank+3] =  tempDP[1];
	  outTable[0][rank+4] =  tempDP[2];
	  outTable[2][rank+4] =  -tempDP[0];
	  outTable[0][rank+5] =  -tempDP[1];
	  outTable[1][rank+5] =  tempDP[0];
	  break;
        }
    }

  for ( i=offset;i<robotRoot2StartJoint.size();i++ )
    {
      aJoint = robotRoot2StartJoint[i];
      aBody=  aJoint->linkedDBody();
      if ( includeFreeFlyer )
	rank = aJoint->rankInConfiguration();
      else
	rank = aJoint->rankInConfiguration()-6;

      tempDP = tempP - aBody->p;

      switch ( aJoint->type() )
        {
	case Joint::REVOLUTE_JOINT:
	  MAL_S3_VECTOR_CROSS_PRODUCT ( tempLV,aBody->w_a,tempDP );
	  for ( j=0;j<3;j++ )
	    outTable[j][rank] = -tempLV[j];
	  break;
	case Joint::PRISMATIC_JOINT:
	  for ( j=0;j<3;j++ )
	    outTable[j][rank] = -aBody->w_a[j];
	  break;
	case Joint::FREE_JOINT:
	  for ( j=0;j<3;j++ )
	    outTable[j][rank+j] =  -1.0;
	  outTable[1][rank+3] =  tempDP[2];
	  outTable[2][rank+3] =  -tempDP[1];
	  outTable[0][rank+4] =  -tempDP[2];
	  outTable[2][rank+4] =  tempDP[0];
	  outTable[0][rank+5] =  tempDP[1];
	  outTable[1][rank+5] =  -tempDP[0];
	  break;
        }
    }
  if ( includeFreeFlyer )
    {
      tempDP = tempP - StartJoint->linkedDBody()->p;

      for ( j=0;j<3;j++ )
	outTable[j][j] =  1.0;

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

bool DynamicMultiBody::getOrientationJacobian ( const CjrlJoint& inStartJoint, 
						const CjrlJoint& inEndJoint, 
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

  //determine participating joints
  std::vector<Joint *> robotRoot2StartJoint, robotRoot2EndJoint;
  Joint* StartJoint = ( Joint* ) ( &inStartJoint );
  Joint* EndJoint = ( Joint* ) ( &inEndJoint );
  robotRoot2StartJoint = StartJoint->jointsFromRootToThisJoint();
  robotRoot2EndJoint = EndJoint->jointsFromRootToThisJoint();

  unsigned int offset = 1;
  unsigned int minChain = ( robotRoot2StartJoint.size() <robotRoot2EndJoint.size() ) 
    ?robotRoot2StartJoint.size() :robotRoot2EndJoint.size();

  for ( i=1; i< minChain; i++ )
    {
      if ( ( robotRoot2StartJoint[i]==robotRoot2EndJoint[i] ) )
	offset++;
    }

  unsigned int rank;
  Joint* aJoint;
  DynamicBody* aBody;

  for ( i=offset;i<robotRoot2EndJoint.size();i++ )
    {
      aJoint=  robotRoot2EndJoint[i];
      aBody=  aJoint->linkedDBody();
      if ( includeFreeFlyer )
	rank = aJoint->rankInConfiguration();
      else
	rank = aJoint->rankInConfiguration()-6;

      switch ( aJoint->type() )
        {
	case Joint::REVOLUTE_JOINT:
	  for ( j=0;j<3;j++ )
	    outTable[j][rank] = aBody->w_a[j];

	  break;
	case Joint::PRISMATIC_JOINT:
	  break;
	case Joint::FREE_JOINT:
	  for ( j=0;j<3;j++ )
	    outTable[j][rank+j] =  1.0;
	  break;
        }
    }

  for ( i=offset;i<robotRoot2StartJoint.size();i++ )
    {
      aJoint = robotRoot2StartJoint[i];
      aBody=  aJoint->linkedDBody();
      if ( includeFreeFlyer )
	rank = aJoint->rankInConfiguration();
      else
	rank = aJoint->rankInConfiguration()-6;

      tempDP = tempP - aBody->p;

      switch ( aJoint->type() )
        {
	case Joint::REVOLUTE_JOINT:
	  for ( j=0;j<3;j++ )
	    outTable[j][rank] = -aBody->w_a[j];

	  break;
	case Joint::PRISMATIC_JOINT:
	  break;
	case Joint::FREE_JOINT:
	  for ( j=0;j<3;j++ )
	    outTable[j][rank+j] =  -1.0;
	  break;
        }
    }

  for ( i=0;i<3;i++ )
    {
      if ( includeFreeFlyer )
	outTable[i][i+3] =  1.0;
      memcpy ( ( &outjacobian.data() [i*lengthJacobian+outOffset] ),
	       outTable[i],valNumberDof *sizeof ( double ) );
    }

  //clean
  for ( i=0; i<3; i++ )
    delete ( outTable[i] );
  delete ( outTable );
  return true;
}
