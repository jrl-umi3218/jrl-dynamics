/*! Implements Inertia Matrix methods of DynMultiBodyPrivate. */

void DynMultiBodyPrivate::computeInertiaMatrix()
{
  if ((MAL_MATRIX_NB_ROWS(m_InertiaMatrix) != numberDof()) || 
      (MAL_MATRIX_NB_COLS(m_InertiaMatrix) != numberDof()))
    MAL_MATRIX_RESIZE(m_InertiaMatrix,numberDof(),numberDof());

  MAL_MATRIX_FILL(m_InertiaMatrix,0);

  unsigned int rank;
  JointPrivate* aJoint;
  DynamicBodyPrivate* aBody;
    
  for(unsigned int i=1;i<m_listOfBodies.size();i++)
    {
      aBody=  m_listOfBodies[i];
      aJoint=(JointPrivate *)aBody->joint();
      
      rank = aJoint->rankInConfiguration();
      
      matrixNxP pJacobian;
      MAL_MATRIX_RESIZE(pJacobian, 6, numberDof());
      vector3d aCoM = aBody->getPositionCoM(); // Dummy 
      getJacobian(*rootJoint(),*aJoint,aCoM,pJacobian);

      ODEBUG("pJacobian:" <<pJacobian);
      matrixNxP pLinearJacobian;
      MAL_MATRIX_RESIZE(pLinearJacobian,3,MAL_MATRIX_NB_COLS(pJacobian));
      MAL_MATRIX_C_eq_EXTRACT_A(pLinearJacobian,pJacobian,double,0,0,3,
				MAL_MATRIX_NB_COLS(pJacobian));
      ODEBUG("pLinearJacobian:" <<endl <<pLinearJacobian);

      matrixNxP pAngularJacobian; 
      MAL_MATRIX_RESIZE(pAngularJacobian,3,MAL_MATRIX_NB_COLS(pJacobian));
      MAL_MATRIX_C_eq_EXTRACT_A(pAngularJacobian,pJacobian,double,3,0,3,
				MAL_MATRIX_NB_COLS(pJacobian));

      ODEBUG("pAngularJacobian:" <<endl <<pAngularJacobian);

      // Used to compute the symmetric matrix.
      double lmasse = aBody->getMasse();
      matrixNxP leftoperand;
      MAL_C_eq_A_by_B(leftoperand,MAL_RET_TRANSPOSE(pLinearJacobian),pLinearJacobian);
      m_InertiaMatrix = m_InertiaMatrix + lmasse * leftoperand;
      
      matrixNxP rightoperand;
      matrix3d tmp2_3d,tmp2_3d2;
      matrixNxP tmp2,tmp3;
      MAL_MATRIX_RESIZE(tmp2,3,3);
      MAL_S3x3_C_eq_A_by_B(tmp2_3d,aBody->getInertie(),MAL_S3x3_RET_TRANSPOSE(aBody->R)); 
      MAL_S3x3_C_eq_A_by_B(tmp2_3d2,aBody->R,tmp2_3d); 

      for(unsigned int i=0;i<3;++i)
	for(unsigned int j=0;j<3;++j)
	  tmp2(i,j) = tmp2_3d2(i,j);

      MAL_C_eq_A_by_B(tmp3,tmp2,pAngularJacobian);
      MAL_C_eq_A_by_B(rightoperand,MAL_RET_TRANSPOSE(pAngularJacobian),tmp3);

      m_InertiaMatrix = m_InertiaMatrix + rightoperand;
    }
  
}

const matrixNxP & DynMultiBodyPrivate::inertiaMatrix() const
{
  return m_InertiaMatrix;
}

matrixNxP & DynMultiBodyPrivate::getInertiaMatrix() 
{
  return m_InertiaMatrix;
}
