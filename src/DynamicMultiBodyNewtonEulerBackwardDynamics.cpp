void DynamicMultiBody::BackwardDynamics(DynamicBody & CurrentBody )
{
  MAL_S3x3_MATRIX(,double) aRt;

  MAL_S3x3_MATRIX(,double) currentBodyR;
  currentBodyR = MAL_S3x3_RET_TRANSPOSE(CurrentBody.R);

  MAL_S3_VECTOR(,double) lg;
  lg(0) = 0.0;
  lg(1) = 0.0;
  lg(2) = -9.81;

  /* Compute the torque
   * with eq. (7.147) Spong RMC p. 277
   *
   *
   */
  MAL_S3_VECTOR(,double) firstterm,
    sndterm, thirdterm, fifthterm,tmp;
  // Do not fourth term because it is the angular acceleration.
  /* Constant part */
  /* 2nd term : -f_i x r_{i,ci} */
  MAL_S3_VECTOR_CROSS_PRODUCT(sndterm,CurrentBody.m_Force, CurrentBody.c);

  /* 5th term : w_i x (I_i w_i)*/
  MAL_S3x3_MATRIX(,double) lI = CurrentBody.getInertie();
  tmp = MAL_S3x3_RET_A_by_B(currentBodyR,CurrentBody.w);
  tmp = MAL_S3x3_RET_A_by_B(lI,tmp);
  MAL_S3_VECTOR_CROSS_PRODUCT(fifthterm,CurrentBody.w,tmp);

  CurrentBody.m_Torque =  MAL_S3x3_RET_A_by_B(currentBodyR,CurrentBody.dw) + fifthterm -sndterm;

  /* Compute with the force
   * eq. (7.146) Spong RMC p. 277
   * fi = R^i_{i+1} * f_{i+1} + m_i * a_{c,i} - m_i * g_i
   * g_i is the gravity express in the i reference frame.
   */

  /* Constant part. */
  tmp = CurrentBody.dv_c - lg;
  tmp = MAL_S3x3_RET_A_by_B(currentBodyR,tmp);
  CurrentBody.m_Force =  tmp * CurrentBody.mass();

  int IndexChild = CurrentBody.child;
  //cout << "Body : " << CurrentBody.getName() << endl;
  while(IndexChild!=-1)
    {
      DynamicBody *Child = m_listOfBodies[IndexChild];
      //cout << "Child Bodies : " << Child->getName() << endl;
      aRt = Child->Riip1;
      //cout << "Riip1: " << aRt << endl;
      // /* Force computation. */
      //// Other immediate child are sisters of the other immediate childs.
      //cout << "Force: " << Child->m_Force << endl;
      tmp= MAL_S3x3_RET_A_by_B(aRt, Child->m_Force);
      CurrentBody.m_Force += tmp;

      /* Torque computation. */
      /* 1st term : R^i_{i+1} t_{i+1} */
      firstterm = MAL_S3x3_RET_A_by_B(aRt, Child->m_Torque);

      /* 3rd term : R_i_{i+1} f_{i+1} */
      MAL_S3_VECTOR_CROSS_PRODUCT(thirdterm,tmp, CurrentBody.w_c);

      CurrentBody.m_Torque += firstterm + thirdterm;

      /* Body selection. */
      IndexChild = m_listOfBodies[IndexChild]->sister;
      if (IndexChild!=-1)
	Child=m_listOfBodies[IndexChild];
    }

  // Update the vector related to the computed quantities.
  for(unsigned int i=0;i<m_StateVectorToJoint.size();i++)
    {
      unsigned int StateRankComputed=false;

      Joint * aJoint = (Joint *)m_JointVector[m_StateVectorToJoint[i]];
      if (aJoint!=0)
        {
	  DynamicBody *aDB = (DynamicBody *) aJoint->linkedBody();
	  if (aDB!=0)
            {
	      StateRankComputed = true;
	      for(unsigned int k=0;k<3;k++)
                {
		  m_Forces(i,k)=aDB->m_Force[k];
		  m_Torques(i,k) = aDB->m_Torque[k];
                }

            }
        }

      if (!StateRankComputed)
        {
	  for(unsigned int k=0;k<3;k++)
            {
	      m_Forces(i,k)=0.0;
	      m_Torques(i,k)=0.0;
            }
        }
    }
}
