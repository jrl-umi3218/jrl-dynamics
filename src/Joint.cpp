// -*- mode: c++; indent-tabs-mode: t; tab-width: 2; c-basic-offset: 2; -*-
#include "dynamicsJRLJapan/Joint.h"
#include "dynamicsJRLJapan/DynamicBody.h"

#define ODEBUG2(x)
#define ODEBUG3(x) cerr << "Joint :" << x << endl

#if 0
#define ODEBUG(x) cerr << "Joint :" <<  x << endl
#else
#define ODEBUG(x) 
#endif

#if 0

#define ODEBUG4(x,y) { ofstream DebugFile; DebugFile.open(y,ofstream::app); DebugFile << "WalkGenJRLIntegrate: " << x << endl; DebugFile.close();}
#define _DEBUG_4_ACTIVATED_ 1 
#else
#define ODEBUG4(x,y)
#endif

using namespace dynamicsJRLJapan;

Joint::Joint(int ltype, MAL_S3_VECTOR(,double) & laxe, 
	     float lquantite, MAL_S4x4_MATRIX(,double) & lpose):
  m_type(ltype),
  m_axe(laxe),
  m_quantity(lquantite),
  m_pose(lpose),
  m_FatherJoint(0),
  m_IDinVRML(-1)
{
  m_FromRootToThis.push_back(this);
  CreateLimitsArray();
}

Joint::Joint(int ltype, MAL_S3_VECTOR(,double) & laxe, 
	     float lquantite, MAL_S3_VECTOR(,double) & translationStatic):
  m_type(ltype),
  m_axe(laxe),
  m_quantity(lquantite),
  m_FatherJoint(0),
  m_IDinVRML(-1)

{
  MAL_S4x4_MATRIX_SET_IDENTITY(m_pose);
  MAL_S4x4_MATRIX_ACCESS_I_J(m_pose,0,3) = translationStatic[0];
  MAL_S4x4_MATRIX_ACCESS_I_J(m_pose,1,3) = translationStatic[1];
  MAL_S4x4_MATRIX_ACCESS_I_J(m_pose,2,3) = translationStatic[2];
  m_FromRootToThis.push_back(this);

  CreateLimitsArray();
}

Joint::Joint(int ltype, MAL_S3_VECTOR(,double) & laxe, 
	     float lquantite):
  m_type(ltype),
  m_axe(laxe),
  m_quantity(lquantite),
  m_FatherJoint(0),
  m_IDinVRML(-1)
{
  MAL_S4x4_MATRIX_SET_IDENTITY(m_pose);
  m_FromRootToThis.push_back(this);
  CreateLimitsArray();
}

Joint::Joint(const Joint &r)
{
  m_type = r.type();
  m_axe = r.axe();
  m_quantity=r.quantity();
  m_pose=r.pose();
  m_FatherJoint = 0;
  m_Name=r.getName();
  m_IDinVRML=r.getIDinVRML();
  m_FromRootToThis.push_back(this);

  CreateLimitsArray();

  for(int i=0;i<numberDof();i++)
    {
      m_LowerLimits[i] = r.getJointLLimit(i);
      m_UpperLimits[i] = r.getJointULimit(i);
    }

}

Joint::Joint():  
  m_quantity(0.0),
  m_FatherJoint(0),
  m_IDinVRML(-1)
  
{
  MAL_S3_VECTOR_ACCESS(m_axe,0) = 0.0;
  MAL_S3_VECTOR_ACCESS(m_axe,1) = 0.0;
  MAL_S3_VECTOR_ACCESS(m_axe,2) = 0.0;
  MAL_S4x4_MATRIX_SET_IDENTITY(m_pose);

  m_type = FREE_JOINT;
  m_FromRootToThis.push_back(this);
  CreateLimitsArray();
}

Joint::~Joint() 
{
  if (m_LowerLimits!=0)
    delete m_LowerLimits;
  
  if (m_UpperLimits!=0)
    delete m_UpperLimits;
}

void Joint::CreateLimitsArray()
{
  m_LowerLimits = new double[numberDof()];
  m_UpperLimits = new double[numberDof()];
}

Joint & Joint::operator=(const Joint & r) 
{
  m_type = r.type();
  m_axe = r.axe();
  m_quantity=r.quantity();
  m_pose=r.pose();
  m_Name = r.getName();
  m_IDinVRML = r.getIDinVRML();
  CreateLimitsArray();
  for(int i=0;i<numberDof();i++)
    {
      m_LowerLimits[i] = r.getJointLLimit(i);
      m_UpperLimits[i] = r.getJointULimit(i);
    }
  return *this;
};


/***********************************************/
/* Implementation of the generic JRL interface */
/***********************************************/

CjrlJoint* Joint::parentJoint() const
{
  return m_FatherJoint;
}

bool Joint::addChildJoint(CjrlJoint& aJoint)
{
  Joint * pjoint = (Joint *)&aJoint;
  m_Children.push_back(pjoint);
  return true;
}

unsigned int Joint::countChildJoints() const
{
  return m_Children.size();
}

CjrlJoint* Joint::childJoint(unsigned int givenRank) const
{
  if ((givenRank>=0) && (givenRank<m_Children.size()))
    return m_Children[givenRank];
  
  return 0; // Previously return m_Children[0] (potential problem)
}

std::vector<CjrlJoint*> Joint::jointsFromRootToThis() const 
{
  return m_FromRootToThis;
}


const MAL_S4x4_MATRIX(,double) & Joint::currentTransformation() const
{
  MAL_S4x4_MATRIX(,double) * A = new MAL_S4x4_MATRIX(,double);

  MAL_S4x4_MATRIX_SET_IDENTITY((*A));

  if (m_Body!=0)
    {
      DynamicBody *m_DBody = (DynamicBody *) m_Body;

      for( unsigned int i=0;i<3;i++)
	for(unsigned int j=0;j<3;j++)
	  MAL_S4x4_MATRIX_ACCESS_I_J((*A),i,j) = m_DBody->R(i,j);
      
      for( unsigned int i=0;i<3;i++)
	MAL_S4x4_MATRIX_ACCESS_I_J((*A),i,3) = m_DBody->p(i);
    }
  return *A;
}

CjrlRigidVelocity Joint::jointVelocity()
{
  
    DynamicBody *m_DBody = dynamic_cast<DynamicBody *>(m_Body);
    CjrlRigidVelocity ajrlRV(m_DBody->v0,m_DBody->w);
    return ajrlRV;
}

CjrlRigidAcceleration Joint::jointAcceleration()
{
  // TODO : Update the member of this object
  // TODO : when calling ForwardDynamics.
  // TODO : This will avoid the dynamic cast.
  MAL_S3_VECTOR(,double) a,b;

  if (m_Body!=0)
    {
      DynamicBody *m_DBody = dynamic_cast<DynamicBody *>(m_Body);

      a = m_DBody->dv;
      b = m_DBody->dw;
    }
  CjrlRigidAcceleration ajrlRA(a,b);
  
  return ajrlRA;

}

unsigned int Joint::numberDof() const
{
  unsigned int r=0;

  switch(m_type)
    {
    case (FREE_JOINT):
      r=6;
      break;
    case (FIX_JOINT):
      r=0;
      break;
    case (REVOLUTE_JOINT):
      r=1;
      break;
    case (PRISMATIC_JOINT):
      r=1;
      break;
    }
  return r;
}

const MAL_MATRIX(,double) & Joint::jacobianJointWrtConfig() const
{  
  return m_J;
}

void Joint::resizeJacobianJointWrtConfig(int lNbDofs)
{
  MAL_MATRIX_RESIZE(m_J,6,lNbDofs);
  MAL_MATRIX_FILL(m_J,0.0);

}

void Joint::computeJacobianJointWrtConfig()
{
  DynamicBody * FinalBody = (DynamicBody *)m_Body;
  getJacobianWorldPointWrtConfig(FinalBody->p, m_J);
}

void Joint::getJacobianWorldPointWrtConfig(const vector3d& inPointWorldFrame,
																					 matrixNxP& outJ) const
{
  vector3d aRa,dp,lv;

  ODEBUG("Size of the jacobian :" << m_FromRootToThis.size()-1);
   
  for(int i=0;i<m_FromRootToThis.size();i++)
    {
      MAL_VECTOR_DIM(LinearAndAngularVelocity,double,6);

      DynamicBody * aBody=  (DynamicBody *) m_FromRootToThis[i]->linkedBody();
      Joint * aJoint = (Joint *)m_FromRootToThis[i];
      
      MAL_S3x3_C_eq_A_by_B(aRa,aBody->R, aBody->a);

      unsigned int lcol = aJoint->stateVectorPosition();
      ODEBUG("Joint: " << aJoint->getName() << " " << lcol);
      dp = (vector3d)inPointWorldFrame - aBody->p;

      MAL_S3_VECTOR_CROSS_PRODUCT(lv,aRa,dp);
      
      switch (aJoint->type())
	{
	  
	case Joint::REVOLUTE_JOINT:
	  for(int j=0;j<3;j++)
	    {
	      outJ(j,lcol) =  lv[j];
	      outJ(j+3,lcol) = aRa[j];
	    }
	  break;
	case Joint::PRISMATIC_JOINT:
	  for(int j=0;j<3;j++)
	    {
	      outJ(j,lcol) =  aRa[j];
	      outJ(j+3,lcol) = 0;
	    }
	  break;
	case Joint::FREE_JOINT:
	  //J =  I M = J11 J12
	  //     0 I   J21 J22
	  //
	  // with M = d(w x dp)/dw
	  //
	  for(int j=0;j<3;j++)
	    {
	      for(int k=0;k<3;k++)
		{
		  // Computation of J11, J12 and J21
		  if (j!=k)
		    {
		      outJ(     j, lcol + k) =0.0;
		      outJ( j + 3, lcol + k + 3) = 0.0;
		    }
		  else
		    { 
		      outJ(     j, lcol + k ) =1.0;
		      outJ( j + 3, lcol + k + 3) = 1.0;
		    }
		  outJ(j+3,k) = 0.0;
		}
	    }
	  // Compute M
	  outJ( 0, lcol + 3 ) =      0; outJ( 0 , lcol + 4 ) =   dp(2); outJ( 0 , lcol + 5 ) = -dp(1);
	  outJ( 1, lcol + 3 ) = -dp(2); outJ( 1 , lcol + 4 ) =      0 ; outJ( 1 , lcol + 5 ) =  dp(0);
	  outJ( 2, lcol + 3 ) =  dp(1); outJ( 2 , lcol + 4 ) =  -dp(0); outJ( 2 , lcol + 5 ) =      0;
	  break;
	}
    }
}

/**
        \brief Get the jacobian of the point specified in local frame by inPointJointFrame.
    The output matrix outjacobian is automatically resized if necessary

 */
void Joint::getJacobianPointWrtConfig(const vector3d& inPointJointFrame, matrixNxP& outJ) const
{
	if (outJ.size1() !=6 || outJ.size2() != m_J.size2())
    {
			outJ.resize(6,m_J.size2(),false);
    }
	outJ.clear();
    

	DynamicBody * FinalBody = (DynamicBody *)m_Body;
  
	vector3d pn = FinalBody->p + MAL_S3x3_RET_A_by_B(FinalBody->R, inPointJointFrame);
	getJacobianWorldPointWrtConfig(pn, outJ);
}

    
CjrlBody* Joint::linkedBody() const
{
  return m_Body;
}

void Joint::setLinkedBody(CjrlBody& inBody)
{
  m_Body = &inBody;
}

void Joint::SetFatherJoint(Joint *aFather)
{
  m_FatherJoint = aFather;

  m_FromRootToThis.clear();
  
  m_FromRootToThis.push_back(this);

  CjrlJoint* aJoint = m_FatherJoint;
  while(aJoint!=0)
    {
      m_FromRootToThis.insert(m_FromRootToThis.begin(),aJoint);
      aJoint = aJoint->parentJoint();
    }
}

const MAL_S4x4_MATRIX(,double) & Joint::initialPosition()
{
  if (m_Body!=0)
    {
      DynamicBody *aDB = (DynamicBody *) m_Body;
      ODEBUG("Joint Name " << m_Name << " " << m_Body);
      for(int i=0;i<3;i++)
	for(int j=0;j<3;j++)
	  MAL_S4x4_MATRIX_ACCESS_I_J(m_pose,i,j) = aDB->R(i,j);
      for(int i=0;i<3;i++)
	MAL_S4x4_MATRIX_ACCESS_I_J(m_pose,i,3) = aDB->p(i);
      ODEBUG( m_pose);

    }
  return m_pose;
}

void Joint::UpdatePoseFrom6DOFsVector(MAL_VECTOR(,double) a6DVector)
{
  // Update the orientation of the joint.
  // Takes the three euler joints 

  MAL_S4x4_MATRIX_ACCESS_I_J(m_pose,0,3) = a6DVector(0);
  MAL_S4x4_MATRIX_ACCESS_I_J(m_pose,1,3) = a6DVector(1);
  MAL_S4x4_MATRIX_ACCESS_I_J(m_pose,2,3) = a6DVector(2);  
  
  DynamicBody* body = dynamic_cast<DynamicBody*>(m_Body);
  body->p[0] = a6DVector(0);
  body->p[1] = a6DVector(1);
  body->p[2] = a6DVector(2);
  
  MAL_S3x3_MATRIX(,double) D,B,C,A;
  double CosTheta, SinTheta, 
    CosPhi, SinPhi,
    CosPsi, SinPsi;
  
  
  CosPsi = cos(a6DVector(3));
  SinPsi = sin(a6DVector(3));
  CosTheta = cos(a6DVector(4));
  SinTheta = sin(a6DVector(4));
  CosPhi = cos(a6DVector(5));
  SinPhi = sin(a6DVector(5));
  
  /*
  D(0,0) =       1; D(0,1) =        0; D(0,2) = 0;
  D(1,0) =       0; D(1,1) =   CosPsi; D(1,2) = -SinPsi;
  D(2,0) =       0; D(2,1) =   SinPsi; D(2,2) = CosPsi;
  
  C(0,0) =  CosTheta; C(0,1) =        0; C(0,2) = SinTheta;
  C(1,0) =         0; C(1,1) =        1; C(1,2) = 0;
  C(2,0) = -SinTheta; C(2,1) =        0; C(2,2) = CosTheta;
  
  B(0,0) =  CosPhi; B(0,1) = -SinPhi; B(0,2) = 0;
  B(1,0) =  SinPhi; B(1,1) = CosPhi;  B(1,2) = 0;
  B(2,0) =       0; B(2,1) =      0;  B(2,2) = 1;

  MAL_S3x3_MATRIX(,double) tmp;
  MAL_S3x3_C_eq_A_by_B(tmp,C,D);
  MAL_S3x3_C_eq_A_by_B(A,B,tmp);

  body->R = A;
  */
  
  //Formulae for the above commented rotation composition
  A(0,0) = CosTheta * CosPhi ;
  A(1,0) = CosTheta * SinPhi;
  A(2,0) = -SinTheta;

  A(0,1) = CosPhi * SinPsi * SinTheta - CosPsi * SinPhi;
  A(1,1) = CosPsi * CosPhi + SinPsi * SinTheta * SinPhi;
  A(2,1) = CosTheta * SinPsi;

  A(0,2) = CosPsi * CosPhi * SinTheta + SinPhi * SinPsi;
  A(1,2) = - CosPhi * SinPsi + CosPsi * SinTheta * SinPhi;
  A(2,2) = CosPsi * CosTheta;
  
  body->R = A;

  for(int i=0;i<3;i++)
    for(int j=0;j<3;j++)
      MAL_S4x4_MATRIX_ACCESS_I_J(m_pose,i,j) = A(i,j);
 
  ODEBUG("m_pose : " << m_pose << 
	  " A: "<<endl << A << 
	  " tmp " << endl << tmp <<
	  "C " << endl << C <<
	  "D " << endl << D << 
	  "B " << endl << B );
}

void Joint::UpdateVelocityFrom2x3DOFsVector(MAL_S3_VECTOR(,double) & aLinearVelocity,
					  MAL_S3_VECTOR(,double) & anAngularVelocity)
{
  m_RigidVelocity.linearVelocity(aLinearVelocity);
  m_RigidVelocity.rotationVelocity(anAngularVelocity);
}


void Joint::RodriguesRotation(vector3d& inAxis, double inAngle, matrix3d& outRotation)
{
    double norm_w = MAL_S3_VECTOR_NORM(inAxis);
    if (norm_w < 10e-7)
    {
        MAL_S3x3_MATRIX_SET_IDENTITY(outRotation);
    }
    else
    {
        double th = norm_w * inAngle;
        wn3d = inAxis / norm_w;
        double ct = cos(th);
        double lct= (1-ct);
        double st = sin(th);
        outRotation(0,0) = ct + wn3d[0]*wn3d[0]* lct;
        outRotation(0,1) = wn3d[0]*wn3d[1]*lct-wn3d[2]*st;
        outRotation(0,2) = wn3d[1] * st+wn3d[0]*wn3d[2]*lct;
        outRotation(1,0) = wn3d[2]*st +wn3d[0]*wn3d[1]*lct;
        outRotation(1,1) = ct + wn3d[1]*wn3d[1]*lct;
        outRotation(1,2) = -wn3d[0]*st+wn3d[1]*wn3d[2]*lct;
        outRotation(2,0) = -wn3d[1]*st+wn3d[0]*wn3d[2]*lct;
        outRotation(2,1) = wn3d[0]*st + wn3d[1]*wn3d[2]*lct;
        outRotation(2,2) = ct + wn3d[2]*wn3d[2]*lct;
    }
}

JointFreeflyer::JointFreeflyer(const MAL_S4x4_MATRIX(,double) &inInitialPosition)
{
  type(Joint::FREE_JOINT);
  pose(inInitialPosition);
  
}

bool JointFreeflyer::updateTransformation(const vectorN& inDofVector)
{
    if (rankInConfiguration()+5 > inDofVector.size() -1 )
    {
        std::cout << "JointFreeflyer::updateTransformation(). Inappropriate configuration vector.\n";
        return false;
    }
    
    if (dof6D.size() != 6)
        dof6D.resize(6,false);
    
    for (unsigned int i=0; i<6; i++)
        dof6D(i) = inDofVector(rankInConfiguration() + i);

    UpdatePoseFrom6DOFsVector(dof6D);
    
    return true;
}

JointRotation::JointRotation(const MAL_S4x4_MATRIX(,double) &inInitialPosition)
{
  type(Joint::REVOLUTE_JOINT);
  pose(inInitialPosition);
}

bool JointRotation::updateTransformation(const vectorN& inDofVector)
{
    if (rankInConfiguration() > inDofVector.size() -1 )
    {
        std::cout << "JointRotation::updateTransformation(). Inappropriate configuration vector .\n";
        return false;
    }
    
    DynamicBody* body = dynamic_cast<DynamicBody*>(linkedBody());
    DynamicBody* parentbody = dynamic_cast<DynamicBody*>(parentJoint()->linkedBody());

    body->q = inDofVector(rankInConfiguration());
    
    RodriguesRotation(body->a, body->q, localR);

    MAL_S3x3_C_eq_A_by_B(body->R ,parentbody->R , localR);
    body->p = parentbody->p + MAL_S3x3_RET_A_by_B(parentbody->R,body->b);

    return true;
}

JointTranslation::JointTranslation(const MAL_S4x4_MATRIX(,double) &inInitialPosition)
{
  type(PRISMATIC_JOINT);
  pose(inInitialPosition);
}

bool JointTranslation::updateTransformation(const vectorN& inDofVector)
{
    if (rankInConfiguration() > inDofVector.size() -1 )
    {
        std::cout << "JointTranslation::updateTransformation(). Inappropriate configuration vector.\n";
        return false;
    }
    DynamicBody* body = dynamic_cast<DynamicBody*>(linkedBody());
    DynamicBody* parentbody = dynamic_cast<DynamicBody*>(parentJoint()->linkedBody());
    
    body->q = inDofVector(rankInConfiguration());
    
    for (unsigned int i = 0; i<3; i++)
        vek[i] *= body->q;
    
    body->R = parentbody->R;
    
    MAL_S3x3_C_eq_A_by_B(wn3d, body->R, vek);

    body->p = parentbody->p+ MAL_S3x3_RET_A_by_B(parentbody->R,body->b) + wn3d;
}


/************************************************************************************/
//Note: the following switch-implementation of the updateTransformation() method should not exist. The robot construction must be reviwed to construct freeflyer, rotation or translation joints as such and not as generic Joints with a type-telling member. For now, this method compiles the code specific to each type of joint
/************************************************************************************/
bool Joint::updateTransformation(const vectorN& inDofVector)
{
    if (rankInConfiguration() > inDofVector.size() -1 )
    {
        std::cout << "JointTranslation::updateTransformation(). Inappropriate configuration vector.\n";
        return false;
    }
    
    switch (type())
    {
        case Joint::REVOLUTE_JOINT :
        {
            DynamicBody* body = (DynamicBody*)(linkedBody());
            DynamicBody* parentbody = (DynamicBody*)(parentJoint()->linkedBody());

            body->q = inDofVector(rankInConfiguration());
            quantity( body->q);
    
            RodriguesRotation(body->a, body->q, localR);

            MAL_S3x3_C_eq_A_by_B(body->R ,parentbody->R , localR);
            body->p = parentbody->p + MAL_S3x3_RET_A_by_B(parentbody->R,body->b);
        }
        break;
        
        case Joint::FREE_JOINT :
        {
            if (dof6D.size() != 6)
                dof6D.resize(6,false);
    
            for (unsigned int i=0; i<6; i++)
                dof6D(i) = inDofVector(rankInConfiguration() + i);

            UpdatePoseFrom6DOFsVector(dof6D);
        }
        break;
        
        case Joint::PRISMATIC_JOINT :
        {
            DynamicBody* body = (DynamicBody*)(linkedBody());
            DynamicBody* parentbody = (DynamicBody*)(parentJoint()->linkedBody());
    
            body->q = inDofVector(rankInConfiguration());
            quantity( body->q);
    
            for (unsigned int i = 0; i<3; i++)
                vek[i] *= body->q;
    
            body->R = parentbody->R;
    
            MAL_S3x3_C_eq_A_by_B(wn3d, body->R, vek);

            body->p = parentbody->p+ MAL_S3x3_RET_A_by_B(parentbody->R,body->b) + wn3d;
        }
        break;
    }
    
}
/************************************************************************************/
