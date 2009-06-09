/* @doc Computation of the dynamic aspect for a robot.
   This class will load the description of a robot from a VRML file
   following the OpenHRP syntax. Using ForwardVelocity it is then
   possible specifying the angular velocity and the angular value 
   to get the absolute position, and absolute velocity of each 
   body separetly. Heavy rewriting from the original source
   of Adrien and Jean-Remy. 
 
   This implantation is an updated based on a mixture between 
   the code provided by Jean-Remy and Adrien.
 
   Copyright (c) 2005-2006, 
   @author Olivier Stasse, Ramzi Sellouati, Jean-Remy Chardonnet, Adrien Escande, Abderrahmane Kheddar
   Copyright (c) 2007-2009
   @author Olivier Stasse, Oussama Kannoun, Fumio Kanehiro.
   JRL-Japan, CNRS/AIST
 
   All rights reserved.

   Please refers to file License.txt for details on the license.

*/

/*! System includes */
#include <fstream>
#include <string.h>

/*! Local library includes. */
#include "dynamicsJRLJapan/DynamicMultiBody.h"
#include "robotDynamics/jrlBody.h"

#define RESETDEBUG5(y) { ofstream DebugFile;	\
    DebugFile.open(y,ofstream::out);		\
    DebugFile.close();}

#define ODEBUG5(x,y) { ofstream DebugFile;	\
    DebugFile.open(y,ofstream::app);		\
    DebugFile << "DMB: " << x << endl;		\
    DebugFile.close();}

#define ODEBUG2(x)
#define ODEBUG3(x) cerr << "DynamicMultiBody :" << x << endl

#if 0
#define ODEBUG(x) cerr << "DynamicMultiBody :" <<  x << endl
#else
#define ODEBUG(x)
#endif

#if 0

#define ODEBUG4(x,y) { ofstream DebugFile; \
    DebugFile.open(y,ofstream::app); \
    DebugFile << "WalkGenJRLIntegrate: " \
	      << x << endl; DebugFile.close();}
#define _DEBUG_4_ACTIVATED_ 1
#else
#define ODEBUG4(x,y)
#endif

#if 0
#define RESETDEBUG4(y) { ofstream DebugFile; DebugFile.open(y,ofstream::out); DebugFile.close();}
#define ODEBUG4(x,y) { ofstream DebugFile; DebugFile.open(y,ofstream::app); \
    DebugFile << "PGI: " << x << endl; DebugFile.close();}
#define _DEBUG_4_ACTIVATED_ 1
#else
#define RESETDEBUG4(y)
#define ODEBUG4(x,y)
#endif

using namespace dynamicsJRLJapan;

DynamicMultiBody::DynamicMultiBody()
{
  m_NbDofs=0;
  m_ComputeVelocity = true;
  m_ComputeCoM = true;
  m_ComputeMomentum = true;
  m_ComputeZMP = false;
  m_ComputeAcceleration=false;
  m_ComputeAccCoM = false;
  m_ComputeBackwardDynamics=false;
  m_ComputeSkewCoM = true;
  m_IterationNumber = 0;
  m_TimeStep = 0.005;
  m_FileLinkJointRank="";

  labelTheRoot=1;

  // Create a default Root tree which
  // is a free joint.
  MAL_S3_VECTOR(,double) anAxis;

  m_RootOfTheJointsTree = 0;

  m_Prev_P(0) =   m_Prev_P(1) =   m_Prev_P(2) = 0.0;
  m_Prev_L(0) =   m_Prev_L(1) =   m_Prev_L(2) = 0.0;

  m_NbOfVRMLIDs = -1;
  RESETDEBUG4("DebugDataPL.dat");
  RESETDEBUG4("DebugDataZMP.dat");
  RESETDEBUG4("DebugDataDMB_ZMP.dat");
}

DynamicMultiBody::~DynamicMultiBody()
{
  for(unsigned int li=0; li<listOfBodies.size();li++)
    {
      Body *body = listOfBodies[li];
      std::vector<Body *>::iterator it;
      for (it = listeCorps.begin(); it != listeCorps.end(); it++)
        {
	  if (*it == body)
            {
	      listeCorps.erase(it);
	      break;
            }
        }
      delete listOfBodies[li];
    }
}

void DynamicMultiBody::SpecifyTheRootLabel(int ID)
{
  labelTheRoot = ID;
  listOfBodies[ID]->setLabelMother(-1);

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
  m_RootOfTheJointsTree->setLinkedBody(*listOfBodies[ID]);
  //specify the type of the root joint

  // Start the vector of joints.
  m_JointVector.clear();
  m_JointVector.insert(m_JointVector.end(),m_RootOfTheJointsTree);
  int lVRMLID = m_RootOfTheJointsTree->getIDinVRML();
  if (m_NbOfVRMLIDs < lVRMLID)
    m_NbOfVRMLIDs = lVRMLID;

  // Find out the next body to be examine.
  ReLabelling(ID,ld);

  // Once finished we initialize the child and the sister.
  for(unsigned int i=0;i<listOfBodies.size();i++)
    {
      int lMother,lElderSister;
      if ((lMother=listOfBodies[i]->getLabelMother()) != -1)
        {

	  if ((lElderSister=listOfBodies[lMother]->child) == -1)
            {
	      listOfBodies[lMother]->child = i;					  // Mother, I am your daughter !

            }
	  else
            {
	      // I have an elder sister !

	      while (listOfBodies[lElderSister]->sister != -1)
		lElderSister = listOfBodies[lElderSister]->sister;  // I have another elder sister !

	      listOfBodies[lElderSister]->sister = i;				  // I am your younger sister !
            }
        }
    }

  for (unsigned int i = 0; i< listOfBodies.size();i++)
    listOfBodies[i]->massCoef(listOfBodies[i]->mass()/mass());
}

void DynamicMultiBody::UpdateBodyParametersFromJoint(int BodyID, int JointID, int LiaisonForFatherJoint)
// cID : corps identifier
// lD : liaison destination
{

  ODEBUG( "Update body :" << BodyID << " from Joint " << JointID);
  // Update the rotation axis.
  listOfBodies[BodyID]->a =  listeLiaisons[JointID].aJoint->axe();
  ODEBUG(" axe: " << listOfBodies[BodyID]->a);
  // Update the translation vector
  listeLiaisons[JointID].aJoint->getStaticTranslation(listOfBodies[BodyID]->b);
  ODEBUG(" JointID: " << JointID << "BodyID: " << BodyID << ", static translation: " << listOfBodies[BodyID]->b);
  // Update the rotation matrix
  listeLiaisons[JointID].aJoint->getStaticRotation(listOfBodies[BodyID]->R_static);
  ODEBUG(" Rotation matrix: " << endl << listOfBodies[BodyID]->R_static);
  listeLiaisons[JointID].aJoint->setLinkedBody(*listOfBodies[BodyID]);
  listOfBodies[BodyID]->joint( listeLiaisons[JointID].aJoint);

}

void DynamicMultiBody::ReLabelling(int corpsCourant, int liaisonDeProvenance)
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
      listOfBodies[corpsSon]->setLabelMother(corpsMother);

      if(listeLiaisons[liaisonDestination].aJoint->getIDinVRML()!=-1)
        {


	  // Update the connections between the Joints.
	  int lIDinVRML = listeLiaisons[liaisonDestination].aJoint->getIDinVRML();
	  if (lIDinVRML!=-1)
            {
	      ConvertIDINVRMLToBodyID[lIDinVRML] = corpsSon;
            }


	  listeLiaisons[liaisonDeProvenance].aJoint->addChildJoint(*listeLiaisons[liaisonDestination].aJoint);

	  listeLiaisons[liaisonDestination].aJoint->SetFatherJoint(listeLiaisons[liaisonDeProvenance].aJoint);


	  // Update the vector of joints.
	  ODEBUG("Inside Relabelling :" << listeLiaisons[liaisonDestination].aJoint->getName().c_str());
	  ODEBUG("JointRankFromName :" << JointRankFromName(listeLiaisons[liaisonDestination].aJoint));
	  if (JointRankFromName(listeLiaisons[liaisonDestination].aJoint)!=-1)
	    m_JointVector.insert(m_JointVector.end(),listeLiaisons[liaisonDestination].aJoint);

	  int lVRMLID = listeLiaisons[liaisonDestination].aJoint->getIDinVRML();
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
      DynamicBody *Child = listOfBodies[IndexChild];
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
      IndexChild = listOfBodies[IndexChild]->sister;
      if (IndexChild!=-1)
	Child=listOfBodies[IndexChild];
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


/*! Kept for backward compatibility. */
void DynamicMultiBody::ForwardVelocity(MAL_S3_VECTOR(&PosForRoot,double),
                                       MAL_S3x3_MATRIX(&OrientationForRoot,double),
                                       MAL_S3_VECTOR(&v0ForRoot,double),
                                       MAL_S3_VECTOR(&wForRoot,double),
                                       MAL_S3_VECTOR(&dvForRoot,double),
                                       MAL_S3_VECTOR(&dwForRoot,double))
{
  NewtonEulerAlgorithm(PosForRoot,OrientationForRoot,v0ForRoot,wForRoot,dvForRoot,dwForRoot);
}


/*! Implementation of the Newton-Euler algorithm */
void DynamicMultiBody::NewtonEulerAlgorithm(MAL_S3_VECTOR(&PosForRoot,double),
					    MAL_S3x3_MATRIX(&OrientationForRoot,double),
					    MAL_S3_VECTOR(&v0ForRoot,double),
					    MAL_S3_VECTOR(&wForRoot,double),
					    MAL_S3_VECTOR(&dvForRoot,double),
					    MAL_S3_VECTOR(&dwForRoot,double))
{
  /** Intermediate variables. The mantra is :
      "To optimize those variables, in the Compiler we trust"
      (with the appropriate compilation options).
  */
  vector3d NE_tmp3, NE_tmp2, NE_wn,NE_cl, NE_lw_c, NE_aRc, NE_aRb,  
    NE_lpComP, NE_RotByMotherdv, NE_lP,NE_lL, NE_tmp;
  matrix3d NE_Rtmp, NE_Rt, NE_Ro, NE_Rot;
  /* End of intermediate */

  double norm_w, th;
  int currentNode = labelTheRoot;
  int lMother=0;
  unsigned int i,j;
  double NORME_EPSILON=1e-6;
  DynamicBody *aDB;

  listOfBodies[labelTheRoot]->p = PosForRoot;
  listOfBodies[labelTheRoot]->v0 = v0ForRoot;
  listOfBodies[labelTheRoot]->R = OrientationForRoot;
  listOfBodies[labelTheRoot]->w = wForRoot;
  listOfBodies[labelTheRoot]->dv = dvForRoot;
  listOfBodies[labelTheRoot]->dw = dwForRoot;

  currentNode = listOfBodies[labelTheRoot]->child;
  //  cout << "STARTING FORWARD VELOCITY " << v0ForRoot << endl;

  if (m_ComputeMomentum)
    {
      MAL_S3_VECTOR_FILL(m_P,0);
      MAL_S3_VECTOR_FILL(m_L,0);
    }

  positionCoMPondere[0] = 0;
  positionCoMPondere[1] = 0;
  positionCoMPondere[2] = 0;

  ODEBUG("PosForRoot: " << PosForRoot );
  ODEBUG("v0ForRoot: " << v0ForRoot );
  ODEBUG("OrientationForRoot: " << OrientationForRoot );
  do
    {

      aDB = listOfBodies[currentNode];

      norm_w = MAL_S3_VECTOR_NORM(aDB->a);
      lMother = aDB->getLabelMother();

      ODEBUG("CurrentBody " << listOfBodies[currentNode]->getName());

      // ----------------------------------
      // Rodrigues formula. (p33)
      if (norm_w< NORME_EPSILON)
        {
	  MAL_S3x3_MATRIX_SET_IDENTITY(NE_Ro);
        }
      else
        {
	  th = norm_w * aDB->q;
	  NE_wn = aDB->a / norm_w;
	  ODEBUG("aDB->a :" << aDB->a );
	  ODEBUG("norm_w:" << norm_w);

	  double ct = cos(th);
	  double lct= (1-ct);
	  double st = sin(th);
	  NE_Ro(0,0) = ct + NE_wn[0]*NE_wn[0]* lct;
	  NE_Ro(0,1) = NE_wn[0]*NE_wn[1]*lct-NE_wn[2]*st;
	  NE_Ro(0,2) = NE_wn[1] * st+NE_wn[0]*NE_wn[2]*lct;
	  NE_Ro(1,0) = NE_wn[2]*st +NE_wn[0]*NE_wn[1]*lct;
	  NE_Ro(1,1) = ct + NE_wn[1]*NE_wn[1]*lct;
	  NE_Ro(1,2) = -NE_wn[0]*st+NE_wn[1]*NE_wn[2]*lct;
	  NE_Ro(2,0) = -NE_wn[1]*st+NE_wn[0]*NE_wn[2]*lct;
	  NE_Ro(2,1) = NE_wn[0]*st + NE_wn[1]*NE_wn[2]*lct;
	  NE_Ro(2,2) = ct + NE_wn[2]*NE_wn[2]*lct;
        }

      ODEBUG("Ro:" << endl << NE_Ro );
      ODEBUG("MR:" << listOfBodies[lMother]->R );
      ODEBUG("b: " << aDB->b);
      ODEBUG("Mp: " << listOfBodies[lMother]->p);
      // End Rodrigues formula
      //-------------------------------

      // Position and orientation in reference frame
      listOfBodies[currentNode]->p = 
	MAL_S3x3_RET_A_by_B(listOfBodies[lMother]->R , aDB->b )
	+ listOfBodies[lMother]->p;
      MAL_S3x3_C_eq_A_by_B(NE_Rtmp ,listOfBodies[lMother]->R , 
			   listOfBodies[currentNode]->R_static);
      MAL_S3x3_C_eq_A_by_B(listOfBodies[currentNode]->R ,NE_Rtmp , NE_Ro);
      listOfBodies[currentNode]->Riip1 = NE_Ro;

      for( i=0;i<3;i++)
	for( j=0;j<3;j++)
	  MAL_S4x4_MATRIX_ACCESS_I_J(listOfBodies[currentNode]->m_transformation,i,j) 
	    = listOfBodies[currentNode]->R(i,j);

      for( i=0;i<3;i++)
	MAL_S4x4_MATRIX_ACCESS_I_J(listOfBodies[currentNode]->m_transformation,i,3) 
	  = listOfBodies[currentNode]->p(i);

      ODEBUG("q: "<< aDB->q );
      ODEBUG("p: "
	     << listOfBodies[currentNode]->p[0] << " "
	     << listOfBodies[currentNode]->p[1] << " "
	     << listOfBodies[currentNode]->p[2] << " " );
      ODEBUG("R: "<< aDB->R );

      //update the translation/rotation axis of joint
      MAL_S3x3_C_eq_A_by_B(listOfBodies[currentNode]->w_a,
			   listOfBodies[currentNode]->R, aDB->a);

      if (m_ComputeVelocity)
        {

	  ODEBUG("dq: "<< aDB->dq );
	  NE_tmp = listOfBodies[currentNode]->a * listOfBodies[currentNode]->dq;
	  NE_tmp = MAL_S3x3_RET_A_by_B(listOfBodies[currentNode]->R,NE_tmp);

	  listOfBodies[currentNode]->w  = listOfBodies[lMother]->w  + NE_tmp;

	  ODEBUG("w: " << listOfBodies[currentNode]->w );

	  // Computes the linear velocity.
	  MAL_S3x3_C_eq_A_by_B(NE_tmp,listOfBodies[lMother]->R,
			       listOfBodies[currentNode]->b);

	  MAL_S3_VECTOR_CROSS_PRODUCT(NE_tmp2,listOfBodies[lMother]->w , NE_tmp);

	  listOfBodies[currentNode]->v0 = listOfBodies[lMother]->v0 + NE_tmp2;

	  ODEBUG("v0: "
		 << listOfBodies[currentNode]->v0[0] << " "
		 << listOfBodies[currentNode]->v0[1] << " "
		 << listOfBodies[currentNode]->v0[2] << " " );
        }

      // Computes also the center of mass in the reference frame.
      if (m_ComputeCoM)
        {
	  ODEBUG("c: " << listOfBodies[currentNode]->c);
	  MAL_S3x3_C_eq_A_by_B(NE_cl,listOfBodies[currentNode]->R, 
			       listOfBodies[currentNode]->c);
	  NE_lw_c = NE_cl + listOfBodies[currentNode]->p;
	  ODEBUG("lw_c: "<<currentNode<<" "<< NE_lw_c[0] 
		 << " " << NE_lw_c[1] << " " << NE_lw_c[2]);
	  positionCoMPondere +=  
	    NE_lw_c * listOfBodies[currentNode]->getMasse();
	  ODEBUG("w_c: " << NE_lw_c[0] << " " 
		 << NE_lw_c[1] << " " << NE_lw_c[2]);
	  ODEBUG("Masse " << listOfBodies[currentNode]->getMasse());
	  ODEBUG("positionCoMPondere " << positionCoMPondere);
	  listOfBodies[currentNode]->w_c = NE_lw_c;
        }
      if (m_ComputeMomentum)
        {

	  // Computes momentum matrix P.
	  ODEBUG("w: " << listOfBodies[currentNode]->w );
	  MAL_S3_VECTOR_CROSS_PRODUCT(NE_tmp,listOfBodies[currentNode]->w, NE_cl);
	  ODEBUG("cl^w: " << NE_tmp);
	  ODEBUG("masse: " << listOfBodies[currentNode]->getMasse());
	  ODEBUG("v0: " << listOfBodies[currentNode]->v0 );
	  NE_lP=  (listOfBodies[currentNode]->v0 +
		   NE_tmp )* listOfBodies[currentNode]->getMasse();
	  listOfBodies[currentNode]->P = NE_lP;
	  ODEBUG("P: " << NE_lP );
	  m_P += NE_lP;

	  // Computes angular momentum matrix L
	  // Lk = xc x Pk + R * I * Rt * w
	  MAL_S3x3_TRANSPOSE_A_in_At(listOfBodies[currentNode]->R,NE_Rt);

	  MAL_S3_VECTOR_CROSS_PRODUCT(NE_tmp3,NE_lw_c,NE_lP);

	  MAL_S3x3_C_eq_A_by_B(NE_tmp2,NE_Rt , listOfBodies[currentNode]->w);
	  MAL_S3x3_C_eq_A_by_B(NE_tmp, listOfBodies[currentNode]->getInertie(),NE_tmp2);
	  MAL_S3x3_C_eq_A_by_B(NE_tmp2, listOfBodies[currentNode]->R,NE_tmp);
	  NE_lL = NE_tmp3 + NE_tmp2;
	  ODEBUG("L: " << lL);

	  listOfBodies[currentNode]->L = NE_lL;
	  m_L+= NE_lL;

        }

      if (m_ComputeAcceleration)
        {
	  // ******************* Computes the angular acceleration for joint i. ********************
	  // NE_tmp2 = z_{i-1} * dqi
	  NE_tmp2 = listOfBodies[currentNode]->w_a * listOfBodies[currentNode]->dq;
	  // NE_tmp3 = w^{(0)}_i x z_{i-1} * dqi
	  MAL_S3_VECTOR_CROSS_PRODUCT(NE_tmp3,listOfBodies[currentNode]->w,NE_tmp2);
	  // NE_tmp2 = z_{i-1} * ddqi
	  NE_tmp2 = listOfBodies[currentNode]->w_a * listOfBodies[currentNode]->ddq;
	  listOfBodies[currentNode]->dw = NE_tmp2 + NE_tmp3 + listOfBodies[lMother]->dw;

	  // ******************* Computes the linear acceleration for joint i. ********************
	  MAL_S3x3_C_eq_A_by_B(NE_aRb, listOfBodies[currentNode]->R, aDB->b);
	  // NE_tmp3 = w_i x (w_i x r_{i,i+1})
	  MAL_S3_VECTOR_CROSS_PRODUCT(NE_tmp2,listOfBodies[currentNode]->w,NE_aRb);
	  MAL_S3_VECTOR_CROSS_PRODUCT(NE_tmp3,listOfBodies[currentNode]->w,NE_tmp2);

	  // NE_tmp2 = dw_I x r_{i,i+1}
	  MAL_S3_VECTOR_CROSS_PRODUCT(NE_tmp2,listOfBodies[currentNode]->dw,NE_aRb);
	  NE_Rot = MAL_S3x3_RET_TRANSPOSE(NE_Ro);
	  MAL_S3x3_C_eq_A_by_B(NE_RotByMotherdv,NE_Rot,listOfBodies[lMother]->dv);
	  listOfBodies[currentNode]->dv = NE_RotByMotherdv + NE_tmp2 + NE_tmp3;
        }

      if (m_ComputeAccCoM)
        {

	  // *******************  Acceleration for the center of mass of body  i ************************
	  MAL_S3x3_C_eq_A_by_B(NE_aRc, listOfBodies[currentNode]->R, aDB->c);
	  MAL_S3_VECTOR_CROSS_PRODUCT(NE_tmp2,listOfBodies[currentNode]->w,NE_aRc);
	  MAL_S3_VECTOR_CROSS_PRODUCT(NE_tmp3,listOfBodies[currentNode]->w,NE_tmp2);

	  // NE_tmp2 = dw_I x r_{i,i+1}
	  MAL_S3_VECTOR_CROSS_PRODUCT(NE_tmp2,listOfBodies[currentNode]->dw,NE_aRc);
	  //
	  listOfBodies[currentNode]->dv_c = NE_RotByMotherdv + NE_tmp2 + NE_tmp3;
        }

      // TO DO if necessary : cross velocity.
      int step=0;
      int NextNode=0;
      do
        {

	  if (step==0)
            {
	      NextNode = listOfBodies[currentNode]->child;
	      step++;
            }
	  else if(step==1)
            {
	      NextNode = listOfBodies[currentNode]->sister;
	      step++;
            }
	  else if (step==2)
            {
	      NextNode = listOfBodies[currentNode]->getLabelMother();
	      if (NextNode>=0)
                {
		  /* Test if current node is leaf,
		     because in this case the force are not set properly. */
		  if (m_ComputeBackwardDynamics)
                    {
		      if ((listOfBodies[currentNode]->sister==-1) &&
			  (listOfBodies[currentNode]->child==-1))
			BackwardDynamics(*listOfBodies[currentNode]);

		      /* Compute backward dynamics */
		      BackwardDynamics(*listOfBodies[NextNode]);
                    }
		  currentNode = NextNode;
		  NextNode = listOfBodies[currentNode]->sister;
                }
	      else
		NextNode=labelTheRoot;
            }


        }
      while (NextNode==-1);
      currentNode = NextNode;

    }
  while(currentNode!=labelTheRoot);

  if (m_ComputeSkewCoM)
    {
      SkewCoM(0,0) =         0;
      SkewCoM(0,1) = - positionCoMPondere[2];
      SkewCoM(0,2) = positionCoMPondere[1];
      SkewCoM(1,0) = positionCoMPondere[2];
      SkewCoM(1,1) =           0;
      SkewCoM(1,2) =-positionCoMPondere[0];
      SkewCoM(2,0) =-positionCoMPondere[1];
      SkewCoM(2,1) =   positionCoMPondere[0];
      SkewCoM(2,2) =         0;
    }

  positionCoMPondere = positionCoMPondere/masse;
  // Zero Momentum Point Computation.
  if (m_ComputeZMP)
    {

      ODEBUG4( m_P << " " << m_L,"DebugDataPL.dat");
      // Update the momentum derivative
      if (m_IterationNumber>1)
        {
	  m_dP = (m_P - m_Prev_P)/m_TimeStep;
	  m_dL = (m_L - m_Prev_L)/m_TimeStep;

	  // Update the ZMP value.
	  double px,py,pz=0.0;
	  CalculateZMP(px,py,
		       m_dP, m_dL,pz);

	  m_ZMP(0) = px;
	  m_ZMP(1) = py;
	  m_ZMP(2) = pz;

	  ODEBUG4(m_ZMP<< " | " << m_dP << " | " << m_dL << "|" << m_IterationNumber,"DebugDataZMP.dat");

        }
      else
        {
	  m_ZMP = positionCoMPondere;
	  m_ZMP(2) = 0.0;
        }

      ODEBUG4( m_IterationNumber << " "
	       << m_ZMP(0) << " "
	       << m_ZMP(1) << " "
	       << m_ZMP(2) << " "
	       << m_P << " "
	       << m_L ,"DebugDataDMB_ZMP.dat" );

      // Update the store previous value.
      if (m_IterationNumber>=1)
        {
	  m_Prev_P = m_P;
	  m_Prev_L = m_L;
        }
    }


  m_IterationNumber++;
}




void DynamicMultiBody::ForwardDynamics(int corpsCourant, int liaisonDeProvenance)
{
  // Building the appropriate transformations.

  if (liaisonDeProvenance == 0)
    { 
      masse = 0;
      MAL_S3_VECTOR_FILL(positionCoMPondere,0);
    }

  masse += listeCorps[corpsCourant]->getMasse();

  switch (liaisons[corpsCourant].size())
    {
    case 1 :
      break;
    case 2 :
      if (liaisons[corpsCourant][0].liaison == liaisonDeProvenance)
        {
	  int liaisonDestination = liaisons[corpsCourant][1].liaison;
	  int corps1 = listeLiaisons[liaisonDestination].indexCorps1;
	  int corps2 = listeLiaisons[liaisonDestination].indexCorps2;
	  if (corpsCourant == corps1)
            {
	      ForwardDynamics(corps2, liaisonDestination);
            }
	  else
            {
	      ForwardDynamics(corps1, liaisonDestination);
            }
        }
      else
        {
	  int liaisonDestination = liaisons[corpsCourant][0].liaison;
	  int corps1 = listeLiaisons[liaisonDestination].indexCorps1;
	  int corps2 = listeLiaisons[liaisonDestination].indexCorps2;
	  if (corpsCourant == corps1)
            {
	      ForwardDynamics(corps2, liaisonDestination);
            }
	  else
            {
	      ForwardDynamics(corps1, liaisonDestination);
            }
        }
      break;
    default : //on a plus de deux liaisons
      for (unsigned int i=0; i<liaisons[corpsCourant].size(); i++)
        {
	  if (liaisons[corpsCourant][i].liaison == liaisonDeProvenance)
	    continue;
	  int liaisonDestination = liaisons[corpsCourant][i].liaison;
	  int corps1 = listeLiaisons[liaisonDestination].indexCorps1;
	  int corps2 = listeLiaisons[liaisonDestination].indexCorps2;
	  if (corpsCourant == corps1)
            {
	      ForwardDynamics(corps2, liaisonDestination);
            }
	  else
            {
	      ForwardDynamics(corps1, liaisonDestination);
            }
        }
      break;
    }
}


void DynamicMultiBody::CreatesTreeStructure(const char * option)
{
  listOfBodies.resize(listeCorps.size());
  DynamicBody *dbody;
  for(unsigned int i=0;i<listeCorps.size();i++)
    {
      dbody = dynamic_cast<DynamicBody *>(listeCorps[i]);
      if (!dbody)
        {
	  dbody = new DynamicBody();
	  *dbody = *listeCorps[i];
        }
      listOfBodies[i] = dbody;
    }

  if (option!=0)
    {
      ReadSpecificities(option);
    }

  ConvertIDINVRMLToBodyID.resize(listOfBodies.size());
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

bool DynamicMultiBody::initialize()
{
  setComputeZMP(true);
  InitializeFromJointsTree();
  unsigned int nbDof = numberDof();
  MAL_VECTOR_DIM(config, double, nbDof);
  MAL_VECTOR_FILL(config, 0);
  currentConfiguration(config);
  computeForwardKinematics();
  return true;
}

void DynamicMultiBody::InitializeFromJointsTree()
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
  int lIDinVRML=-1;
  unsigned int lRank=0;

  while(CurrentJoint!=0)
    {
      // Deal with the current joint.

      // Update the joint value of the current link.
      CurrentLink.aJoint = CurrentJoint;


      // Update the joint ID value in case there is none.
      if (CurrentLink.aJoint->getIDinVRML()==-1)
	CurrentLink.aJoint->setIDinVRML(lIDinVRML);

      lIDinVRML++;

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

void DynamicMultiBody::parserVRML(string path,
                                  string nom,
                                  const char *option)
{
  listOfBodies.clear();
  MultiBody::parserVRML(path, nom, option);
  CreatesTreeStructure(option);
}



double DynamicMultiBody::Getq(int JointID) const
{
  if ((JointID>=0) &&
      ((unsigned int)JointID<listOfBodies.size()))
    return listOfBodies[ConvertIDINVRMLToBodyID[JointID]]->q;
  return 0.0;
}

void DynamicMultiBody::Setq(int JointID,double q)
{
  if ((JointID>=0) &&
      ((unsigned int)JointID<listOfBodies.size()))
    {
      listOfBodies[ConvertIDINVRMLToBodyID[JointID]]->q= q;
      ((Joint *)listOfBodies[ConvertIDINVRMLToBodyID[JointID]]->joint())->quantity(q);
    }
}

void DynamicMultiBody::Setdq(int JointID, double dq)
{
  if ((JointID>=0) &&
      ((unsigned int)JointID<listOfBodies.size()))
    {
      listOfBodies[ConvertIDINVRMLToBodyID[JointID]]->dq= dq;
    }
}

void DynamicMultiBody::Setv(int JointID, MAL_S3_VECTOR(,double) v0)
{
  if ((JointID>=0) &&
      ((unsigned int)JointID<listOfBodies.size()))
    listOfBodies[ConvertIDINVRMLToBodyID[JointID]]->v0 = v0;
}

MAL_S3_VECTOR(,double) DynamicMultiBody::Getv(int JointID)
{
  if ((JointID>=0) &&
      ((unsigned int)JointID<listOfBodies.size()))
    return listOfBodies[ConvertIDINVRMLToBodyID[JointID]]->v0;

  MAL_S3_VECTOR(dr,double);
  dr[0] = 0.0;
  dr[1] = 0.0;
  dr[2] = 0.0;
  return dr;
}

MAL_S3_VECTOR(,double) DynamicMultiBody::GetvBody(int BodyID)
{
  if ((BodyID>=0) &&
      ((unsigned int)BodyID<listOfBodies.size()))
    return listOfBodies[BodyID]->v0;
  MAL_S3_VECTOR(dr,double);
  dr[0] = 0.0;
  dr[1] = 0.0;
  dr[2] = 0.0;
  return dr;
}

MAL_S3_VECTOR(,double) DynamicMultiBody::GetwBody(int BodyID)
{
  if ((BodyID>=0) &&
      ((unsigned int)BodyID<listOfBodies.size()))
    return listOfBodies[BodyID]->w;
  MAL_S3_VECTOR(dr,double);
  dr[0] = 0.0;
  dr[1] = 0.0;
  dr[2] = 0.0;
  return dr;
}

MAL_S3_VECTOR(,double) DynamicMultiBody::Getw(int JointID)
{
  if ((JointID>=0) &&
      ((unsigned int)JointID<listOfBodies.size()))
    return listOfBodies[ConvertIDINVRMLToBodyID[JointID]]->w;

  MAL_S3_VECTOR(dr,double);
  dr[0] = 0.0;
  dr[1] = 0.0;
  dr[2] = 0.0;
  return dr;
}

void DynamicMultiBody::Setw(int JointID, MAL_S3_VECTOR(,double) w)
{
  if ((JointID>=0) &&
      ((unsigned int)JointID<listOfBodies.size()))
    listOfBodies[ConvertIDINVRMLToBodyID[JointID]]->w = w;
}

MAL_S3_VECTOR(,double) DynamicMultiBody::Getp(int JointID)
{
  MAL_S3_VECTOR(empty,double);
  if ((JointID>=0) &&
      ((unsigned int)JointID<listOfBodies.size()))
    return listOfBodies[ConvertIDINVRMLToBodyID[JointID]]->p;
  return empty;
}

void DynamicMultiBody::Setp(int JointID, MAL_S3_VECTOR(,double) apos)
{
  if ((JointID>=0) &&
      ((unsigned int)JointID<listOfBodies.size()))
    listOfBodies[ConvertIDINVRMLToBodyID[JointID]]->p = apos;
}

MAL_S3_VECTOR(,double) DynamicMultiBody::getPositionCoM(void)
{
  return (positionCoMPondere);
}

void DynamicMultiBody::GetPandL(MAL_S3_VECTOR(,double) &aP, MAL_S3_VECTOR(,double) &aL)
{
  aP = m_P;
  aL = m_L;
}

void DynamicMultiBody::CalculateZMP(double &px, double &py,
                                    MAL_S3_VECTOR(,double) dP,
                                    MAL_S3_VECTOR(,double) dL,
                                    double zmpz)
{
  double g= 9.80665;

  px = ( g * positionCoMPondere[0]*masse +
	 zmpz * dP[0] - dL[1])/(masse * g + dP[2]);
  py = ( g * positionCoMPondere[1]*masse +
	 zmpz * dP[1] + dL[0])/(masse * g + dP[2]);

  ODEBUG(" CalculateZMP : Masse :"<< masse << " g:" << g  << " "
	 << dP << " " << dL << " " << positionCoMPondere );


}

string DynamicMultiBody::GetName(int JointID)
{
  string empty;
  if ((JointID>=0) &&
      ((unsigned int)JointID<listOfBodies.size()))
    return listOfBodies[ConvertIDINVRMLToBodyID[JointID]]->getName();
  return empty;

}

CjrlJoint* DynamicMultiBody::GetJointFromVRMLID(int JointID)
{

  for(unsigned int i=0;i<m_JointVector.size();i++)
    {
      Joint * r;
      if (((r=(Joint *)m_JointVector[i])->getIDinVRML())==JointID)
        {
	  ODEBUG("Joint : "<< r->getName() << " " << JointID );

	  return r;
        }
    }
  return 0;
}

MAL_S3_VECTOR(,double) DynamicMultiBody::GetL(int JointID)
{
  MAL_S3_VECTOR(empty,double);
  if ((JointID>=0) &&
      ((unsigned int)JointID<listOfBodies.size()))
    return listOfBodies[ConvertIDINVRMLToBodyID[JointID]]->L;
  return empty;
}

MAL_S3_VECTOR(,double) DynamicMultiBody::GetP(int JointID)
{
  MAL_S3_VECTOR(empty,double);
  if ((JointID>=0) &&
      ((unsigned int)JointID<listOfBodies.size()))
    return listOfBodies[ConvertIDINVRMLToBodyID[JointID]]->P;
  return empty;
}

double DynamicMultiBody::Getdq(int JointID) const
{
  double empty=0.0;
  if ((JointID>=0) &&
      ((unsigned int)JointID<listOfBodies.size()))
    return listOfBodies[ConvertIDINVRMLToBodyID[JointID]]->dq;
  return empty;
}



void DynamicMultiBody::SetRBody(int BodyID, MAL_S3x3_MATRIX(,double) R)
{
  if ((BodyID>=0) &&
      ((unsigned int)BodyID<listOfBodies.size()))
    listOfBodies[BodyID]->R = R;
}

/***********************************************/
/* Implementation of the generic JRL interface */
/***********************************************/

// Set the root joint of the robot.
void DynamicMultiBody::rootJoint(CjrlJoint& inJoint)
{
  m_RootOfTheJointsTree = & (Joint &)inJoint;

}

// Get the robot joint of the robot.
CjrlJoint* DynamicMultiBody::rootJoint() const
{
  return m_RootOfTheJointsTree;
}

// Get a vector containning all the joints
std::vector<CjrlJoint*> DynamicMultiBody::jointVector()
{
  return m_JointVector;
}

std::vector<CjrlJoint*> DynamicMultiBody::jointsBetween(const CjrlJoint& inStartJoint, 
							const CjrlJoint& inEndJoint) const
{
  std::vector<CjrlJoint*> outJoints;
  std::vector<CjrlJoint *> robotRoot2StartJoint, robotRoot2EndJoint;
  robotRoot2StartJoint = inStartJoint.jointsFromRootToThis();
  robotRoot2EndJoint = inEndJoint.jointsFromRootToThis();

  unsigned int i;
  unsigned int lastCommonJointRank = 0;
  unsigned int minChainLength = (robotRoot2StartJoint.size()<robotRoot2EndJoint.size())
    ?robotRoot2StartJoint.size():robotRoot2EndJoint.size();

  for(i=1; i< minChainLength; i++)
    {
      if ((robotRoot2StartJoint[i]==robotRoot2EndJoint[i]))
	lastCommonJointRank++;
    }

  for(i = robotRoot2StartJoint.size()-1; i>lastCommonJointRank; i--)
    outJoints.push_back(robotRoot2StartJoint[i]);

  for(i=lastCommonJointRank+1; i< robotRoot2EndJoint.size(); i++)
    outJoints.push_back(robotRoot2EndJoint[i]);

  return outJoints;
}

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
    delete ( outTable[i] );
  delete ( outTable );

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

bool DynamicMultiBody::getJacobianCenterOfMass ( const CjrlJoint& inStartJoint, 
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

  Joint* StartJoint = ( Joint* ) ( &inStartJoint );
  //determine participating joints
  if ( rootJoint() !=&inStartJoint )
    {
      std::vector<Joint *> robotRoot2StartJoint = StartJoint->jointsFromRootToThisJoint();

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
  Joint* aJoint;
  DynamicBody* aBody;

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
	case Joint::REVOLUTE_JOINT:
	  for ( j=0;j<3;j++ )
	    tempDP[j] = aJoint->subTreeMCom() [j]- aJoint->subTreeCoef() *aBody->p[j];
	  MAL_S3_VECTOR_CROSS_PRODUCT ( tempLV,aBody->w_a,tempDP );
	  for ( j=0;j<3;j++ )
	    if ( jointsigns[m_ConfigurationToJoints[i]->rankInConfiguration() ]>0 )
	      outTable[j][rank] =  tempLV[j];
	    else
	      outTable[j][rank] =  -tempLV[j];
	  break;
	case Joint::PRISMATIC_JOINT:
	  for ( j=0;j<3;j++ )
	    {
	      if ( jointsigns[m_ConfigurationToJoints[i]->rankInConfiguration() ]>0 )
		outTable[j][rank] = aBody->w_a[j];
	      else
		outTable[j][rank] = -aBody->w_a[j];
	    }
	  break;
	case Joint::FREE_JOINT:
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

void DynamicMultiBody::getJacobianLinearMomentumWrtCoM(matrixNxP &outjacobian)
{
  matrixNxP JCoM;
  getJacobianCenterOfMass(*rootJoint(),JCoM);
  outjacobian = masse * JCoM;
}

void DynamicMultiBody::getJacobianAngularMomentumWrtCoM(matrixNxP &outjacobian)
{
  if ((MAL_MATRIX_NB_ROWS(outjacobian) != 3) || 
      (MAL_MATRIX_NB_COLS(outjacobian) != numberDof()))
    MAL_MATRIX_RESIZE(outjacobian,3,numberDof());

  MAL_MATRIX_FILL(outjacobian,0);

  unsigned int rank;
  Joint* aJoint;
  DynamicBody* aBody;
    
  for(unsigned int i=0;i<m_ConfigurationToJoints.size();i++)
    {
      if (m_ConfigurationToJoints[i] == rootJoint())
	continue;

      aJoint = m_ConfigurationToJoints[i];
      aBody=  aJoint->linkedDBody();
      rank = aJoint->rankInConfiguration();
      
      matrixNxP pJacobian;
      vector3d av(0,0,0); // Dummy 
      MAL_MATRIX_RESIZE(pJacobian,6, numberDof());
      getJacobian(*rootJoint(),*aJoint,av,pJacobian);

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

      // Used to compute the anti-symmetric matrix.
      matrixNxP xkmxg_cp;double lmasse = aBody->getMasse();
      MAL_MATRIX_RESIZE(xkmxg_cp,3,3);
      av =aBody->w_c - positionCoMPondere;
      xkmxg_cp(0,0) =          0.0; xkmxg_cp(0,1) = -lmasse*av(2); xkmxg_cp(0,2) = lmasse*av(1);
      xkmxg_cp(1,0) = lmasse*av(2); xkmxg_cp(1,1) =           0.0; xkmxg_cp(1,2) =-lmasse*av(0);
      xkmxg_cp(2,0) =-lmasse*av(1); xkmxg_cp(2,1) =  lmasse*av(0); xkmxg_cp(2,2) =         0.0;

      ODEBUG("xkmxg_cp: " <<xkmxg_cp);

      matrixNxP leftoperand;
      MAL_C_eq_A_by_B(leftoperand,xkmxg_cp,pLinearJacobian);
      outjacobian = outjacobian + leftoperand;
      
      matrixNxP rightoperand;
      matrix3d tmp2_3d;
      matrixNxP tmp2;
      MAL_MATRIX_RESIZE(tmp2,3,3);
      MAL_S3x3_C_eq_A_by_B(tmp2_3d,aBody->R,aBody->getInertie()); 
      for(unsigned int i=0;i<3;++i)
	for(unsigned int j=0;j<3;++j)
	  tmp2(i,j) = tmp2_3d(i,j);

      MAL_C_eq_A_by_B(rightoperand,tmp2,pAngularJacobian);
      
      outjacobian = outjacobian + rightoperand;
    }
  
}

void DynamicMultiBody::ComputeNumberOfJoints()
{
  unsigned int r=0;
  ODEBUG("JointVector :" << m_JointVector.size());
  for(unsigned int i=0;i<m_JointVector.size();i++)
    {
      ODEBUG("Joint " << i << " : "
	     << m_JointVector[i]->numberDof() <<  " "
	     << ((Joint *)m_JointVector[i])->getIDinVRML() << " "
	     << ((Joint *)m_JointVector[i])->getName());
      r += m_JointVector[i]->numberDof();
    }

  if (r!=m_NbDofs)
    m_NbDofs=r;

  // Resize the jacobian of the CoM.
  MAL_MATRIX_RESIZE(m_JacobianOfTheCoM,3,m_NbDofs);
  MAL_MATRIX_RESIZE(m_attCalcJointJacobian,6,m_NbDofs);
}

void DynamicMultiBody::ReadSpecificities(string aFileName)
{
  FILE *fp;
  fp = fopen((char *)aFileName.c_str(),"r");

  if (fp==0)
    {
      cerr << "Unable to read " << aFileName << endl;
      return;
    }

  if (look_for(fp,"LinkJointNameAndRank"))
    {
      NameAndRank_t aNameAndRank;

      if (look_for(fp,"Nb"))
        {
	  fscanf(fp,"%d", &m_LinksBetweenJointNamesAndRankNb);
	  ODEBUG("Links: " << m_LinksBetweenJointNamesAndRankNb);
        }
      for(int j=0;j<m_LinksBetweenJointNamesAndRankNb;j++)
        {
	  if(look_for(fp,"Link"))
            {
	      char Buffer[128];
	      memset(Buffer,0,128);
	      fscanf(fp,"%s",Buffer);

	      strcpy(aNameAndRank.LinkName,Buffer);
	      fscanf(fp,"%d", &aNameAndRank.RankInConfiguration);

	      look_for(fp,"</Link>");
	      m_LinksBetweenJointNamesAndRank.insert(m_LinksBetweenJointNamesAndRank.end(),
						     aNameAndRank);
            }

	  ODEBUG( aNameAndRank.LinkName << " " << aNameAndRank.RankInConfiguration);
        }
      //     ODEBUG((char *)((Joint *)m_JointVector[0])->getName().c_str()
      // << " "  << m_JointVector[0]->rankInConfiguration());
    }
  fclose(fp);
}

int DynamicMultiBody::JointRankFromName(Joint *aJoint)
{

  ODEBUG("m_LinksBetweenJointNamesAndRank.size():" << m_LinksBetweenJointNamesAndRank.size());
  for(unsigned int i=0;i<m_LinksBetweenJointNamesAndRank.size();i++)
    {
      if (!strcmp(m_LinksBetweenJointNamesAndRank[i].LinkName,(char *)aJoint->getName().c_str()))
	return m_LinksBetweenJointNamesAndRank[i].RankInConfiguration;
    }
  return -1;
}

Joint * DynamicMultiBody::JointFromRank(int aRank)
{
  if (m_LinksBetweenJointNamesAndRank.size()!=0)
    {
      string JointName;
      for(unsigned int i=0;i<m_LinksBetweenJointNamesAndRank.size();i++)
        {
	  if (m_LinksBetweenJointNamesAndRank[i].RankInConfiguration==(unsigned int)aRank)
	    JointName = m_LinksBetweenJointNamesAndRank[i].LinkName;
        }
      for(unsigned int i=0;i<m_JointVector.size();i++)
        {
	  if (((Joint *)m_JointVector[i])->getName()==JointName)
	    return (Joint *)m_JointVector[i];
        }
    }

  int CurrentTestRank=0;

  for(unsigned int i=0;i<m_JointVector.size();i++)
    {
      int RankRangeBegin=CurrentTestRank;
      int RankRangeEnd=RankRangeBegin+((Joint *)m_JointVector[i])->numberDof();
      if((aRank>=RankRangeBegin) &&
	 (aRank<RankRangeEnd))
	return (Joint *)m_JointVector[i];
      CurrentTestRank=RankRangeEnd;
    }
  ODEBUG("Looking for rank " << aRank << " failed " << m_LinksBetweenJointNamesAndRank.size());
  return (Joint *)0;
}


void DynamicMultiBody::BuildStateVectorToJointAndDOFs()
{
  m_StateVectorToJoint.resize(m_NbDofs);
  int lindex=0,StateVectorIndexDefault=0;
  for(unsigned int i=0;i<m_JointVector.size();i++)
    {
      lindex = JointRankFromName((Joint *)m_JointVector[i]);
      if (lindex==-1)
	lindex = StateVectorIndexDefault;

      for(unsigned int j=0;j<m_JointVector[i]->numberDof();j++)
        {
	  m_StateVectorToJoint[lindex++]=i;
	  StateVectorIndexDefault++;
        }
    }


  m_StateVectorToDOFs.clear();
  m_StateVectorToDOFs.resize(m_NbDofs);
  lindex=0;
  StateVectorIndexDefault=0;
  for(unsigned int i=0;i<m_JointVector.size();i++)
    {
      lindex = JointRankFromName((Joint *)m_JointVector[i]);
      if (lindex==-1)
	lindex = StateVectorIndexDefault;

      ODEBUG(((Joint *)m_JointVector[i])->getName() << " " << lindex);
      for(unsigned int j=0;j<m_JointVector[i]->numberDof();j++)
        {
	  m_StateVectorToDOFs[lindex++]=j;
	  StateVectorIndexDefault++;
        }
    }

  m_VRMLIDToConfiguration.resize(m_NbOfVRMLIDs+1);
  for(unsigned int i=0;i<m_StateVectorToJoint.size();)
    {
      int r;
      Joint * aJoint = (Joint *)m_JointVector[m_StateVectorToJoint[i]];

      // ASSUMPTION: The joint in the VRML have only one degree of freedom.
      if ((r=aJoint->getIDinVRML())!=-1)
        {
	  m_VRMLIDToConfiguration[r] = i;
        }

      aJoint->stateVectorPosition((unsigned int)i);
      i+= aJoint->numberDof();
    }

  //added by oussama 30.03.2007
  //build m_ConfigurationToJoints (supposes the robot structure remains the same)
  for (unsigned int i=0;i<m_NbDofs;i++)
    m_ConfigurationToJoints.push_back(JointFromRank(i));
}

void DynamicMultiBody::UpdateTheSizeOfJointsJacobian()
{
  for(unsigned int i=0;i<m_JointVector.size();i++)
    {
      ((Joint *)m_JointVector[i])->resizeJacobianJointWrtConfig(m_NbDofs);
    }
}

void DynamicMultiBody::GetJointIDInConfigurationFromVRMLID(std::vector<int> &VectorFromVRMLIDToConfigurationID)
{
  VectorFromVRMLIDToConfigurationID.clear();
  VectorFromVRMLIDToConfigurationID = m_VRMLIDToConfiguration;
}

// Get the number of degrees of freedom of the robot
unsigned int DynamicMultiBody::numberDof() const
{
  return m_NbDofs;
}

/**
   \name Configuration, velocity and acceleration
*/

/**
   \brief Set the current configuration of the robot.  
   
   
   \param inConfig the configuration vector \f${\bf q}\f$.
   
   \return true if success, false if failure (the dimension of the
   input vector does not fit the number of degrees of freedom of the
   robot).
*/
bool DynamicMultiBody::currentConfiguration(const MAL_VECTOR(,double)& inConfig)
{
  if (MAL_VECTOR_SIZE(inConfig)!=
      MAL_VECTOR_SIZE(m_Configuration))
    return false;

  // Copy the configuration
  m_Configuration =  inConfig;

  ODEBUG("Went through here");
  int lindex=0;
  for (unsigned int i=0;i<m_ConfigurationToJoints.size();i++)
    {
      lindex = m_ConfigurationToJoints[i]->rankInConfiguration();

      // Update the pose of the joint when it is a free joint
      if (m_ConfigurationToJoints[i]->numberDof()==6)
        {
	  MAL_VECTOR_DIM(a6DPose,double,6);
	  for(int j=0;j<6;j++)
	    a6DPose(j) = inConfig[lindex++];

	  m_ConfigurationToJoints[i]->UpdatePoseFrom6DOFsVector(a6DPose);
        }
      else if (m_ConfigurationToJoints[i]->numberDof()==0)
        {
	  // do nothing
        }
      // Update only the quantity when it is a revolute or a prismatic joint.
      else
        {
	  int lIDinVRML = m_ConfigurationToJoints[i]->getIDinVRML();
	  if (lIDinVRML!=-1)
            {
	      m_ConfigurationToJoints[i]->quantity(inConfig[lindex]);
	      listOfBodies[ConvertIDINVRMLToBodyID[lIDinVRML]]->q = inConfig[lindex];
            }
	  else
            {
	      m_ConfigurationToJoints[i]->quantity(0.0);
	      listOfBodies[ConvertIDINVRMLToBodyID[lIDinVRML]]->q = 0.0;
            }

	  lindex++;
        }

    }

  if (0)
    {
      for(unsigned int i=0;i<listOfBodies.size();i++)
        {
	  cout << listOfBodies[i]->q * 180/M_PI << endl;
        }
    }
  return true;
}

/**
   \brief Get the current configuration of the robot.
   
   \return the configuration vector \f${\bf q}\f$.
*/
const MAL_VECTOR(,double)& DynamicMultiBody::currentConfiguration() const
{
  return m_Configuration;
}

/**
   \brief Set the current velocity of the robot.  
   
   \param inVelocity the velocity vector \f${\bf \dot{q}}\f$.
   
   \return true if success, false if failure (the dimension of the
   input vector does not fit the number of degrees of freedom of the
   robot).
*/
bool DynamicMultiBody::currentVelocity(const MAL_VECTOR(,double)& inVelocity)
{
  if (MAL_VECTOR_SIZE(inVelocity)!=
      MAL_VECTOR_SIZE(m_Velocity))
    MAL_VECTOR_RESIZE(m_Velocity,MAL_VECTOR_SIZE(inVelocity));

  // Copy the configuration
  for(unsigned int i=0;i<MAL_VECTOR_SIZE(inVelocity);i++)
    {
      // ODEBUG("Velocity : " << i << " " << inVelocity(i) );
      m_Velocity(i)= inVelocity(i);
    }

  int lindex=0;
  for (unsigned int i=0;i<m_JointVector.size();i++)
    {

      lindex = m_JointVector[i]->rankInConfiguration();

      // Update the pose of the joint when it is a free joint
      if (m_JointVector[i]->numberDof()==6)
        {
	  MAL_S3_VECTOR(,double) alinearVelocity;
	  MAL_S3_VECTOR(,double) anAngularVelocity;
	  for(int j=0;j<3;j++)
            {
	      alinearVelocity(j) = inVelocity[lindex];
	      anAngularVelocity(j) = inVelocity[lindex+3];
	      lindex++;
            }

	  ((Joint *)m_JointVector[i])->UpdateVelocityFrom2x3DOFsVector(alinearVelocity,
								       anAngularVelocity);
	  lindex+=3;
        }
      // Update only the quantity when it is a revolute or a prismatic joint.
      else
        {
	  int lIDinVRML = ((Joint *)m_JointVector[i])->getIDinVRML();
	  if (lIDinVRML>=0)
            {
	      listOfBodies[ConvertIDINVRMLToBodyID[lIDinVRML]]->dq = inVelocity[lindex];
	      // Update dq.
	      /* ODEBUG(" Id (Vs) :" << lindex
		 << " Id (JV) : " << i 
		 << " Id (VRML): " << lIDinVRML 
		 << " Id (Body): " << ConvertIDINVRMLToBodyID[lIDinVRML]
		 << " dq: " << listOfBodies[ConvertIDINVRMLToBodyID[lIDinVRML]]->dq ); */
            }
	  else
	    listOfBodies[ConvertIDINVRMLToBodyID[lIDinVRML]]->dq = 0.0;

	  lindex++;
        }

    }

  return true;

}

/**
   \brief Get the current velocity of the robot.
   
   \return the velocity vector \f${\bf \dot{q}}\f$.
*/
const MAL_VECTOR(,double)& DynamicMultiBody::currentVelocity() const
{
  return m_Velocity;

}
/**
   \brief Set the current acceleration of the robot.  
   
   \param inAcceleration the acceleration vector \f${\bf \ddot{q}}\f$.
   
   \return true if success, false if failure (the dimension of the
   input vector does not fit the number of degrees of freedom of the
   robot).
*/
bool DynamicMultiBody::currentAcceleration(const MAL_VECTOR(,double)& inAcceleration)
{
  if (MAL_VECTOR_SIZE(inAcceleration)!=
      MAL_VECTOR_SIZE(m_Acceleration))
    MAL_VECTOR_RESIZE(m_Acceleration,MAL_VECTOR_SIZE(inAcceleration));

  // Copy the configuration
  for(unsigned int i=0;i<MAL_VECTOR_SIZE(inAcceleration);i++)
    {
      // ODEBUG("Velocity : " << i << " " << inVelocity(i) );
      m_Acceleration(i)= inAcceleration(i);
    }

  int lindex=0;
  for (unsigned int i=0;i<m_JointVector.size();i++)
    {

      lindex = m_JointVector[i]->rankInConfiguration();

      // Update the pose of the joint when it is a free joint
      if (m_JointVector[i]->numberDof()==6)
        {
	  MAL_S3_VECTOR(,double) alinearAcceleration;
	  MAL_S3_VECTOR(,double) anAngularAcceleration;
	  for(int j=0;j<3;j++)
            {
	      alinearAcceleration(j) = inAcceleration[lindex];
	      anAngularAcceleration(j) = inAcceleration[lindex+3];
	      lindex++;
            }

	  //((Joint *)m_JointVector[i])->UpdateVelocityFrom2x3DOFsVector(alinearVelocity,
	  //								       anAngularVelocity);
	  lindex+=3;
        }
      // Update only the quantity when it is a revolute or a prismatic joint.
      else
        {
	  int lIDinVRML = ((Joint *)m_JointVector[i])->getIDinVRML();
	  if (lIDinVRML>=0)
            {
	      listOfBodies[ConvertIDINVRMLToBodyID[lIDinVRML]]->ddq = inAcceleration[lindex];
	      // Update dq.
	      /* ODEBUG(" Id (Vs) :" << lindex
		 << " Id (JV) : " << i 
		 << " Id (VRML): " << lIDinVRML 
		 << " Id (Body): " << ConvertIDINVRMLToBodyID[lIDinVRML]
		 << " dq: " << listOfBodies[ConvertIDINVRMLToBodyID[lIDinVRML]]->dq ); */
            }
	  else
	    listOfBodies[ConvertIDINVRMLToBodyID[lIDinVRML]]->ddq = 0.0;

	  lindex++;
        }

    }

  return true;
}

/**
   \brief Get the current acceleration of the robot.
   
   \return the acceleration vector \f${\bf \ddot{q}}\f$.
*/
const MAL_VECTOR(,double)& DynamicMultiBody::currentAcceleration() const
{
  return m_Acceleration;
}


/**
   @}
*/

/**
   \name Forward kinematics and dynamics
*/


/**
   \brief Compute forward kinematics.
   
   Update the position, velocity and accelerations of each
   wrt \f${\bf {q}}\f$, \f${\bf \dot{q}}\f$, \f${\bf \ddot{q}}\f$.
   
*/
bool DynamicMultiBody::computeForwardKinematics()
{
  MAL_S3_VECTOR(,double) lPositionForRoot;
  MAL_S3x3_MATRIX(,double) lOrientationForRoot;
  MAL_S3_VECTOR(,double) lLinearVelocityForRoot;
  MAL_S3_VECTOR(,double) lAngularVelocityForRoot;
  MAL_S3_VECTOR(,double) lLinearAccelerationForRoot;
  MAL_S3_VECTOR(,double) lAngularAccelerationForRoot;

  for(unsigned int i=0;i<3;i++)
    {
      lPositionForRoot(i)=(*m_RootOfTheJointsTree)(i,3);
      lLinearVelocityForRoot(i)=m_Velocity(i);
    }

  for(unsigned int i=0;i<3;i++)
    for(unsigned int j=0;j<3;j++)
      lOrientationForRoot(i,j)=(*m_RootOfTheJointsTree)(i,j);

  if (rootJoint()->numberDof() == 6)
    {
      for(unsigned int i=0;i<3;i++)
	lLinearVelocityForRoot(i)  = m_Velocity(i);

      for(unsigned int i=3;i<6;i++)
	lAngularVelocityForRoot(i-3)  = m_Velocity(i);
    }
  else
    {
      MAL_S3_VECTOR_FILL(lLinearVelocityForRoot, 0);
      MAL_S3_VECTOR_FILL(lAngularVelocityForRoot, 0);
    }

  if (rootJoint()->numberDof() == 6)
    {
      for(unsigned int i=0;i<3;i++)
	lLinearAccelerationForRoot(i)  = m_Acceleration(i);

      for(unsigned int i=3;i<6;i++)
	lAngularAccelerationForRoot(i-3)  = m_Acceleration(i);
    }
  else
    {
      MAL_S3_VECTOR_FILL(lLinearVelocityForRoot, 0);
      MAL_S3_VECTOR_FILL(lAngularVelocityForRoot, 0);
    }
  ODEBUG(" Position for Root: " << lPositionForRoot);
  ForwardVelocity(lPositionForRoot,
		  lOrientationForRoot,
		  lLinearVelocityForRoot,
		  lAngularVelocityForRoot,
		  lLinearAccelerationForRoot,
		  lAngularAccelerationForRoot);

  return true;
}

/*
  \brief Compute the dynamics of the center of mass.
  
  Compute the linear and  angular momentum and their time derivatives, at the center of mass.
*/
bool DynamicMultiBody::computeCenterOfMassDynamics()
{
  computeForwardKinematics();
  return true;
};

/**
   \brief Get the position of the center of mass.
*/
const MAL_S3_VECTOR(,double)&  DynamicMultiBody::positionCenterOfMass() const
{
  return positionCoMPondere;
}


/**
   \brief Get the velocity of the center of mass.
*/
const MAL_S3_VECTOR(,double)& DynamicMultiBody::velocityCenterOfMass()
{
  return m_VelocityCenterOfMass;
};

/**
   \brief Get the acceleration of the center of mass.
*/
const MAL_S3_VECTOR(,double)& DynamicMultiBody::accelerationCenterOfMass()
{
  return m_AccelerationCenterOfMass;
};

/**
   \brief Get the linear momentum of the robot.
*/
const MAL_S3_VECTOR(,double)& DynamicMultiBody::linearMomentumRobot()
{
  return m_P;
};

/**
   \brief Get the time-derivative of the linear momentum.
*/
const MAL_S3_VECTOR(,double)& DynamicMultiBody::derivativeLinearMomentum()
{
  return m_dP;
};

/**
   \brief Get the angular momentum of the robot at the center of mass.
*/
const MAL_S3_VECTOR(,double)& DynamicMultiBody::angularMomentumRobot()
{
  return m_L;

} ;

/**
   \brief Get the time-derivative of the angular momentum at the center of mass.
*/
const MAL_S3_VECTOR(,double)& DynamicMultiBody::derivativeAngularMomentum()
{

  return m_dL;

};

/* Jacobian functions */

// Compute the Jacobian matrix of the center of Mass. 
// Interaction with the environement not taken into account.
void DynamicMultiBody::computeJacobianCenterOfMass()
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

const MAL_MATRIX(,double) &DynamicMultiBody::jacobianCenterOfMass() const
{
  return m_JacobianOfTheCoM;

}

MAL_MATRIX(,double) &DynamicMultiBody::getJacobianOfTheCoM()
{
  return m_JacobianOfTheCoM;
}

double DynamicMultiBody::upperBoundDof(unsigned int inRankInConfiguration)
{
  //     if (inRankInConfiguration == m_RootOfTheJointsTree->rankInConfiguration())
  //         return 0;

  return m_ConfigurationToJoints[inRankInConfiguration]->upperBound(0);
  //WARNING: this code does not work if the joints have more than a single degree of freedom
}

double DynamicMultiBody::lowerBoundDof(unsigned int inRankInConfiguration)
{
  //     if (inRankInConfiguration == m_RootOfTheJointsTree->rankInConfiguration())
  //         return 0;

  return m_ConfigurationToJoints[inRankInConfiguration]->lowerBound(0);
  //WARNING: this code does not work if the joints have more than a single degree of freedom
}

double DynamicMultiBody::upperBoundDof(unsigned int inRankInConfiguration,
                                       const vectorN& inConfig)
{
  return upperBoundDof(inRankInConfiguration);
}

double DynamicMultiBody::lowerBoundDof(unsigned int inRankInConfiguration,
                                       const vectorN& inConfig)
{
  return lowerBoundDof(inRankInConfiguration);
}

/* Methods related to the fixed joints */

void DynamicMultiBody::addFixedJoint(CjrlJoint *inFixedJoint)
{
  m_VectorOfFixedJoints.insert(m_VectorOfFixedJoints.end(),inFixedJoint);
}

unsigned int DynamicMultiBody::countFixedJoints() const
{
  return m_VectorOfFixedJoints.size();
}

void DynamicMultiBody::removeFixedJoint(CjrlJoint * inFixedJoint)
{
  std::vector<CjrlJoint *>::iterator it_Joint = m_VectorOfFixedJoints.begin();
  while((*it_Joint!= inFixedJoint) &&
	(it_Joint!=m_VectorOfFixedJoints.end()))
    it_Joint++;

  if (it_Joint!=m_VectorOfFixedJoints.end())
    m_VectorOfFixedJoints.erase(it_Joint);

}

void DynamicMultiBody::clearFixedJoints()
{
  m_VectorOfFixedJoints.clear();
}

CjrlJoint& DynamicMultiBody::fixedJoint(unsigned int inJointRank)
{

  //if ((inJointRank>0) & (inJointRank<=m_VectorOfFixedJoints.size()))
  if (inJointRank<m_VectorOfFixedJoints.size())
    return *m_VectorOfFixedJoints[inJointRank];
  return *m_VectorOfFixedJoints[0];
}
/* End of Methods related to the fixed joints */

int DynamicMultiBody::setLinksBetweenJointNamesAndRank(std::vector<NameAndRank_t> &aLinks)
{
  if (m_LinksBetweenJointNamesAndRank.size()!=
      aLinks.size())
    m_LinksBetweenJointNamesAndRank.resize(aLinks.size());

  for(unsigned int i=0;i<m_LinksBetweenJointNamesAndRank.size();i++)
    {
      m_LinksBetweenJointNamesAndRank[i] = aLinks[i];
    }
  return 0;
}

int DynamicMultiBody::getLinksBetweenJointNamesAndRank(std::vector<NameAndRank_t> &aLinks)
{
  if (m_LinksBetweenJointNamesAndRank.size()!=
      aLinks.size())
    aLinks.resize(m_LinksBetweenJointNamesAndRank.size());

  for(unsigned int i=0;i<m_LinksBetweenJointNamesAndRank.size();i++)
    {
      aLinks[i] = m_LinksBetweenJointNamesAndRank[i];
    }
  return 0;
}

void DynamicMultiBody::setJointOrderInConfig(std::vector<CjrlJoint *>inJointVector)
{
  if (m_LinksBetweenJointNamesAndRank.size()!=inJointVector.size())
    m_LinksBetweenJointNamesAndRank.resize(inJointVector.size());

  unsigned int LocalRank = 0;
  for(unsigned int i=0;i<inJointVector.size();i++)
    {

      char Buffer[128];
      memset(Buffer,0,128);
      sprintf(Buffer,"JOINT_%2d",i);
      strcpy(m_LinksBetweenJointNamesAndRank[i].LinkName,Buffer);
      m_LinksBetweenJointNamesAndRank[i].RankInConfiguration=LocalRank;
      LocalRank+= inJointVector[i]->numberDof();
    }

}

bool DynamicMultiBody::isSupported(const std::string &aName)
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
  return false;
}

const matrixNxP & DynamicMultiBody::currentTorques() const
{
  return m_Torques;
}

const matrixNxP & DynamicMultiBody::currentForces() const
{
  return m_Forces;
}

bool DynamicMultiBody::getProperty(const std::string &inProperty,std::string &outValue)
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
  outValue="false";
  return false;
}

bool DynamicMultiBody::setProperty(std::string &inProperty,const std::string &inValue)
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
  return false;
}

void DynamicMultiBody::angularMomentumWrtCoM(vector3d & angularmomentum) 
{
  angularMomentumWrtToPt(positionCoMPondere, angularmomentum);
}

void DynamicMultiBody::angularMomentumWrtToPt(vector3d &apoint, vector3d & angularmomentum)
{
  /** Intermediate variables. The mantra is :
      "To optimize those variables, in the Compiler we trust"
      (with the appropriate compilation options).
  */
  vector3d NE_lP,NE_lw_c, NE_tmp3, NE_tmp2, NE_tmp,NE_lL;
  matrix3d NE_Rtmp, NE_Rt, NE_Ro, NE_Rot;
  /* End of intermediate */

  DynamicBody *aDB=0;
  int currentNode = labelTheRoot;
  currentNode = listOfBodies[labelTheRoot]->child;
  vector3d lL(0.0,0.0,0.0);

  do
    {

      aDB = listOfBodies[currentNode];

      NE_lP = listOfBodies[currentNode]->P;
      ODEBUG("P: " << NE_lP );
      NE_lw_c = listOfBodies[currentNode]->w_c - positionCoMPondere;

      // Computes angular momentum matrix L
      // Lk = xc x Pk + R * I * Rt * w
      MAL_S3x3_TRANSPOSE_A_in_At(listOfBodies[currentNode]->R,NE_Rt);

      MAL_S3_VECTOR_CROSS_PRODUCT(NE_tmp3,NE_lw_c,NE_lP);

      MAL_S3x3_C_eq_A_by_B(NE_tmp2,NE_Rt , listOfBodies[currentNode]->w);
      MAL_S3x3_C_eq_A_by_B(NE_tmp, listOfBodies[currentNode]->getInertie(),NE_tmp2);
      MAL_S3x3_C_eq_A_by_B(NE_tmp2, listOfBodies[currentNode]->R,NE_tmp);
      NE_lL = NE_tmp3 + NE_tmp2;
      ODEBUG("L: " << lL);

      lL += NE_lL;

      int step=0;
      int NextNode=0;
      do
        {

	  if (step==0)
            {
	      NextNode = listOfBodies[currentNode]->child;
	      step++;
            }
	  else if(step==1)
            {
	      NextNode = listOfBodies[currentNode]->sister;
	      step++;
            }
	  else if (step==2)
            {
	      NextNode = listOfBodies[currentNode]->getLabelMother();
	      if (NextNode>=0)
                {
		  /* Test if current node is leaf,
		     because in this case the force are not set properly. */
		  if (m_ComputeBackwardDynamics)
                    {
		      if ((listOfBodies[currentNode]->sister==-1) &&
			  (listOfBodies[currentNode]->child==-1))
			BackwardDynamics(*listOfBodies[currentNode]);

		      /* Compute backward dynamics */
		      BackwardDynamics(*listOfBodies[NextNode]);
                    }
		  currentNode = NextNode;
		  NextNode = listOfBodies[currentNode]->sister;
                }
	      else
		NextNode=labelTheRoot;
            }


        }
      while (NextNode==-1);
      currentNode = NextNode;

    }
  while(currentNode!=labelTheRoot);
  
  angularmomentum = lL;
}

void DynamicMultiBody::computeInertiaMatrix()
{
  if ((MAL_MATRIX_NB_ROWS(m_InertiaMatrix) != numberDof()) || 
      (MAL_MATRIX_NB_COLS(m_InertiaMatrix) != numberDof()))
    MAL_MATRIX_RESIZE(m_InertiaMatrix,numberDof(),numberDof());

  MAL_MATRIX_FILL(m_InertiaMatrix,0);

  unsigned int rank;
  Joint* aJoint;
  DynamicBody* aBody;
    
  for(unsigned int i=0;i<m_ConfigurationToJoints.size();i++)
    {
      if (m_ConfigurationToJoints[i] == rootJoint())
	continue;

      aJoint = m_ConfigurationToJoints[i];
      aBody=  aJoint->linkedDBody();
      rank = aJoint->rankInConfiguration();
      
      matrixNxP pJacobian;
      MAL_MATRIX_RESIZE(pJacobian, 6, numberDof());
      vector3d av(0,0,0); // Dummy 
      getJacobian(*rootJoint(),*aJoint,av,pJacobian);

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

      // Used to compute the anti-symmetric matrix.
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

const matrixNxP & DynamicMultiBody::inertiaMatrix() const
{
  return m_InertiaMatrix;
}

matrixNxP & DynamicMultiBody::getInertiaMatrix() 
{
  return m_InertiaMatrix;
}

