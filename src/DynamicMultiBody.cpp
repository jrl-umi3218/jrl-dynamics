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
   
   JRL-Japan, CNRS/AIST

   All rights reserved.
   
   Redistribution and use in source and binary forms, with or without modification, 
   are permitted provided that the following conditions are met:
   
   * Redistributions of source code must retain the above copyright notice, 
   this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright notice, 
   this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
   * Neither the name of the CNRS and AIST nor the names of its contributors 
   may be used to endorse or promote products derived from this software without specific prior written permission.
   
   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS 
   OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY 
   AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER 
   OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, 
   OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS 
   OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
   HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
   STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
   IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include <fstream>

#include <DynamicMultiBody.h>

#define RESETDEBUG5(y) { ofstream DebugFile; \
    DebugFile.open(y,ofstream::out); \
    DebugFile.close();}

#define ODEBUG5(x,y) { ofstream DebugFile; \
    DebugFile.open(y,ofstream::app); \
    DebugFile << "DMB: " << x << endl; \
    DebugFile.close();}

#define ODEBUG2(x)
#define ODEBUG3(x) cerr << "DynamicMultiBody :" << x << endl

#if 0
#define ODEBUG(x) cerr << "DynamicMultiBody :" <<  x << endl
#else
#define ODEBUG(x) 
#endif

#if 0

#define ODEBUG4(x,y) { ofstream DebugFile; DebugFile.open(y,ofstream::app); DebugFile << "WalkGenJRLIntegrate: " << x << endl; DebugFile.close();}
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
  m_IterationNumber = 0;

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
  RESETDEBUG5("DebugDataDMB_ZMP.dat");
}

DynamicMultiBody::~DynamicMultiBody()
{

}

void DynamicMultiBody::SpecifyTheRootLabel(int ID)
{
  labelTheRoot = ID;
  listOfBodies[ID].setLabelMother(-1);

  // Right now it is assume that the first body is the world,
  // and that this body is related to another body.
  // Thus liaisons[ID].size() should be equal to one.


  if (liaisons[ID].size()!=1)
    {
      cout << "Wrong assumption concerning the initial body." << endl;
      return;
    }
  int ld = liaisons[ID][0].liaison;
  m_RootOfTheJointsTree = & listeLiaisons[ld].aJoint;
  m_RootOfTheJointsTree->setLinkedBody(listOfBodies[ID]);
  // Start the vector of joints.
  m_JointVector.clear();
  m_JointVector.insert(m_JointVector.end(),m_RootOfTheJointsTree);
  int lVRMLID = m_RootOfTheJointsTree->getIDinVRML();
  if (m_NbOfVRMLIDs < lVRMLID)
    m_NbOfVRMLIDs = lVRMLID;
      
  // Find out the next body to be examine.
  /*
  int NewBody = listeLiaisons[ld].indexCorps1;
  if (NewBody == ID)
    NewBody = listeLiaisons[ld].indexCorps2;

    ReLabelling(NewBody,ld);   */
  ReLabelling(ID,ld);
  
  // Once finished we initialize the child and the sister.
  for(unsigned int i=0;i<listOfBodies.size();i++)
    {
      int lMother,lElderSister;
      if ((lMother=listOfBodies[i].getLabelMother()) != -1)
	{

	  if ((lElderSister=listOfBodies[lMother].child) == -1)
	    {
	      listOfBodies[lMother].child = i;					  // Mother, I am your daughter !
	      
	    }
	  else
	    {
	      // I have an elder sister !

	      while (listOfBodies[lElderSister].sister != -1)
		lElderSister = listOfBodies[lElderSister].sister;  // I have another elder sister !

	      listOfBodies[lElderSister].sister = i;				  // I am your younger sister !
	    }
	}
    }
}

void DynamicMultiBody::UpdateBodyParametersFromJoint(int BodyID, int JointID, int LiaisonForFatherJoint)
  // cID : corps identifier
  // lD : liaison destination
{

  // Update the rotation axis.
  listOfBodies[BodyID].a =  listeLiaisons[JointID].aJoint.axe();
  // Update the translation vector
  listeLiaisons[JointID].aJoint.getStaticTranslation(listOfBodies[BodyID].b);
  // listOfBodies[cID].R = 
  listeLiaisons[JointID].aJoint.setLinkedBody(listOfBodies[BodyID]);
  listOfBodies[BodyID].joint( &listeLiaisons[JointID].aJoint);
  
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
      listOfBodies[corpsSon].setLabelMother(corpsMother);
      
      if( listeLiaisons[liaisonDestination].aJoint.getIDinVRML()!=-1)
	{

	  // Update the connections between the Joints.
	  int lIDinVRML = listeLiaisons[liaisonDestination].aJoint.getIDinVRML();
	  if (lIDinVRML!=-1)
	    {
	      ConvertIDINVRMLToBodyID[lIDinVRML] = corpsSon;
	      
	    }

	  listeLiaisons[liaisonDeProvenance].aJoint.addChildJoint(listeLiaisons[liaisonDestination].aJoint);
	  listeLiaisons[liaisonDestination].aJoint.SetFatherJoint(&listeLiaisons[liaisonDeProvenance].aJoint);
	  
	  // Update the vector of joints.
	  m_JointVector.insert(m_JointVector.end(),&listeLiaisons[liaisonDestination].aJoint);
	  int lVRMLID = listeLiaisons[liaisonDestination].aJoint.getIDinVRML();
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
  DynamicBody *Child = &listOfBodies[IndexChild];
  while(IndexChild!=-1)
    {
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
      IndexChild = listOfBodies[IndexChild].sister;
      if (IndexChild!=-1)
	Child=&listOfBodies[IndexChild];
    }
  
}



void DynamicMultiBody::ForwardVelocity(MAL_S3_VECTOR(&PosForRoot,double), 
                                       MAL_S3x3_MATRIX(&OrientationForRoot,double),
                                               MAL_S3_VECTOR(&v0ForRoot,double),
                                                       MAL_S3_VECTOR(&wForRoot,double))
{
    double norm_w, th;
 //  int NbOfNodes=1;
    int currentNode = labelTheRoot;
    int lMother=0;
    MAL_S3x3_MATRIX( Ro,double);
    MAL_S3x3_MATRIX( w_wedge,double);
    MAL_S3_VECTOR( wn,double);
    double NORME_EPSILON=10e-7;

    listOfBodies[labelTheRoot].p = PosForRoot;
    listOfBodies[labelTheRoot].v0 = v0ForRoot;
    listOfBodies[labelTheRoot].R = OrientationForRoot;
    listOfBodies[labelTheRoot].w = wForRoot;

    currentNode = listOfBodies[labelTheRoot].child;
  //  cout << "STARTING FORWARD VELOCITY " << v0ForRoot << endl;
  
    MAL_S3_VECTOR_FILL(m_P,0);
    MAL_S3_VECTOR_FILL(m_L,0);
    MAL_S3_VECTOR(lP,double);
    MAL_S3_VECTOR(lL,double);

    currentNode = listOfBodies[labelTheRoot].child;
    positionCoMPondere[0] = 0;
    positionCoMPondere[1] = 0;
    positionCoMPondere[2] = 0;
  
    MAL_S3_VECTOR(tmp,double);
    MAL_S3_VECTOR(tmp2,double);
    ODEBUG("PosForRoot: " << PosForRoot );
    ODEBUG("v0ForRoot: " << v0ForRoot );
    ODEBUG("OrientationForRoot: " << OrientationForRoot );
    do
    {

        DynamicBody aDB = listOfBodies[currentNode];

        norm_w = MAL_S3_VECTOR_NORM(aDB.a);
        lMother = aDB.getLabelMother();

        ODEBUG("CurrentBody " << listOfBodies[currentNode].getName());
      
      // ----------------------------------
      // Rodrigues formula. (p33)
        if (norm_w< NORME_EPSILON)
        {
            MAL_S3x3_MATRIX_SET_IDENTITY(Ro);
        }
        else 
        {
            th = norm_w * aDB.q;
            wn = aDB.a / norm_w;
            w_wedge(0,0) =   0.0;w_wedge(0,1)= -wn[2]; w_wedge(0,2)=  wn[1]; // Cross product
            w_wedge(1,0) = wn[2];w_wedge(1,1)=    0.0; w_wedge(1,2)= -wn[0];
            w_wedge(2,0) =-wn[1];w_wedge(2,1)=  wn[0]; w_wedge(2,2)=    0.0;
	  
            ODEBUG("w_wedge : " << w_wedge);
            ODEBUG("aDB.a :" << aDB.a );
            ODEBUG("norm_w:" << norm_w);

            double ct = cos(th); double lct= (1-ct);
            double st = sin(th);
            Ro(0,0) = ct + wn[0]*wn[0]* lct;  
            Ro(0,1) = wn[0]*wn[1]*lct-wn[2]*st; 
            Ro(0,2) = wn[1] * st+wn[0]*wn[2]*lct;
            Ro(1,0) = wn[2]*st +wn[0]*wn[1]*lct; 
            Ro(1,1) = ct + wn[1]*wn[1]*lct;    
            Ro(1,2) = -wn[0]*st+wn[1]*wn[2]*lct;
            Ro(2,0) = -wn[1]*st+wn[0]*wn[2]*lct; 
            Ro(2,1) = wn[0]*st + wn[1]*wn[2]*lct; 
            Ro(2,2) = ct + wn[2]*wn[2]*lct;
        }
      
        ODEBUG("Ro:" << endl << Ro );
        ODEBUG("MR:" << listOfBodies[lMother].R );
        ODEBUG("b: " << aDB.b);
        ODEBUG("Mp: " << listOfBodies[lMother].p);
      // End Rodrigues formula
      //-------------------------------

      // Position and orientation in reference frame
        listOfBodies[currentNode].p = MAL_S3x3_RET_A_by_B(listOfBodies[lMother].R , aDB.b )
                + listOfBodies[lMother].p;
        MAL_S3x3_C_eq_A_by_B(listOfBodies[currentNode].R ,listOfBodies[lMother].R , Ro);
        listOfBodies[currentNode].Riip1 = Ro;

        ODEBUG("q: "<< aDB.q );
        ODEBUG("p: " 
                << listOfBodies[currentNode].p[0] << " " 
                << listOfBodies[currentNode].p[1] << " " 
                << listOfBodies[currentNode].p[2] << " " );
        ODEBUG("R: "<< aDB.R );
      // Computes the angular velocity. 

        ODEBUG("dq: "<< aDB.dq );
        tmp = listOfBodies[currentNode].a * listOfBodies[currentNode].dq;
        tmp = MAL_S3x3_RET_A_by_B(listOfBodies[lMother].R,tmp);

        listOfBodies[currentNode].w  = listOfBodies[lMother].w  + tmp;

        ODEBUG("w: " << listOfBodies[currentNode].w );

      // Computes the linear velocity.
        MAL_S3x3_C_eq_A_by_B(tmp,listOfBodies[lMother].R,
                             listOfBodies[currentNode].b);

        MAL_S3_VECTOR_CROSS_PRODUCT(tmp2,listOfBodies[lMother].w , tmp);

        listOfBodies[currentNode].v0 = listOfBodies[lMother].v0 + tmp2;

        ODEBUG("v0: " 
                << listOfBodies[currentNode].v0[0] << " " 
                << listOfBodies[currentNode].v0[1] << " " 
                << listOfBodies[currentNode].v0[2] << " " );
	
      // Computes also the center of mass in the reference frame.

        MAL_S3_VECTOR( cl , double);
        ODEBUG("c: " << listOfBodies[currentNode].c);
        MAL_S3x3_C_eq_A_by_B(cl,listOfBodies[currentNode].R, listOfBodies[currentNode].c);
        MAL_S3_VECTOR(lw_c,double);
        lw_c = cl + listOfBodies[currentNode].p;
        positionCoMPondere +=  lw_c * listOfBodies[currentNode].getMasse();
        ODEBUG("w_c: " << lw_c[0] << " " << lw_c[1] << " " << lw_c[2]);
        ODEBUG("Masse " << listOfBodies[currentNode].getMasse());
        ODEBUG("positionCoMPondere " << positionCoMPondere);

      // Computes momentum matrix P.
        tmp2 = cl;
        ODEBUG("w: " << listOfBodies[currentNode].w );
        MAL_S3_VECTOR_CROSS_PRODUCT(tmp, tmp2 , listOfBodies[currentNode].w);
        ODEBUG("cl^w: " << tmp);
        ODEBUG("masse: " << listOfBodies[currentNode].getMasse());
        ODEBUG("v0: " << listOfBodies[currentNode].v0 );
        lP=  (listOfBodies[currentNode].v0 + 
                tmp )* listOfBodies[currentNode].getMasse();
        listOfBodies[currentNode].P = lP;
        ODEBUG("P: " << lP );
        m_P += lP;
      
      // Computes angular momentum matrix L
        MAL_S3x3_MATRIX( Rt,double);
        Rt = listOfBodies[currentNode].R;
        Rt = MAL_S3x3_RET_TRANSPOSE(Rt);

        MAL_S3_VECTOR(tmp3,double);
        MAL_S3_VECTOR_CROSS_PRODUCT(tmp3,lw_c,lP);

        MAL_S3x3_C_eq_A_by_B(tmp2,Rt , listOfBodies[currentNode].w);
        MAL_S3x3_C_eq_A_by_B(tmp, listOfBodies[currentNode].getInertie(),tmp2);
        MAL_S3x3_C_eq_A_by_B(tmp2, listOfBodies[currentNode].R,tmp);
        lL = tmp3 + tmp2; 
        ODEBUG("L: " << lL);
      
        listOfBodies[currentNode].L = lL;
        listOfBodies[currentNode].w_c = lw_c;
        m_L+= lL;

      // ******************* Computes the angular acceleration for joint i. ******************** 
        MAL_S3_VECTOR(,double) aRa; // Spong p.278
        MAL_S3x3_C_eq_A_by_B(aRa,listOfBodies[currentNode].R, aDB.a);
      // tmp2 = z_{i-1} * dqi
        tmp2 = aRa * listOfBodies[currentNode].dq; 
      // tmp3 = w^{(0)}_i x z_{i-1} * dqi
        MAL_S3_VECTOR_CROSS_PRODUCT(tmp3,listOfBodies[currentNode].w,tmp2); 
      // tmp2 = z_{i-1} * ddqi
        tmp2 = aRa * listOfBodies[currentNode].ddq; 
        listOfBodies[currentNode].dw = tmp2 + tmp3 + listOfBodies[lMother].dw;

      // ******************* Computes the linear acceleration for joint i. ******************** 
        MAL_S3_VECTOR(,double) aRb; // Spong p. 279
        MAL_S3x3_C_eq_A_by_B(aRb, listOfBodies[currentNode].R, aDB.b);
      // tmp3 = w_i x (w_i x r_{i,i+1})
        MAL_S3_VECTOR_CROSS_PRODUCT(tmp2,listOfBodies[currentNode].w,aRb);
        MAL_S3_VECTOR_CROSS_PRODUCT(tmp3,listOfBodies[currentNode].w,tmp2);

      // tmp2 = dw_I x r_{i,i+1}
        MAL_S3_VECTOR_CROSS_PRODUCT(tmp2,listOfBodies[currentNode].dw,aRb);

        // 
        MAL_S3x3_MATRIX(Rot,double);
        Rot = MAL_S3x3_RET_TRANSPOSE(Ro);
        MAL_S3_VECTOR(,double) RotByMotherdv;
        MAL_S3x3_C_eq_A_by_B(RotByMotherdv,Rot,listOfBodies[lMother].dv);
        listOfBodies[currentNode].dv = RotByMotherdv + tmp2 + tmp3;

      // *******************  Acceleration for the center of mass of body  i ************************
        MAL_S3_VECTOR(,double) aRc; // Spong p. 279
        MAL_S3x3_C_eq_A_by_B(aRc, listOfBodies[currentNode].R, aDB.c);
        MAL_S3_VECTOR_CROSS_PRODUCT(tmp2,listOfBodies[currentNode].w,aRc);
        MAL_S3_VECTOR_CROSS_PRODUCT(tmp3,listOfBodies[currentNode].w,tmp2);

      // tmp2 = dw_I x r_{i,i+1}
        MAL_S3_VECTOR_CROSS_PRODUCT(tmp2,listOfBodies[currentNode].dw,aRc);
        // 
        listOfBodies[currentNode].dv_c = RotByMotherdv + tmp2 + tmp3;

      
      // TO DO if necessary : cross velocity.
        int step=0;
        int NextNode=0;
        do{
       
            if (step==0)
            {
                NextNode = listOfBodies[currentNode].child;
                step++;
            }
            else if(step==1)
            {
                NextNode = listOfBodies[currentNode].sister;
                step++;
            }
            else if (step==2)
            {
                NextNode = listOfBodies[currentNode].getLabelMother();
                if (NextNode>=0)
                {
		/* Test if current node is leaf, 
                    because in this case the force are not set properly. */
                    if ((listOfBodies[currentNode].sister==-1) &&
                         (listOfBodies[currentNode].child==-1))
                        BackwardDynamics(listOfBodies[currentNode]);

                    /* Compute backward dynamics */
                    BackwardDynamics(listOfBodies[NextNode]);
                    currentNode = NextNode;
                    NextNode = listOfBodies[currentNode].sister;
                }
                else 
                    NextNode=labelTheRoot;
            }

	  
        } 
        while (NextNode==-1);
        currentNode = NextNode;
      
    }
    while(currentNode!=labelTheRoot);

  // Compute the skew matrix related to the weighted CoM.
    MAL_S3_VECTOR(,double) lpComP = positionCoMPondere/masse;

    positionCoMPondere = lpComP;
  
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

    ODEBUG5( m_IterationNumber << " " 
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

    ODEBUG("Position of the CoM = " << positionCoMPondere <<endl <<
            "Weighted Com = "<< lpComP );

    SkewCoM(0,0) =         0; SkewCoM(0,1) = - lpComP[2]; SkewCoM(0,2) = lpComP[1];
    SkewCoM(1,0) = lpComP[2]; SkewCoM(1,1) =           0; SkewCoM(1,2) =-lpComP[0];
    SkewCoM(2,0) =-lpComP[1]; SkewCoM(2,1) =   lpComP[0]; SkewCoM(2,2) =         0;

    m_IterationNumber++;
}



void DynamicMultiBody::FiniteDifferenceStateUpdate(double inTimeStep, bool reset)
{
//     Joint* joint = 0;
            
    if (reset)
    {
        vector3d zeros3;
        MAL_S3_VECTOR_FILL(zeros3,0); 
        for (unsigned int i=0;i<listOfBodies.size();i++)
        {
            DynamicBody& body = listOfBodies[i];
          
            body.pastp = body.p;
            body.pastR = body.R;
            body.pastv0 = body.v0;
            body.pastw = body.w;
            
            body.v0 = zeros3;
            body.w = zeros3;
            body.dv = zeros3;
            body.dw = zeros3;
            body.P = zeros3;
            body.L = zeros3;
            
            /*//(temporary ?) update of CjrlRigidVelocity member of Joints
            joint = (Joint*)body.joint();
            joint->UpdateVelocityFrom2x3DOFsVector(zeros3, zeros3);
            */
            m_P = zeros3;
            m_Prev_P = zeros3;
            m_L = zeros3;
            m_Prev_L = zeros3;
            m_ZMP = positionCoMPondere;
            m_ZMP(2) = 0.0;
        }
        return;
    }
    
    matrix3d Ro,Roo,Rt;
    vector3d vek;

    //for momenta and ZMP computation
    vector3d lP;
    vector3d lL;
    MAL_S3_VECTOR_FILL(m_P,0);
    MAL_S3_VECTOR_FILL(m_L,0);

    vector3d tmp,tmp2,tmp3;
    vector3d wlc; //from joint to joint com in world frame

    for (unsigned int i=0;i<listOfBodies.size();i++)
    {
        DynamicBody& body = listOfBodies[i];

        // Linear velocity.
        body.v0 = (body.p - body.pastp)/inTimeStep;
        body.pastp = body.p;

        // Angular velocity.
        Rt = MAL_S3x3_RET_TRANSPOSE(body.pastR);
        Roo = (body.R - body.pastR);
        MAL_S3x3_C_eq_A_by_B(Ro , Roo, Rt);
        body.w[0]  = Ro(1,2)/inTimeStep;
        body.w[1]  = Ro(0,2)/inTimeStep;
        body.w[2]  = Ro(1,0)/inTimeStep;
        body.pastR = body.R;

        /*//(temporary ?) update of CjrlRigidVelocity member of Joints
        joint = (Joint*)body.joint();
        joint->UpdateVelocityFrom2x3DOFsVector(body.v0, body.w);
        */
        // Linear acceleration
        body.dv = (body.v0 - body.pastv0)/inTimeStep;
        body.pastv0 = body.v0;
        
        // Angular acceleration
        body.dw = (body.w - body.pastw)/inTimeStep;
        body.pastw = body.w;

        
        // contribution of this body to the linear momentum.
        
        MAL_S3x3_C_eq_A_by_B(wlc, body.R, body.c);
        MAL_S3_VECTOR_CROSS_PRODUCT(tmp, body.w, wlc);
        lP =  (body.v0 + tmp)* body.getMasse();
        body.P = lP;
        m_P += lP;

        // contribution of this body to the angular momentum.
        Rt = MAL_S3x3_RET_TRANSPOSE(body.R);
        
        MAL_S3_VECTOR_CROSS_PRODUCT(tmp3,body.w_c,lP);
        
        MAL_S3x3_C_eq_A_by_B(tmp2,Rt , body.w);
        MAL_S3x3_C_eq_A_by_B(tmp, body.getInertie(),tmp2);
        MAL_S3x3_C_eq_A_by_B(tmp2, body.R,tmp);
        lL = tmp3 + tmp2;
        body.L = lL;
        m_L += lL;

    }

    // Update the momentum derivative

    m_dP = (m_P - m_Prev_P)/inTimeStep;
    m_dL = (m_L - m_Prev_L)/inTimeStep;

    // Update the ZMP value.
    double px,py, pz = 0.0;
    CalculateZMP(px, py, m_dP, m_dL, pz);

    m_ZMP(0) = px;
    m_ZMP(1) = py;
    m_ZMP(2) = pz;

    m_Prev_P = m_P;
    m_Prev_L = m_L;

}

void DynamicMultiBody::InertiaMatricesforRMCFirstStep()
{

  double norm_w;
  //  int NbOfNodes=1;
  int currentNode = labelTheRoot;
  int lMother=0;
  MAL_S3x3_MATRIX(Ro,double);
  MAL_S3x3_MATRIX(w_wedge,double);
  
  currentNode = listOfBodies[labelTheRoot].child;
  //  cout << "STARTING FORWARD VELOCITY " << v0ForRoot << endl;

  for(unsigned int i=0;i<listOfBodies.size();i++)
    {
      listOfBodies[i].setExplored(0);
      listOfBodies[i].m_tildem=0.0;
      listOfBodies[i].m_tildem_sister=0.0;
      MAL_S3_VECTOR_FILL(listOfBodies[i].m_tildec,0.0);
      MAL_S3_VECTOR_FILL(listOfBodies[i].m_tildec_sister,0.0);
      
    }
  // cout << "labelTheRoot" << labelTheRoot<< endl;
  //  cout << "Current Node while starting: " << currentNode;
  //  cout << "Mother of the current Node" << listOfBodies[currentNode].getLabelMother() << endl;
  DynamicBody aDB;
  do
    {

      aDB = listOfBodies[currentNode];
      norm_w = MAL_S3_VECTOR_NORM(aDB.a);
      lMother = aDB.getLabelMother();
      

      int NextNode=0;
      int lContinue = 1;
      bool Ec=false, Es=false;
      int Ic=-1, Is=-1;

      do{

	Ec=false; Es=false;
	// Test if the node should be compute.
	Ic = aDB.child;
	Is = aDB.sister;
	if (!aDB.getExplored())
	  {
	    // Leaf
	    if ((Ic==-1) && (Is==-1))
	      {
		Es=Ec=true;
		lContinue =0;
	      }
	    else 
	      {
		
		if (Ic!=-1)
		  {
		    //cout << "IC: " << Ic << " " << (int) listOfBodies[Ic].getExplored() << endl;
		    if (listOfBodies[Ic].getExplored())
		      Ec = true;
		  }
		else Ec = true;
		
		if (Is!=-1)
		  {
		    //cout << "IS: " << Is << " " << (int) listOfBodies[Is].getExplored() << endl;
		    if (listOfBodies[Is].getExplored())
		      Es =true;
		  }
		else Es =true;
		
		if (Es && Ec)
		  lContinue = 0;
	      }
	  }

	// Depth first exploration
	if (lContinue)
	  {
	    if ((Ic!=-1) && (!Ec))
	      NextNode = aDB.child;
	    else 
	      NextNode = aDB.sister;
	    currentNode = NextNode;

	  } 
	aDB = listOfBodies[currentNode];
	//	cout << aDB.getName()<< " " << currentNode << endl;
	//	cout << aDB.sister << endl;
      }
      while (lContinue);

      double ltotaltildem = aDB.getMasse();      
      /*      cout << "FirstStep:currentNode : " << currentNode << " " << listOfBodies[currentNode].getLabelMother() 
	   << " " << aDB.getName() << endl;
      cout << "ltotaltiledm " << ltotaltildem << " " << listOfBodies[currentNode].getName()<< endl;
      cout << "Ic " << Ic << " Is :" <<Is << " Ec :" << (int)Ec << " Es :" << (int)Es << endl;
      */

      // Compute tilde m
      aDB.m_tildem = ltotaltildem;
      
      if ((Ec) && (Ic!=-1))
	{
	  aDB.m_tildem += listOfBodies[Ic].m_tildem +
	    listOfBodies[Ic].m_tildem_sister;
	}
      
      if ((Es) && (Is!=-1))
	{
	  aDB.m_tildem_sister += listOfBodies[Is].m_tildem +
	    listOfBodies[Is].m_tildem_sister;
	}
	
      //      cout << "Up here" << endl;
      // Compute Tilde CoM (Eq 24 on Kajita IROS 2003 p1647)
      MAL_S3_VECTOR(ltildec,double);
      ltildec =   aDB.w_c *ltotaltildem ;
      
      // Compute Tilde CoM for this Node.
      if ((Ec) && (Ic!=-1))
	{
	  
	  double lIctildem = listOfBodies[Ic].m_tildem;
	  ltotaltildem += lIctildem;
	  double lIctildem_sister = listOfBodies[Ic].m_tildem_sister;
	  ltotaltildem += lIctildem_sister;

	  ltildec += (listOfBodies[Ic].m_tildec * lIctildem) +
	    (listOfBodies[Ic].m_tildec_sister * lIctildem_sister);
	  /*
	  cout << "Ic " << Ic << " Ec " << Ec << endl;
	  cout << "listOfBodies[Ic].m_tildec " << listOfBodies[Ic].m_tildec << endl;
	  cout << "listOfBodies[Ic].m_tildec_sister " << listOfBodies[Ic].m_tildec_sister << endl;
	  cout << "ltotaltildem "<< ltotaltildem <<endl;
	  */
	}
      aDB.m_tildec = ltildec * (1.0/ltotaltildem);

      //      cout << "Up here 2" << endl;      
      // Compute TildeCom for the tree having the sister as root.
      if ((Es) && (Is!=-1))
	{
	  double l2totaltildem = 0.0;
	  double lIctildem = listOfBodies[Is].m_tildem;
	  l2totaltildem += lIctildem;
	  double lIctildem_sister = listOfBodies[Is].m_tildem_sister;
	  l2totaltildem += lIctildem_sister;

	  aDB.m_tildec_sister += 
	    (listOfBodies[Is].m_tildec * lIctildem +
	    listOfBodies[Is].m_tildec_sister * lIctildem_sister)*(1.0/l2totaltildem) ;
	}
      
      //      cout << "Up here 3" << endl;
      // Setting variables for the depth first exploration
      aDB.setExplored(1);
      //cout << "Node : " << currentNode << " Name " << aDB.getName() << endl; 
      //cout << "Tilde m: " << aDB.m_tildem << endl;
      //cout << "Tilde c: " << aDB.m_tildec << endl;
      
      listOfBodies[currentNode] = aDB;
      currentNode = aDB.getLabelMother();
      //      cout << "currentNode :" << currentNode << endl;
      
    }
  while(currentNode!=-1);
  //  cout << "Masse du corps: " << listOfBodies[0].m_tildem << endl;

}

MAL_S3x3_MATRIX(,double) DynamicMultiBody::D(MAL_S3_VECTOR(,double) &r)
{
  MAL_S3x3_MATRIX( res,double);

  res(0,0) = r[0]*r[0];
  res(0,1) = r[0]*r[1];
  res(0,2) = r[0]*r[2];
  
  res(1,0) = r[1]*r[0];
  res(1,1) = r[1]*r[1];
  res(1,2) = r[1]*r[2];

  res(2,0) = r[2]*r[0];
  res(2,1) = r[2]*r[1];
  res(2,2) = r[2]*r[2];
  
  return res;
}

void DynamicMultiBody::InertiaMatricesforRMCSecondStep()
{

  double norm_w;
  //  int NbOfNodes=1;
  int currentNode = labelTheRoot;
  int lMother=0;
  MAL_S3x3_MATRIX( Ro,double);
  MAL_S3x3_MATRIX(w_wedge,double);
  
  currentNode = listOfBodies[labelTheRoot].child;
  //  cout << "STARTING FORWARD VELOCITY " << v0ForRoot << endl;

  currentNode = listOfBodies[labelTheRoot].child;
  for(unsigned int i=0;i<listOfBodies.size();i++)
    {
      listOfBodies[i].setExplored(0);
    }
  do
    {

      DynamicBody aDB = listOfBodies[currentNode];

      norm_w = MAL_S3_VECTOR_NORM(aDB.a);
      lMother = aDB.getLabelMother();

      
      int NextNode=0;
      int lContinue = 1;
      bool Ec=false, Es=false;
      int Ic=-1, Is=-1;

      do{

	Ec=false; Es=false;
	// Test if the node should be compute.
	Ic = aDB.child;
	Is = aDB.sister;
	//cout << "Ic : "<< Ic << " Is:" << Is << endl;
	if (!aDB.getExplored())
	  {
	    // Leaf
	    if ((Ic==-1) && (Is==-1))
	      {
		Es=Ec=true;
		lContinue =0;
	      }
	    else 
	      {
		
		if (Ic!=-1)
		  {
		    //cout << "IC: " << Ic << " " << (int) listOfBodies[Ic].getExplored() << endl;
		    if (listOfBodies[Ic].getExplored())
		      Ec = true;
		  }
		else Ec = true;
		
		if (Is!=-1)
		  {
		    //cout << "IS: " << Is << " " << (int) listOfBodies[Is].getExplored() << endl;
		    if (listOfBodies[Is].getExplored())
		      Es =true;
		  }
		else Es =true;
		
		if (Es && Ec)
		  lContinue = 0;
	      }
	  }

	// Depth first exploration
	if (lContinue)
	  {
	    if ((Ic!=-1) && (!Ec))
	      NextNode = aDB.child;
	    else 
	      NextNode = aDB.sister;
	    currentNode = NextNode;

	  } 

	aDB = listOfBodies[currentNode];
      }
      while (lContinue);
      
      //cout << "Body evaluated " << aDB.getName() <<endl;
      // Compute Tilde inertia matrix.
      MAL_S3x3_MATRIX(tmp,double);
      MAL_S3x3_MATRIX(tmp2,double);
      tmp = aDB.R;
      tmp = MAL_S3x3_RET_TRANSPOSE(tmp);
      MAL_S3x3_C_eq_A_by_B(tmp2,aDB.R,aDB.getInertie());
      tmp = MAL_S3x3_RET_A_by_B(tmp2,tmp);
      
      MAL_S3_VECTOR(diff_vec,double);
      diff_vec = aDB.w_c - aDB.m_tildec; 

      MAL_S3x3_MATRIX(diff_mat,double);
      diff_mat = D(diff_vec) * aDB.getMasse();
      diff_mat = tmp;
      
      aDB.m_tildeI = diff_mat;
      if ((Ec) && (Ic!=-1))
	{
	  MAL_S3_VECTOR(diff_vec,double);
	  diff_vec = listOfBodies[Ic].w_c - aDB.m_tildec; 
	  MAL_S3x3_MATRIX(tmp2,double);
	  tmp2 = D(diff_vec);

	  /*
	  aDB.m_tildeI +=  listOfBodies[Ic].m_tildeI +
	    tmp2 * listOfBodies[Ic].m_tildem + 
	    listOfBodies[Ic].m_tildeI_sister +  
	    listOfBodies[Ic].m_Dsister * listOfBodies[Ic].m_tildem_sister; */
	  aDB.m_tildeI +=  listOfBodies[Ic].m_tildeI;
	  aDB.m_tildeI +=  tmp2 * listOfBodies[Ic].m_tildem;
	  aDB.m_tildeI +=  tmp2 * listOfBodies[Ic].m_tildeI_sister;
	  aDB.m_tildeI +=  listOfBodies[Ic].m_Dsister * listOfBodies[Ic].m_tildem_sister;
	    
	}

      if ((Es) && (Is!=-1))
	{
	  int lMother = aDB.getLabelMother();
	  MAL_S3_VECTOR(diff_vec,double);
	  diff_vec = listOfBodies[Is].w_c - listOfBodies[lMother].m_tildec; 
	  MAL_S3x3_MATRIX(tmp2,double);
	  tmp2 = D(diff_vec);
	  
	  aDB.m_Dsister = tmp2;
	  aDB.m_tildeI_sister =  listOfBodies[Is].m_tildeI 
	    + ( tmp2 * listOfBodies[Is].m_tildem )
	    + listOfBodies[Is].m_tildeI_sister 
	    + (listOfBodies[Is].m_Dsister*  listOfBodies[Is].m_tildem_sister );

	}


      // Compute the two inertia matrices following Kajita 2003 IROS, p1647
      
      MAL_S3_VECTOR_CROSS_PRODUCT(aDB.m_RMC_m ,
				  aDB.a,
				  ( (aDB.m_tildec - aDB.p) * 
				    aDB.m_tildem)); // Eq 18

      MAL_S3_VECTOR(h0,double);
      MAL_S3_VECTOR_CROSS_PRODUCT(h0,
				  aDB.m_tildec,
				  aDB.m_RMC_m);  
      h0= h0 + MAL_S3x3_RET_A_by_B(aDB.m_tildeI, aDB.a); // Eq 19

      aDB.m_RMC_h = h0 - MAL_S3x3_RET_A_by_B(SkewCoM,aDB.m_RMC_m); // Eq 21
      
      //cout << "Mtilde :" << aDB.m_RMC_m << " " << aDB.m_RMC_h <<endl;
      // Computation of the last 
      // Setting variables for the depth first exploration
      aDB.setExplored(1);
      /*      cout << "Node : " << currentNode << " Name " << aDB.getName() << endl;
      cout << "RMC m: " << aDB.m_RMC_m << endl;
      cout << "RMC h: " << aDB.m_RMC_h << endl;
      */
      listOfBodies[currentNode] = aDB;
      currentNode = aDB.getLabelMother();
    }
  while(currentNode!=-1);
  
  //  cout << "Masse du corps: " << listOfBodies[0].m_tildem << endl;
}

void DynamicMultiBody::ForwardDynamics(int corpsCourant, int liaisonDeProvenance)
{
  // Building the appropriate transformations.
  if (listeLiaisons[liaisonDeProvenance].indexCorps1 == corpsCourant) 
    {
      empilerTransformationsLiaisonInverse(liaisonDeProvenance);
    }
  else 
    {
      empilerTransformationsLiaisonDirecte(liaisonDeProvenance);
    }
  
  
  if (liaisonDeProvenance == 0) { //a revoir
    masse = 0;
    MAL_S3_VECTOR_FILL(positionCoMPondere,0);
  }
  
  //  double matrice[16];
  //  glGetDoublev(GL_MODELVIEW_MATRIX, matrice);
  //  positionCoMPondere += (matrice*(listeCorps[corpsCourant].getPositionCoM()))*listeCorps[corpsCourant].getMasse();
  masse += listeCorps[corpsCourant].getMasse();
  //listeCorps[corpsCourant].dessiner(alpha);
  
  switch (liaisons[corpsCourant].size()) {
  case 1 :
    break;
  case 2 :
    if (liaisons[corpsCourant][0].liaison == liaisonDeProvenance) {
      int liaisonDestination = liaisons[corpsCourant][1].liaison;
      int corps1 = listeLiaisons[liaisonDestination].indexCorps1;
      int corps2 = listeLiaisons[liaisonDestination].indexCorps2;
      if (corpsCourant == corps1) {
	ForwardDynamics(corps2, liaisonDestination);
      }
      else {
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

inline void DynamicMultiBody::empilerTransformationsLiaisonDirecte(int liaison)
{
}

inline void DynamicMultiBody::empilerTransformationsLiaisonInverse(int liaison)
{
}

void DynamicMultiBody::parserVRML(string path, 
				  string nom, 
				  const char *option)
{
  listOfBodies.clear();
  MultiBody::parserVRML(path, nom, option);
  listOfBodies.resize(listeCorps.size());
  for(unsigned int i=0;i<listeCorps.size();i++)
    listOfBodies[i] = listeCorps[i];

  ConvertIDINVRMLToBodyID.resize(listOfBodies.size());
  SpecifyTheRootLabel(0);
  ComputeNumberOfJoints();
  ReadSpecificities(option);
  BuildStateVectorToJointAndDOFs();
  UpdateTheSizeOfJointsJacobian();

  MAL_VECTOR_RESIZE(m_Configuration,m_NbDofs);
  MAL_VECTOR_RESIZE(m_Velocity,m_NbDofs);
  MAL_VECTOR_RESIZE(m_Acceleration,m_NbDofs);
}

void DynamicMultiBody::calculerMatriceTransformationEntre(int corps1, int corps2, float *matrice)
{
  vector<int> v = trouverCheminEntre(corps1, corps2);
  if (v.size() == 0) {
    matrice = NULL;
    return;
  }

  //  glPushMatrix();
  //  glLoadIdentity();

  int corpsCourant = corps1;
  for (unsigned int i=0; i<v.size(); i++) {
    cout << "i = " << i << endl;
    //il s'agit de savoir si on lit la liaison dans le bon sens :
    //la liaison est oriente de indexCorps1 vers indexCorps2 pour ce qui est des transformations
    if (corpsCourant == listeLiaisons[v[i]].indexCorps1) {	
      // on est dans le bon sens
      empilerTransformationsLiaisonDirecte(v[i]);
      corpsCourant = listeLiaisons[v[i]].indexCorps2;
    }
    else {
      // il faut "lire" la liaison a l'envers
      empilerTransformationsLiaisonInverse(v[i]);
      corpsCourant = listeLiaisons[v[i]].indexCorps1;
    }
  }

  //  glGetFloatv(GL_MODELVIEW_MATRIX, matrice);
  for (int i=0; i<16; i++) {
    cout << matrice[i] << "  ";
  }
  cout << endl;
  //  glPopMatrix();
  return;
	
}

void DynamicMultiBody::calculerMatriceTransformationEntre(int corps1, int corps2, double *matrice)
{

  //	cout << "entree calculerMatrice" << endl;

  vector<int> v = trouverCheminEntre(corps1, corps2);
  if (v.size() == 0) {
    matrice = NULL;
    cout << "pas de chemin entre " << corps1 << " et " << corps2 << endl;
    return;
  }

  //  glPushMatrix();
  // glLoadIdentity();

  int corpsCourant = corps1;
  for (unsigned int i=0; i<v.size(); i++) 
    {
      //		cout << "i = " << i << endl;
      //il s'agit de savoir si on lit la liaison dans le bon sens :
      //la liaison est oriente de indexCorps1 vers indexCorps2 pour ce qui est des transformations
      if (corpsCourant == listeLiaisons[v[i]].indexCorps1) 
	{	
	  // on est dans le bon sens
	  empilerTransformationsLiaisonDirecte(v[i]);
	  corpsCourant = listeLiaisons[v[i]].indexCorps2;
	}
      else 
	{
	  // il faut "lire" la liaison a l'envers
	  empilerTransformationsLiaisonInverse(v[i]);
	  corpsCourant = listeLiaisons[v[i]].indexCorps1;
	}
      
  }
  //  glGetDoublev(GL_MODELVIEW_MATRIX, matrice);
  /*	for (int i=0; i<16; i++) {
	cout << matrice[i] << "  ";
	}
	cout << endl;*/
  //  glPopMatrix();
  return;
	
}

vector<int> DynamicMultiBody::trouverCheminEntre(int corps1, int corps2)
{
  if (corps1 < 0 || (unsigned int)corps1 >= listeCorps.size() || corps2 < 0 || (unsigned int)corps2 >= listeCorps.size()) {
    cout << "Indice hors limites lors de l'appel de trouverCheminEntre(" << corps1 << ", " << corps2 << ")" <<endl;
    return vector<int>();
  }
  for (unsigned int i=0; i<liaisons[corps1].size(); i++) {
    vector<int> v = vector<int>();
    int c1 = listeLiaisons[liaisons[corps1][i].liaison].indexCorps1;
    int c2 = listeLiaisons[liaisons[corps1][i].liaison].indexCorps2;
    trouverCheminEntreAux((c1==corps1)?c2:c1, corps2, liaisons[corps1][i].liaison, v);
    if (v.size() !=0 ) {
      return v;
    }
  }

  cout << "aucun chemin trouve entre le corps " << corps1 << " et le corps " << corps2 << endl;
  return vector<int>();
}

void DynamicMultiBody::trouverCheminEntreAux(int corpsCourant, int corpsVise, 
					     int liaisonDeProvenance, vector<int> &chemin)
{
  if (corpsCourant == corpsVise) {
    chemin.push_back(liaisonDeProvenance);
    return;
  }

  chemin.push_back(liaisonDeProvenance);

  switch (liaisons[corpsCourant].size()) {
  case 1 :
    break;
  case 2 :
    if (liaisons[corpsCourant][0].liaison == liaisonDeProvenance) {
      int liaisonDestination = liaisons[corpsCourant][1].liaison;
      int corps1 = listeLiaisons[liaisonDestination].indexCorps1;
      int corps2 = listeLiaisons[liaisonDestination].indexCorps2;
      if (corpsCourant == corps1) {
	trouverCheminEntreAux(corps2, corpsVise, liaisonDestination, chemin);
      }
      else {
	trouverCheminEntreAux(corps1, corpsVise, liaisonDestination, chemin);
      }
    }
    else {
      int liaisonDestination = liaisons[corpsCourant][0].liaison;
      int corps1 = listeLiaisons[liaisonDestination].indexCorps1;
      int corps2 = listeLiaisons[liaisonDestination].indexCorps2;
      if (corpsCourant == corps1) {
	trouverCheminEntreAux(corps2, corpsVise, liaisonDestination, chemin);
      }
      else {
	trouverCheminEntreAux(corps1, corpsVise, liaisonDestination, chemin);
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
	    trouverCheminEntreAux(corps2, corpsVise, liaisonDestination, chemin);
	  }
	else 
	  {
	    trouverCheminEntreAux(corps1, corpsVise, liaisonDestination, chemin);
	  }
      }
    break;
  }

  if (chemin.size() != 0) 
    {
      if (listeLiaisons[chemin[chemin.size()-1]].indexCorps1 == corpsVise || 
	  listeLiaisons[chemin[chemin.size()-1]].indexCorps2 == corpsVise) 
	{
	  return;
	}
    }

  chemin.pop_back();

}

void DynamicMultiBody::changerCorpsInitial(int nouveauCorps)
{
}

// TODO: Remove any reference to OpenGL and makes this function right.
int DynamicMultiBody::ComputeJacobian(int corps1, int corps2, 
				      MAL_S3_VECTOR(,double) coordLocales, 
				      double *jacobienne[6])
{
  vector<int> chemin = trouverCheminEntre(corps1, corps2);
  MAL_VECTOR(ve,double);
  double translationInit[3];
  double axe[3],angle=0.0;
  int l = chemin.size();
  if (l == 0) 
    {
      jacobienne = NULL;
      return 1;
    }

  double *matrice = new double[16];

  //  glPushMatrix();
  //  glLoadIdentity();
  //  glGetDoublev(GL_MODELVIEW_MATRIX, matrice);

  int corpsCourant = corps1;
  for (int i=0; i<l; i++) {
    /*		for (int j=0; j<16; j++) {
		cout << matrice[j] << "  ";
		}*/
    //		cout << endl;
    //		cout << "i = " << i << endl;
    //il s'agit de savoir si on lit la liaison dans le bon sens :
    //la liaison est oriente de indexCorps1 vers indexCorps2 pour ce qui est des transformations
    if (corpsCourant == listeLiaisons[chemin[i]].indexCorps1) {
      // on est dans le bon sens
	//				cout << "bon sens" << endl;
	Joint t = listeLiaisons[chemin[i]].aJoint;
	switch (t.type()) {
	case Joint::REVOLUTE_JOINT :
	  //					cout << "rotation : " << t.quantite << endl;
	  //ve = matrice*(t.axe);
	  jacobienne[3][i] = ve[0]-matrice[12];
	  jacobienne[4][i] = ve[1]-matrice[13];
	  jacobienne[5][i] = ve[2]-matrice[14];
	  break;
	case Joint::FREE_JOINT :
	  //	  Matrix2AxeAngle(t.rotation, axe, angle);
	  cout << t.pose() ;
	  cout << endl;
	  jacobienne[3][i] = axe[0];
	  jacobienne[4][i] = axe[1];
	  jacobienne[5][i] = axe[2];
	  cout << axe[0] << " " << axe[1] << " " << axe[3] << "  angle : " << angle << endl;
	  break;
	case Joint::PRISMATIC_JOINT :
	  //A faire
	  break;
	default :
	  cout << "attention, transformations non prises en charge lors du calcul de la jacobienne !" << endl;
	}
    
      empilerTransformationsLiaisonDirecte(chemin[i]);
      corpsCourant = listeLiaisons[chemin[i]].indexCorps2;
    }
    else 
      {
      // il faut "lire" la liaison a l'envers
	//				cout << "sens inverse" << endl;
	Joint t = listeLiaisons[chemin[i]].aJoint;
	switch (t.type()) {
	case Joint::REVOLUTE_JOINT :
	  //	  ve = matrice*(t.axe);
	  jacobienne[3][i] = -ve[0]+matrice[12];//rajouter +matrice[12] ?
	  jacobienne[4][i] = -ve[1]+matrice[13];
	  jacobienne[5][i] = -ve[2]+matrice[14];
	  break;
	case Joint::FREE_JOINT :
	  //	  Matrix2AxeAngle(t.rotation, axe, angle);
	  jacobienne[3][i] = -axe[0];
	  jacobienne[4][i] = -axe[1];
	  jacobienne[5][i] = -axe[2];
	  break;
	case Joint::PRISMATIC_JOINT :
	  //A faire
	  break;
	default :
	  cout << "attention, transformations non prises en charge lors du calcul de la jacobienne !" << endl;
	}

	empilerTransformationsLiaisonInverse(chemin[i]);
	corpsCourant = listeLiaisons[chemin[i]].indexCorps1;
      }
    //    glGetDoublev(GL_MODELVIEW_MATRIX, matrice);

	
    //recuperation des vecteurs
    if (i == 0) {
      /*				cout <<  "translation init" << endl;
					cout << (translationInit[0] = matrice[12]) << endl;
					cout << (translationInit[1] = matrice[13]) << endl;
					cout << (translationInit[2] = matrice[14]) << endl;*/
      translationInit[0] = matrice[12];
      translationInit[1] = matrice[13];
      translationInit[2] = matrice[14];
    }
    else {
      jacobienne[0][i-1] = matrice[12] - translationInit[0];
      jacobienne[1][i-1] = matrice[13] - translationInit[1];
      jacobienne[2][i-1] = matrice[14] - translationInit[2];
    }

    /*
      MAL_VECTOR_DIM(translationOriente,double,3)1(matrice[12],matrice[13],matrice[14]);
      MAL_VECTOR_DIM(translationOriente,double,3)2 = matrice*translationOriente1 - translationOriente1;

      jacobienne[0][i] = translationOriente1[0];
      jacobienne[1][i] = translationOriente1[1];
      jacobienne[2][i] = translationOriente1[2];
    */
  }
  //glPopMatrix();
  /*	cout << endl;
	for (int j=0; j<16; j++) {
	cout << matrice[j] << "  ";
	}
	cout << endl;
	cout << endl;*/
  //a present, il faut recalculer les vecteurs : on a en effet les trois premieres composantes de chaque 
  //colonnes de la jacobienne qui correspondent aux vecteurs depuis l'origine jusqu'a un joint, alors qu'on
  //cherche a avoir le vecteur de ce joint a l'extremite
  //MAL_VECTOR_DIM(point,double,3)(0,0,0);
  MAL_S3_VECTOR(point,double);
  point = coordLocales;
  //  point = matrice*point;
  for(int i=0;i<3;i++)
    cout << point[i] << " " ;
  cout << endl;
  point[0] -= translationInit[0];
  point[1] -= translationInit[1];
  point[2] -= translationInit[2];
  for(int i=0;i<3;i++)
    cout << point[i] << " " ;
  cout << endl;

  /*
    cout << endl;
    cout << "*-*-*" << endl;
    for (int i=0; i<6; i++) {
    for (int j=0; j<l; j++) {
    if (jacobienne[i][j]<0.000001 && jacobienne[i][j]>-0.000001) {
    cout << " " << 0 << "  ";
    }
    else {
    cout << (jacobienne[i][j]<0.000001?"":" ") << jacobienne[i][j] << "  ";
    }
    }
    cout << endl;
    }
    cout << endl;
  */

  for (int i=l-1; i>0; i--) {
    jacobienne[0][i] = point[0] - jacobienne[0][i-1];
    jacobienne[1][i] = point[1] - jacobienne[1][i-1];
    jacobienne[2][i] = point[2] - jacobienne[2][i-1];
  }
  jacobienne[0][0] = point[0];
  jacobienne[1][0] = point[1];
  jacobienne[2][0] = point[2];

  cout << endl;
  cout << "***" << endl;
  /*	for (int i=0; i<6; i++) {
	for (int j=0; j<l; j++) {
	if (jacobienne[i][j]<0.000001 && jacobienne[i][j]>-0.000001) {
	cout << " " << 0 << "  ";
	}
	else {
	cout << (jacobienne[i][j]<0.000001?"":" ") << jacobienne[i][j] << "  ";
	}
	}
	cout << endl;
	}
	cout << endl;
  */
  //on calcul les produits vectoriels (j[3],j[4],j[5])*(j[0],j[1],j[2])
  double pvx, pvy, pvz;
  for (int i=0; i<l; i++) {
    pvx = jacobienne[2][i]*jacobienne[4][i] - jacobienne[1][i]*jacobienne[5][i];
    pvy = jacobienne[0][i]*jacobienne[5][i] - jacobienne[2][i]*jacobienne[3][i];
    pvz = jacobienne[1][i]*jacobienne[3][i] - jacobienne[0][i]*jacobienne[4][i];
    jacobienne[0][i] = pvx;
    jacobienne[1][i] = pvy;
    jacobienne[2][i] = pvz;
  }


  for (int i=0; i<6; i++) {
    for (int j=0; j<l; j++) {
      if (jacobienne[i][j]<0.000001 && jacobienne[i][j]>-0.000001) {
	cout << " " << 0 << "  ";
      }
      else {
	cout << (jacobienne[i][j]<0.000001?"":" ") << jacobienne[i][j] << "  ";
      }
    }
    cout << endl;
  }

  return 0;
}

void DynamicMultiBody::ComputeJacobianWithPath(vector<int> aPath,MAL_MATRIX(&J,double))
{
  int last=aPath.back();
  //  cout << "last "<< last <<endl;
  MAL_S3_VECTOR(target,double);
  target = listOfBodies[ConvertIDINVRMLToBodyID[last]].p;
  //  cout << "target " << endl
  //       <<target << endl;
  for (unsigned int i=0;i<aPath.size();i++)
    {
      int li = aPath[i];
      //cout << "li " << li << endl;
      DynamicBody aDB = listOfBodies[ConvertIDINVRMLToBodyID[li]];


      MAL_S3_VECTOR(wa,double);
      MAL_S3x3_C_eq_A_by_B(wa , aDB.R, aDB.a);
      MAL_S3_VECTOR(cwa,double);
      MAL_S3_VECTOR_CROSS_PRODUCT(cwa ,wa, (target - aDB.p));
      for(int j=0;j<3;j++)
	J(j,i) = cwa[j];
      for(int j=0;j<3;j++)
	J(j+3,i) = wa[j];
      
    }
  //  cout << "J " << endl << J<< endl;

}

MAL_S3_VECTOR(,double) 
  DynamicMultiBody::getPositionPointDansRepere(MAL_S3_VECTOR(,double) point, 
					       int corpsDuPoint, int corpsDuRepere)
{
  //	cout << "entree getPosition" << endl;
  double matrice[16];
  calculerMatriceTransformationEntre(corpsDuRepere, corpsDuPoint, matrice);
  //  return (matrice*point);
  return point;
}

double DynamicMultiBody::Getq(int JointID) const
{
  if ((JointID>=0) &&
      ((unsigned int)JointID<listOfBodies.size()))
    return listOfBodies[ConvertIDINVRMLToBodyID[JointID]].q;
  return 0.0;
}

void DynamicMultiBody::Setq(int JointID,double q)
{
 if ((JointID>=0) &&
     ((unsigned int)JointID<listOfBodies.size()))
   {  
     listOfBodies[ConvertIDINVRMLToBodyID[JointID]].q= q;
     ((Joint *)listOfBodies[ConvertIDINVRMLToBodyID[JointID]].joint())->quantity(q);
   }
}

void DynamicMultiBody::Setdq(int JointID, double dq)
{
 if ((JointID>=0) &&
     ((unsigned int)JointID<listOfBodies.size()))
   {  
     listOfBodies[ConvertIDINVRMLToBodyID[JointID]].dq= dq;
   }
}

void DynamicMultiBody::Setv(int JointID, MAL_S3_VECTOR(,double) v0)
{
  if ((JointID>=0) &&
      ((unsigned int)JointID<listOfBodies.size()))
    listOfBodies[ConvertIDINVRMLToBodyID[JointID]].v0 = v0;
}

MAL_S3_VECTOR(,double) DynamicMultiBody::Getv(int JointID)
{
  if ((JointID>=0) &&
      ((unsigned int)JointID<listOfBodies.size()))
    return listOfBodies[ConvertIDINVRMLToBodyID[JointID]].v0;
  
  MAL_S3_VECTOR(dr,double);
  dr[0] = 0.0;  dr[1] = 0.0;  dr[2] = 0.0;
  return dr;
}

MAL_S3_VECTOR(,double) DynamicMultiBody::GetvBody(int BodyID)
{
  if ((BodyID>=0) &&
      ((unsigned int)BodyID<listOfBodies.size()))
    return listOfBodies[BodyID].v0;
  MAL_S3_VECTOR(dr,double);
  dr[0] = 0.0;  dr[1] = 0.0;  dr[2] = 0.0;
  return dr;
}

MAL_S3_VECTOR(,double) DynamicMultiBody::GetwBody(int BodyID)
{
  if ((BodyID>=0) &&
      ((unsigned int)BodyID<listOfBodies.size()))
    return listOfBodies[BodyID].w;
  MAL_S3_VECTOR(dr,double);
  dr[0] = 0.0;  dr[1] = 0.0;  dr[2] = 0.0;
  return dr;
}

MAL_S3_VECTOR(,double) DynamicMultiBody::Getw(int JointID)
{
  if ((JointID>=0) &&
      ((unsigned int)JointID<listOfBodies.size()))
    return listOfBodies[ConvertIDINVRMLToBodyID[JointID]].w;
 
  MAL_S3_VECTOR(dr,double);
  dr[0] = 0.0;  dr[1] = 0.0;  dr[2] = 0.0;
  return dr;
}

void DynamicMultiBody::Setw(int JointID, MAL_S3_VECTOR(,double) w)
{
  if ((JointID>=0) &&
      ((unsigned int)JointID<listOfBodies.size()))
    listOfBodies[ConvertIDINVRMLToBodyID[JointID]].w = w;
}

MAL_S3_VECTOR(,double) DynamicMultiBody::Getp(int JointID)
{
  MAL_S3_VECTOR(empty,double);
  if ((JointID>=0) &&
      ((unsigned int)JointID<listOfBodies.size()))
    return listOfBodies[ConvertIDINVRMLToBodyID[JointID]].p;
  return empty;
}

void DynamicMultiBody::Setp(int JointID, MAL_S3_VECTOR(,double) apos)
{
  if ((JointID>=0) &&
      ((unsigned int)JointID<listOfBodies.size()))
    listOfBodies[ConvertIDINVRMLToBodyID[JointID]].p = apos;
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

  //  cout << "px :"<< px<< " py: " << py<< endl;

}

string DynamicMultiBody::GetName(int JointID)
{
  string empty;
  if ((JointID>=0) &&
      ((unsigned int)JointID<listOfBodies.size()))
    return listOfBodies[ConvertIDINVRMLToBodyID[JointID]].getName();
  return empty;
    
}

CjrlJoint* DynamicMultiBody::GetJointFromVRMLID(int JointID)
{
#if 0
  if ((JointID>=0) && 
      ((unsigned int)JointID<listOfBodies.size()))
    return (CjrlJoint<MAL_MATRIX(,double), MAL_S4x4_MATRIX(,double),MAL_S3x3_MATRIX(,double),
	    MAL_VECTOR(,double),MAL_S3_VECTOR(,double)> *)listOfBodies[ConvertIDINVRMLToBodyID[JointID]].joint();
#else
  for(int i=0;i<m_JointVector.size();i++)
    {
      Joint * r;
      if (((r=(Joint *)m_JointVector[i])->getIDinVRML())==JointID)
	{
	  ODEBUG("Joint : "<< r->getName() << " " << JointID );
	  
	  return r;
	}
    }
#endif
  return 0;
}

MAL_S3_VECTOR(,double) DynamicMultiBody::GetL(int JointID)
{
  MAL_S3_VECTOR(empty,double);
  if ((JointID>=0) &&
      ((unsigned int)JointID<listOfBodies.size()))
    return listOfBodies[ConvertIDINVRMLToBodyID[JointID]].L;
  return empty;
}

MAL_S3_VECTOR(,double) DynamicMultiBody::GetP(int JointID)
{
  MAL_S3_VECTOR(empty,double);
  if ((JointID>=0) &&
      ((unsigned int)JointID<listOfBodies.size()))
    return listOfBodies[ConvertIDINVRMLToBodyID[JointID]].P;
  return empty;
}

double DynamicMultiBody::Getdq(int JointID) const
{
  double empty=0.0;
  if ((JointID>=0) &&
      ((unsigned int)JointID<listOfBodies.size()))
    return listOfBodies[ConvertIDINVRMLToBodyID[JointID]].dq;
  return empty;
}

void DynamicMultiBody::BuildSplittedInertialMatrices(  vector<int> LeftLeg, vector<int> RightLeg,
						       int WaistIndex, vector<int> FreeJoints)
{
  MAL_MATRIX(aJacobian,double);
  MAL_MATRIX(IaJacobian,double);
  MAL_MATRIX(MHlegi,double);
  // Build M*Fi and H*Fi

  MAL_MATRIX_RESIZE(aJacobian,6,LeftLeg.size());

  ComputeJacobianWithPath(LeftLeg,aJacobian);


  //  cout << "Jacobian Left Foot"<< aJacobian << endl;
  //  cout << "Inverse Jacobian"<< Inverse(aJacobian) << endl;
  MAL_MATRIX_RESIZE(MHlegi,6,LeftLeg.size());
  int lindex;
  for(unsigned int i=0;i<LeftLeg.size();i++)
    {
      lindex =ConvertIDINVRMLToBodyID[LeftLeg[i]];
      //cout << listOfBodies[lindex].getName() <<endl;
      // M 
      MHlegi(0,i) = listOfBodies[lindex].m_RMC_m[0];
      MHlegi(1,i) = listOfBodies[lindex].m_RMC_m[1];
      MHlegi(2,i) = listOfBodies[lindex].m_RMC_m[2];
      // H
      MHlegi(3,i) = listOfBodies[lindex].m_RMC_h[0];
      MHlegi(4,i) = listOfBodies[lindex].m_RMC_h[1];
      MHlegi(5,i) = listOfBodies[lindex].m_RMC_h[2];
      
    }
  //  cout << "MH for LeftFoot"<< MHlegi << endl;
  MAL_INVERSE(aJacobian, m_ILeftJacobian,double);

  MAL_C_eq_A_by_B(m_MHStarLeftFoot, MHlegi, m_ILeftJacobian);

  MAL_MATRIX_RESIZE(aJacobian,6,RightLeg.size());
  ComputeJacobianWithPath(RightLeg,aJacobian);
  //  cout << "Jacobian Right Foot"<< aJacobian << endl;
  //  cout << "Inverse Jacobian"<< Inverse(aJacobian) << endl;

  MAL_MATRIX_RESIZE(MHlegi,6,RightLeg.size());
  for(unsigned int i=0;i<RightLeg.size();i++)
    {
      lindex = ConvertIDINVRMLToBodyID[RightLeg[i]];
      //      cout << listOfBodies[lindex].getName() <<endl;
      // M 
      MHlegi(0,i) = listOfBodies[lindex].m_RMC_m[0];
      MHlegi(1,i) = listOfBodies[lindex].m_RMC_m[1];
      MHlegi(2,i) = listOfBodies[lindex].m_RMC_m[2];
      // H
      MHlegi(3,i) = listOfBodies[lindex].m_RMC_h[0];
      MHlegi(4,i) = listOfBodies[lindex].m_RMC_h[1];
      MHlegi(5,i) = listOfBodies[lindex].m_RMC_h[2];
      
    }
  //  cout << "MH for RightFoot"<< MHlegi << endl;
  //  cout << "MH for LeftFoot"<< MHlegi << endl;

  MAL_MATRIX(mIRightJacobian,double);
  MAL_INVERSE(aJacobian, mIRightJacobian,double);
  MAL_C_eq_A_by_B( m_MHStarRightFoot, MHlegi, m_IRightJacobian);

  //  cout << " m_MHStarLeftFoot" << m_MHStarLeftFoot << endl;
  //cout << " m_MHStarRightFoot" << m_MHStarRightFoot << endl;

  // Create M*B

  // Create the variables of equation 10 et 11 given pages 1645
  MAL_S3_VECTOR(rbCoM,double);
  rbCoM = positionCoMPondere - listOfBodies[WaistIndex].p;
  
  //  cout << "rbCoM : " << endl << rbCoM << " "
  //       << positionCoMPondere << endl <<  listOfBodies[WaistIndex].p << endl;
  MAL_MATRIX_DIM(FirstTerm,double,6,6);
  
  FirstTerm(0,0) =   FirstTerm(1,1) =  FirstTerm(2,2) =  masse;
  
  for(unsigned i=0;i<3;i++)
    for(unsigned j=0;j<3;j++)
      FirstTerm(i+3,j+3) = listOfBodies[labelTheRoot].m_tildeI(i,j);
  
  FirstTerm(0,3) = 0;              FirstTerm(0,4) = masse* rbCoM[2]; FirstTerm(0,5) = -masse * rbCoM[1];
  FirstTerm(1,3) = -masse*rbCoM[2]; FirstTerm(1,4) =              0; FirstTerm(1,5) =  masse * rbCoM[0];
  FirstTerm(2,3) =  masse*rbCoM[1]; FirstTerm(2,4) =-masse* rbCoM[0]; FirstTerm(2,5) =                0;

  MAL_MATRIX_DIM(SecondTerm,double,6,6);
  MAL_MATRIX_DIM(ThirdTerm,double,6,6);

  MAL_S3_VECTOR(rbfi,double);
  rbfi = listOfBodies[ConvertIDINVRMLToBodyID[LeftLeg.back()]].p - listOfBodies[WaistIndex].p;
  for(unsigned i=0;i<3;i++)
    for(unsigned j=0;j<3;j++)
      if (i==j)
	SecondTerm(i,j) = 1.0;
      else 
	SecondTerm(i,j) = 0.0;

  SecondTerm(0,3) =       0; SecondTerm(0,4) = -rbfi[2]; SecondTerm(0,5) = rbfi[1];
  SecondTerm(1,3) =  rbfi[2]; SecondTerm(1,4) =       0; SecondTerm(1,5) =-rbfi[0];
  SecondTerm(2,3) = -rbfi[1]; SecondTerm(2,4) =  rbfi[0]; SecondTerm(2,5) =       0;
  // To compute the leg joint velocity... hello ramzi ! .. did you find me ?
  m_ERBFI_Left = SecondTerm;
  
  MAL_C_eq_A_by_B(ThirdTerm,m_MHStarLeftFoot,SecondTerm);
  ThirdTerm = -1.0 * ThirdTerm;


  rbfi = listOfBodies[ConvertIDINVRMLToBodyID[RightLeg.back()]].p - listOfBodies[WaistIndex].p;
  for(unsigned i=0;i<3;i++)
    for(unsigned j=0;j<3;j++)
      if (i==j)
	SecondTerm(i,j) = 1.0;
      else 
	SecondTerm(i,j) = 0.0;

  SecondTerm(0,3) =       0; SecondTerm(0,4) = -rbfi[2]; SecondTerm(0,5) = rbfi[1];
  SecondTerm(1,3) =  rbfi[2]; SecondTerm(1,4) =       0; SecondTerm(1,5) =-rbfi[0];
  SecondTerm(2,3) = -rbfi[1]; SecondTerm(2,4) =  rbfi[0]; SecondTerm(2,5) =       0;
  // To compute the leg joint velocity... hello ramzi ! .. did you find me (again)?
  m_ERBFI_Right = SecondTerm;

  MAL_MATRIX(tmp3rdTerm,double);
  MAL_C_eq_A_by_B(tmp3rdTerm, m_MHStarRightFoot,SecondTerm);
  ThirdTerm = ThirdTerm - tmp3rdTerm;

  ODEBUG("FirstTerm " << endl << FirstTerm);
  ODEBUG("ThirdTerm " << endl << ThirdTerm);
  m_MHStarB = FirstTerm + ThirdTerm;

  /*  cout << "MHStarB" <<endl 
      << m_MHStarB << endl;
  */
  MAL_MATRIX_RESIZE(m_MHFree,6,FreeJoints.size());

  for(unsigned int i=0;i<FreeJoints.size();i++)
    {
      // M 
      m_MHFree(0,i) = listOfBodies[ConvertIDINVRMLToBodyID[FreeJoints[i]]].m_RMC_m[0];
      m_MHFree(1,i) = listOfBodies[ConvertIDINVRMLToBodyID[FreeJoints[i]]].m_RMC_m[1];
      m_MHFree(2,i) = listOfBodies[ConvertIDINVRMLToBodyID[FreeJoints[i]]].m_RMC_m[2];
      // H
      m_MHFree(3,i) = listOfBodies[ConvertIDINVRMLToBodyID[FreeJoints[i]]].m_RMC_h[0];
      m_MHFree(4,i) = listOfBodies[ConvertIDINVRMLToBodyID[FreeJoints[i]]].m_RMC_h[1];
      m_MHFree(5,i) = listOfBodies[ConvertIDINVRMLToBodyID[FreeJoints[i]]].m_RMC_h[2];
      
    }

  //  cout << "MHFree " <<endl
  //    << m_MHFree << endl;
}

void DynamicMultiBody::BuildLinearSystemForRMC(MAL_MATRIX( &PLref, double),
					       MAL_MATRIX(&XiLeftFootRef,double),
					       MAL_MATRIX(&XiRightFootRef,double),
					       int NbOfFreeJoints,
					       MAL_MATRIX(&S,double),
					       MAL_MATRIX(&XiBdThetaFreeRef,double),
					       MAL_MATRIX(&XiBdThetaFree,double),
					       MAL_MATRIX(&LeftLegVelocity,double),
					       MAL_MATRIX(&RightLegVelocity,double))
{
  // Building y (eq 10 p 1646, Kajita IROS 2003)
  MAL_MATRIX_DIM(TmpVector,double, 6,1);
  MAL_MATRIX_DIM(Tmp2,double,6,1);
  MAL_MATRIX_DIM(Tmp3,double,6,1);
  MAL_MATRIX_DIM(Tmp4,double,6,1);
  
  MAL_MATRIX_CLEAR(TmpVector);

  //  cout << "PLref:" <<endl << PLref << endl;
  Tmp2 = PLref; 

  //  cout << "Tmp2 1: " << Tmp2 << endl;
  
  Tmp2 -= MAL_RET_A_by_B(m_MHStarLeftFoot,XiLeftFootRef);
  //  cout << "Tmp2 2: " << Tmp2 << endl;
  Tmp2 -= MAL_RET_A_by_B(m_MHStarRightFoot, XiRightFootRef);
  //  cout << "Tmp2 3: " << Tmp2 << endl;

  MAL_MATRIX_RESIZE(TmpVector, 
		    MAL_MATRIX_NB_ROWS(Tmp2),
		    MAL_MATRIX_NB_COLS(Tmp2));

  for(unsigned int i=0;i<6;i++)
    TmpVector(i,0) = Tmp2(i,0);

  //  cout << TmpVector<< endl;

  MAL_MATRIX(y,double);
  y = MAL_RET_A_by_B(S , TmpVector);
 
  //  cout << "y " << endl << y << endl;
  cout << "m_MHStarB " << endl
       << m_MHStarB << endl;
  cout << "m_MHFree " << endl
       << m_MHFree << endl;

  MAL_MATRIX_DIM(TmpMatrix,double,6, 6+ NbOfFreeJoints);

  for(unsigned int i=0;i<6;i++)
    {
      for(unsigned int j=0;j<6;j++)
	{
	  TmpMatrix(i,j) = m_MHStarB(i,j);
	}
      for(unsigned int j=0;j<(unsigned int)NbOfFreeJoints;j++)
	{
	  TmpMatrix(i,j+5) = m_MHFree(i,j);
	}
    }

  MAL_MATRIX(A,double);

  A = MAL_RET_A_by_B(S , TmpMatrix);
  
  cout << "S " << endl << S << endl;
  cout << "TmpMatrix " << endl << TmpMatrix << endl;
  cout << "A " << endl << A << endl;
  
  MAL_MATRIX(PinvA,double);
  MAL_PSEUDOINVERSE(A,PinvA,double);

  //  cout << "y" << endl << y << endl;
  //  cout << "PinvA: " << endl << PinvA<< endl;
  
  MAL_MATRIX(KernA,double);
  MAL_MATRIX_DIM(E,double,
		 MAL_MATRIX_NB_ROWS(PinvA),
		 MAL_MATRIX_NB_COLS(A));
  for(unsigned int i=0;i<MAL_MATRIX_NB_ROWS(E);i++)
    for(unsigned int j=0;j<MAL_MATRIX_NB_COLS(E);j++)
      if (i==j)
	E(i,j)=1.0;
      else
	E(i,j)=0.0;

  KernA = E - MAL_RET_A_by_B(PinvA , A);
  
  MAL_MATRIX_DIM(lXiBdThetaFree,double,
		 MAL_MATRIX_NB_ROWS(XiBdThetaFreeRef),
		 MAL_MATRIX_NB_COLS(XiBdThetaFreeRef));
  
    //cout << "KernA" << endl << KernA.Rows() << " " << KernA.Columns() << endl;
  //  cout << "KernA * lXiBdThetaFreeRef" << endl << KernA* lXiBdThetaFreeRef << endl;
  lXiBdThetaFree = 
    MAL_RET_A_by_B(PinvA , y)
    + MAL_RET_A_by_B(KernA , XiBdThetaFreeRef);

  MAL_MATRIX_DIM(XiBd,double,6,1);
  MAL_MATRIX_RESIZE(XiBdThetaFree,
		    MAL_MATRIX_NB_ROWS(lXiBdThetaFree),
		    MAL_MATRIX_NB_COLS(lXiBdThetaFree));

  for(unsigned int i=0;i<MAL_MATRIX_NB_ROWS(lXiBdThetaFree);i++)
    for(unsigned int j=0;j<MAL_MATRIX_NB_COLS(lXiBdThetaFree);j++)
      {
	XiBdThetaFree(i,j)=lXiBdThetaFree(i,j);
	if ((i<6) && (j==0))
	  XiBd(i,j) = lXiBdThetaFree(i,j);
	  
      }

  // Computes the new leg joints velocities. (eq 3) 
  LeftLegVelocity = MAL_RET_A_by_B(m_ERBFI_Left , XiBd);
  LeftLegVelocity = MAL_RET_A_by_B(m_ILeftJacobian , XiLeftFootRef)
    - MAL_RET_A_by_B(m_ILeftJacobian ,LeftLegVelocity);

  RightLegVelocity = MAL_RET_A_by_B(m_ERBFI_Right , XiBd);
  RightLegVelocity = MAL_RET_A_by_B(m_IRightJacobian , XiRightFootRef) 
    - MAL_RET_A_by_B(m_IRightJacobian ,RightLegVelocity);

}

void DynamicMultiBody::SetRBody(int BodyID, MAL_S3x3_MATRIX(,double) R)
{
  if ((BodyID>=0) &&
      ((unsigned int)BodyID<listOfBodies.size()))  
    listOfBodies[BodyID].R = R;
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

void DynamicMultiBody::ComputeNumberOfJoints()
{
  int r=0;
  ODEBUG("JointVector :" << m_JointVector.size());
  for(int i=0;i<m_JointVector.size();i++)
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
    }
  fclose(fp);
}

int DynamicMultiBody::JointRankFromName(Joint *aJoint)
{

  for(int i=0;i<m_LinksBetweenJointNamesAndRank.size();i++)
    {
      if (!strcmp(m_LinksBetweenJointNamesAndRank[i].LinkName,(char *)aJoint->getName().c_str()))
	return m_LinksBetweenJointNamesAndRank[i].RankInConfiguration;
    }
  return -1;
}

Joint * DynamicMultiBody::JointFromRank(int aRank)
{
  string JointName;
  for(int i=0;i<m_LinksBetweenJointNamesAndRank.size();i++)
    {
      if (m_LinksBetweenJointNamesAndRank[i].RankInConfiguration==aRank)
	JointName = m_LinksBetweenJointNamesAndRank[i].LinkName;
    }
  for(int i=0;i<m_JointVector.size();i++)
    {
      if (((Joint *)m_JointVector[i])->getName()==JointName)
	return (Joint *)m_JointVector[i];
    }
}
void DynamicMultiBody::BuildStateVectorToJointAndDOFs()
{
  m_StateVectorToJoint.resize(m_NbDofs);
  int lindex=0;
  for(int i=0;i<m_JointVector.size();i++)
    {
      lindex = JointRankFromName((Joint *)m_JointVector[i]);
      for(int j=0;j<m_JointVector[i]->numberDof();j++)
	m_StateVectorToJoint[lindex++]=i;
    }
  
  
  m_StateVectorToDOFs.clear();
  m_StateVectorToDOFs.resize(m_NbDofs);
  lindex=0;
  for(int i=0;i<m_JointVector.size();i++)
    {
      lindex = JointRankFromName((Joint *)m_JointVector[i]);
      ODEBUG(((Joint *)m_JointVector[i])->getName() << " " << lindex);
      for(int j=0;j<m_JointVector[i]->numberDof();j++)
	m_StateVectorToDOFs[lindex++]=j;
    }

  m_VRMLIDToConfiguration.resize(m_NbOfVRMLIDs+1);
  for(int i=0;i<m_StateVectorToJoint.size();)
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

}

void DynamicMultiBody::UpdateTheSizeOfJointsJacobian()
{
  for(int i=0;i<m_JointVector.size();i++)
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
    MAL_VECTOR_RESIZE(m_Configuration,MAL_VECTOR_SIZE(inConfig));

  // Copy the configuration
  for(unsigned int i=0;i<MAL_VECTOR_SIZE(inConfig);i++)
    m_Configuration(i)= inConfig(i);

  ODEBUG("Went through here");
  int lindex=0;
  for (int i=0;i<m_JointVector.size();i++)
    {
      lindex = m_JointVector[i]->rankInConfiguration();

      // Update the pose of the joint when it is a free joint
      if (m_JointVector[i]->numberDof()==6)
	{
	  MAL_VECTOR_DIM(a6DPose,double,6);
	  for(int j=0;j<6;j++)
	    a6DPose(j) = inConfig[lindex++];
	  
	  ((Joint *)m_JointVector[i])->UpdatePoseFrom6DOFsVector(a6DPose);	  
	}
      // Update only the quantity when it is a revolute or a prismatic joint.
      else 
	{
	  int lIDinVRML = ((Joint *)m_JointVector[i])->getIDinVRML();
	  ((Joint *)m_JointVector[i])->quantity(inConfig[lindex]);
	  listOfBodies[ConvertIDINVRMLToBodyID[lIDinVRML]].q = inConfig[lindex];

	  lindex++;
	}
      
    } 

  if (0)
  {
    for(int i=0;i<listOfBodies.size();i++)
      {
	cout << listOfBodies[i].q * 180/M_PI << endl;
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
  for (int i=0;i<m_JointVector.size();i++)
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
	      listOfBodies[ConvertIDINVRMLToBodyID[lIDinVRML]].dq = inVelocity[lindex];
	      // Update dq.
	      /* ODEBUG(" Id (Vs) :" << lindex 
		 << " Id (JV) : " << i 
		 << " Id (VRML): " << lIDinVRML 
		 << " Id (Body): " << ConvertIDINVRMLToBodyID[lIDinVRML]
		 << " dq: " << listOfBodies[ConvertIDINVRMLToBodyID[lIDinVRML]].dq ); */
	    }

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
  
    MAL_VECTOR_RESIZE(m_Acceleration,MAL_VECTOR_SIZE(inAcceleration));
  
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
    \brief Apply a configuration

    This method updates the joints transformations only, according to the passed configuration vector.
    \return true if success, false if failure (the dimension of the
    input vector does not fit the number of degrees of freedom of the
    robot).
 */
bool DynamicMultiBody::applyConfiguration(const vectorN& inConfiguration)
{
    if (MAL_VECTOR_SIZE(inConfiguration)!=
        MAL_VECTOR_SIZE(m_Configuration))
        return false;
    
    currentConfiguration(inConfiguration);
    
    MAL_S3_VECTOR_FILL(positionCoMPondere,0);
    
    forwardTransformation(m_RootOfTheJointsTree);
    
    positionCoMPondere = positionCoMPondere/masse;
}
/**
\brief Recursive method to update the kinematic tree transformations starting from the given joint
 */
void DynamicMultiBody::forwardTransformation(Joint* inJoint)
{
    DynamicBody* body = dynamic_cast<DynamicBody*>(inJoint->linkedBody());

    if (inJoint != m_RootOfTheJointsTree)
    {

        RodriguesRotation(body->a,body->q,localR);
        
        DynamicBody* parentbody = dynamic_cast<DynamicBody*>(inJoint->parentJoint().linkedBody());

        MAL_S3x3_C_eq_A_by_B(body->R ,parentbody->R , localR);
        body->p = parentbody->p + MAL_S3x3_RET_A_by_B(parentbody->R,body->b);
    }
    
    //update position of center of mass in world frame
    MAL_S3x3_C_eq_A_by_B(wn3d, body->R, body->c);
    body->w_c = wn3d + body->p;
    positionCoMPondere += body->w_c * body->getMasse();
    
    for (unsigned int i = 0; i<inJoint->countChildJoints(); i++)
    {
        CjrlJoint* jrlchildJoint = const_cast<CjrlJoint*>(&(inJoint->childJoint(i)));
        Joint* childJoint = dynamic_cast<Joint*>(jrlchildJoint);
        forwardTransformation(childJoint);
    }
}


void DynamicMultiBody::RodriguesRotation(vector3d& inAxis, double inAngle, matrix3d& outRotation)
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

  for(unsigned int i=0;i<3;i++)
    {
      lPositionForRoot(i)=(*m_RootOfTheJointsTree)(i,3);
      lLinearVelocityForRoot(i)=m_Velocity(i);
    }

  for(unsigned int i=0;i<3;i++)
    for(unsigned int j=0;j<3;j++)
      lOrientationForRoot(i,j)=(*m_RootOfTheJointsTree)(i,j);
  
  for(unsigned int i=0;i<3;i++)
    lLinearVelocityForRoot(i)  = m_Velocity(i);

  for(unsigned int i=3;i<6;i++)
    lAngularVelocityForRoot(i-3)  = m_Velocity(i);
    
  
  ODEBUG(" Position for Root: " << lPositionForRoot);
  ForwardVelocity(lPositionForRoot,
		  lOrientationForRoot,
		  lLinearVelocityForRoot,
		  lAngularVelocityForRoot);

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
const MAL_S3_VECTOR(,double)&  DynamicMultiBody::positionCenterOfMass() 
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

// Compute the Jacobian matrix of the center of Mass
void DynamicMultiBody::computeJacobianCenterOfMass()
{ 
  double L[108],q[36];
  
  // Remap correctly the current configuration 
  // and the one for the CoM Jacobian.

  // Waist Configuration 
  for(int i=0;i<6;i++)
    q[30+i] = m_Configuration[i];

  // Rotational joint configurations.
  for(int i=0;i<30;i++)
    q[i] = m_Configuration[i+6];


  double t807;
  double t495;
  double t161;
  double t448;
  double t411;
  double t860;
  double t462;
  double t1047;
  double t412;
  double t498;
  double t1072;
  double t825;
  double t296;
  double t298;
  double t302;
  double t304;
  double t309;
  double t196;
  double t374;
  double t971;
  double t268;
  double t930;
  double t932;
  double t389;
  double t413;
  double t369;
  double t16;
  double t17;
  double t895;
  double t780;
  double t1280;
  double t491;
  double t484;
  double t427;
  double t253;
  double t254;
  double t256;
  double t59;
  double t719;
  double t504;
  double t1160;
  double t537;
  double t414;
  double t784;
  double t786;
  double t260;
  double t94;
  double t1;
  double t2;
  double t752;
  double t937;
  double t90;
  double t871;
  double t122;
  double t124;
  double t126;
  double t428;
  double t1069;
  double t685;
  double t496;
  double t520;
  double t834;
  double t483;
  double t987;
  double t482;
  double t939;
  double t136;
  double t434;
  double t1054;
  double t415;
  double t290;
  double t292;
  double t1042;
  double t435;
  double t502;
  double t354;
  double t955;
  double t481;
  double t497;
  double t193;
  double t480;
  double t539;
  double t499;
  double t643;
  double t436;
  double t358;
  double t596;
  double t417;
  double t416;
  double t541;
  double t805;
  double t387;
  double t437;
  double t420;
  double t557;
  double t479;
  double t776;
  double t419;
  double t418;
  double t765;
  double t192;
  double t1066;
  double t421;
  double t464;
  double t478;
  double t465;
  double t167;
  double t32;
  double t460;
  double t476;
  double t424;
  double t423;
  double t191;
  double t477;
  double t422;
  double t645;
  double t474;
  double t475;
  double t699;
  double t890;
  double t802;
  double t315;
  double t325;
  double t708;
  double t185;
  double t503;
  double t821;
  double t636;
  double t438;
  double t723;
  double t692;
  double t473;
  double t432;
  double t56;
  double t524;
  double t522;
  double t507;
  double t565;
  double t564;
  double t562;
  double t560;
  double t472;
  double t734;
  double t439;
  double t471;
  double t968;
  double t470;
  double t43;
  double t981;
  double t361;
  double t347;
  double t508;
  double t468;
  double t670;
  double t1050;
  double t469;
  double t675;
  double t188;
  double t248;
  double t440;
  double t509;
  double t381;
  double t467;
  double t195;
  double t466;
  double t364;
  double t757;
  double t1063;
  double t897;
  double t459;
  double t545;
  double t747;
  double t957;
  double t510;
  double t662;
  double t441;
  double t457;
  double t112;
  double t458;
  double t443;
  double t27;
  double t409;
  double t707;
  double t251;
  double t262;
  double t742;
  double t817;
  double t442;
  double t856;
  double t456;
  double t966;
  double t736;
  double t888;
  double t444;
  double t402;
  double t997;
  double t572;
  double t388;
  double t375;
  double t365;
  double t366;
  double t368;
  double t370;
  double t371;
  double t372;
  double t373;
  double t376;
  double t668;
  double t730;
  double t681;
  double t429;
  double t396;
  double t516;
  double t1060;
  double t755;
  double t589;
  double t377;
  double t461;
  double t657;
  double t920;
  double t797;
  double t99;
  double t728;
  double t399;
  double t46;
  double t47;
  double t230;
  double t234;
  double t235;
  double t236;
  double t239;
  double t241;
  double t242;
  double t350;
  double t351;
  double t352;
  double t353;
  double t356;
  double t357;
  double t359;
  double t360;
  double t362;
  double t363;
  double t454;
  double t763;
  double t576;
  double t240;
  double t132;
  double t149;
  double t153;
  double t455;
  double t159;
  double t163;
  double t1025;
  double t66;
  double t67;
  double t3;
  double t4;
  double t5;
  double t382;
  double t383;
  double t384;
  double t385;
  double t390;
  double t391;
  double t392;
  double t393;
  double t710;
  double t395;
  double t398;
  double t453;
  double t400;
  double t401;
  double t944;
  double t1013;
  double t841;
  double t222;
  double t223;
  double t224;
  double t225;
  double t226;
  double t227;
  double t229;
  double t452;
  double t902;
  double t13;
  double t14;
  double t15;
  double t117;
  double t827;
  double t679;
  double t949;
  double t1077;
  double t21;
  double t138;
  double t143;
  double t513;
  double t48;
  double t1023;
  double t1289;
  double t386;
  double t73;
  double t379;
  double t6;
  double t7;
  double t405;
  double t130;
  double t36;
  double t463;
  double t630;
  double t1097;
  double t650;
  double t37;
  double t38;
  double t39;
  double t602;
  double t604;
  double t721;
  double t283;
  double t430;
  double t759;
  double t451;
  double t582;
  double t8;
  double t9;
  double t975;
  double t740;
  double t1035;
  double t355;
  double t768;
  double t883;
  double t380;
  double t378;
  double t81;
  double t82;
  double t1037;
  double t445;
  double t911;
  double t913;
  double t694;
  double t492;
  double t704;
  double t394;
  double t1000;
  double t18;
  double t621;
  double t61;
  double t57;
  double t58;
  double t1017;
  double t10;
  double t1021;
  double t181;
  double t184;
  double t186;
  double t187;
  double t189;
  double t712;
  double t809;
  double t810;
  double t25;
  double t26;
  double t1002;
  double t1004;
  double t979;
  double t450;
  double t49;
  double t50;
  double t51;
  double t52;
  double t772;
  double t1362;
  double t190;
  double t44;
  double t45;
  double t829;
  double t221;
  double t926;
  double t40;
  double t106;
  double t288;
  double t1031;
  double t866;
  double t490;
  double t231;
  double t367;
  double t907;
  double t494;
  double t110;
  double t22;
  double t23;
  double t24;
  double t915;
  double t778;
  double t1090;
  double t634;
  double t844;
  double t849;
  double t62;
  double t64;
  double t875;
  double t33;
  double t34;
  double t272;
  double t953;
  double t194;
  double t397;
  double t652;
  double t28;
  double t29;
  double t30;
  double t31;
  double t881;
  double t626;
  double t53;
  double t611;
  double t609;
  double t1133;
  double t687;
  double t879;
  double t530;
  double t327;
  double t70;
  double t71;
  double t72;
  double t1082;
  double t489;
  double t792;
  double t790;
  double t65;
  double t446;
  double t228;
  double t985;
  double t218;
  double t219;
  double t41;
  double t42;
  double t433;
  double t854;
  double t319;
  double t714;
  double t87;
  double t88;
  double t1008;
  double t425;
  double t551;
  double t85;
  double t404;
  double t406;
  double t407;
  double t408;
  double t403;
  double t488;
  double t102;
  double t104;
  double t952;
  double t431;
  double t716;
  double t959;
  double t961;
  double t487;
  double t726;
  double t584;
  double t1057;
  double t197;
  double t198;
  double t199;
  double t200;
  double t201;
  double t202;
  double t203;
  double t204;
  double t205;
  double t206;
  double t207;
  double t208;
  double t209;
  double t210;
  double t211;
  double t212;
  double t213;
  double t214;
  double t215;
  double t216;
  double t217;
  double t449;
  double t447;
  double t410;
  double t333;
  double t335;
  double t337;
  double t342;
  double t19;
  double t20;
  double t599;
  double t615;
  double t924;
  double t1029;
  double t638;
  double t220;
  double t169;
  double t171;
  double t176;
  double t426;
  double t992;
  double t486;
  double t485;
  double t973;
  double t329;
  double t96;
  double t964;
  double t35;
  double t1010;
  double t580;
  double t847;
  double t493;
  double t666;
  double t265;
  double t270;
  double t276;
  double t278;
  double t11;
  double t12;
  t1 = q[35];
  t2 = cos(t1);
  t3 = q[34];
  t4 = cos(t3);
  t5 = t2 * t4;
  t6 = q[0];
  t7 = sin(t6);
  t8 = q[2];
  t9 = cos(t8);
  t10 = q[3];
  t11 = cos(t10);
  t12 = q[4];
  t13 = cos(t12);
  t14 = 0.4318600466e-2 * t13;
  t15 = sin(t12);
  t16 = q[5];
  t17 = sin(t16);
  t18 = 0.7684553059e-2 * t17;
  t19 = cos(t16);
  t20 = 0.1071449130e0 * t19;
  t21 = -0.1580767500e-2 - t18 - t20;
  t22 = t15 * t21;
  t23 = 0.3143585651e-1 + t14 + t22;
  t24 = t11 * t23;
  t25 = sin(t10);
  t26 = 0.4318600466e-2 * t15;
  t27 = t13 * t21;
  t28 = -0.9296094464e0 - t26 + t27;
  t29 = t25 * t28;
  t30 = 0.7182945987e-2 + t24 + t29;
  t31 = t9 * t30;
  t32 = sin(t8);
  t33 = t25 * t23;
  t34 = t11 * t28;
  t35 = -0.1526986523e1 - t33 + t34;
  t36 = t32 * t35;
  t37 = -0.1354459759e-1 + t31 + t36;
  t38 = t7 * t37;
  t39 = cos(t6);
  t40 = q[1];
  t41 = cos(t40);
  t42 = 0.7684553059e-2 * t19;
  t43 = 0.1071449130e0 * t17;
  t44 = -0.3652058335e0 - t42 + t43;
  t45 = t41 * t44;
  t46 = sin(t40);
  t47 = t32 * t30;
  t48 = t9 * t35;
  t49 = -0.2353404227e-2 - t47 + t48;
  t50 = t46 * t49;
  t51 = 0.5692054037e-2 + t45 - t50;
  t52 = t39 * t51;
  t53 = -t38 - t52;
  t56 = sin(t3);
  t57 = t2 * t56;
  t58 = q[33];
  t59 = sin(t58);
  t61 = sin(t1);
  t62 = cos(t58);
  t64 = t57 * t59 - t61 * t62;
  t65 = t39 * t37;
  t66 = t7 * t51;
  t67 = t65 - t66;
  L[0] = 0.1780275616e-1 * t5 * t53 + 0.1780275616e-1 * t64 * t67;
  t70 = t46 * t44;
  t71 = t41 * t49;
  t72 = -t70 - t71;
  t73 = t7 * t72;
  t81 = t57 * t62 + t61 * t59;
  t82 = t45 - t50;
  L[1] = -0.1780275616e-1 * t5 * t73 + 0.1780275616e-1 * t64 * t39 * t72 + 0.1780275616e-1 * t81 * t82;
  t85 = -t47 + t48;
  t87 = t7 * t46;
  t88 = -t31 - t36;
  t90 = t39 * t85 + t87 * t88;
  t94 = t39 * t46;
  t96 = t7 * t85 - t94 * t88;
  t99 = t81 * t41;
  L[2] = 0.1780275616e-1 * t5 * t90 + 0.1780275616e-1 * t64 * t96 + 0.1780275616e-1 * t99 * t88;
  t102 = -t33 + t34;
  t104 = -t24 - t29;
  t106 = t9 * t102 + t32 * t104;
  t110 = -t32 * t102 + t9 * t104;
  t112 = t39 * t106 + t87 * t110;
  t117 = t7 * t106 - t94 * t110;
  L[3] = 0.1780275616e-1 * t5 * t112 + 0.1780275616e-1 * t64 * t117 + 0.1780275616e-1 * t99 * t110;
  t122 = -t26 + t27;
  t124 = -t14 - t22;
  t126 = t11 * t122 + t25 * t124;
  t130 = -t25 * t122 + t11 * t124;
  t132 = t9 * t126 + t32 * t130;
  t136 = -t32 * t126 + t9 * t130;
  t138 = t39 * t132 + t87 * t136;
  t143 = t7 * t132 - t94 * t136;
  L[4] = 0.1780275616e-1 * t5 * t138 + 0.1780275616e-1 * t64 * t143 + 0.1780275616e-1 * t99 * t136;
  t149 = -t42 + t43;
  t153 = t11 * t15 * t149 + t25 * t13 * t149;
  t159 = -t25 * t15 * t149 + t11 * t13 * t149;
  t161 = t9 * t153 + t32 * t159;
  t163 = t18 + t20;
  t167 = -t32 * t153 + t9 * t159;
  t169 = t41 * t163 - t46 * t167;
  t171 = t39 * t161 - t7 * t169;
  t176 = t7 * t161 + t39 * t169;
  t181 = t46 * t163 + t41 * t167;
  L[5] = 0.1780275616e-1 * t5 * t171 + 0.1780275616e-1 * t64 * t176 + 0.1780275616e-1 * t81 * t181;
  t184 = q[6];
  t185 = sin(t184);
  t186 = q[8];
  t187 = cos(t186);
  t188 = q[9];
  t189 = cos(t188);
  t190 = q[10];
  t191 = cos(t190);
  t192 = 0.4318600466e-2 * t191;
  t193 = sin(t190);
  t194 = q[11];
  t195 = sin(t194);
  t196 = 0.7684553059e-2 * t195;
  t197 = cos(t194);
  t198 = 0.1071449130e0 * t197;
  t199 = -0.1580767500e-2 + t196 - t198;
  t200 = t193 * t199;
  t201 = 0.3143585651e-1 + t192 + t200;
  t202 = t189 * t201;
  t203 = sin(t188);
  t204 = 0.4318600466e-2 * t193;
  t205 = t191 * t199;
  t206 = -0.9296094464e0 - t204 + t205;
  t207 = t203 * t206;
  t208 = 0.7182945987e-2 + t202 + t207;
  t209 = t187 * t208;
  t210 = sin(t186);
  t211 = t203 * t201;
  t212 = t189 * t206;
  t213 = -0.1526986523e1 - t211 + t212;
  t214 = t210 * t213;
  t215 = -0.1354459759e-1 + t209 + t214;
  t216 = t185 * t215;
  t217 = cos(t184);
  t218 = q[7];
  t219 = cos(t218);
  t220 = 0.7684553059e-2 * t197;
  t221 = 0.1071449130e0 * t195;
  t222 = 0.3652058335e0 + t220 + t221;
  t223 = t219 * t222;
  t224 = sin(t218);
  t225 = t210 * t208;
  t226 = t187 * t213;
  t227 = -0.2353404227e-2 - t225 + t226;
  t228 = t224 * t227;
  t229 = -0.5692054037e-2 + t223 - t228;
  t230 = t217 * t229;
  t231 = -t216 - t230;
  t234 = t217 * t215;
  t235 = t185 * t229;
  t236 = t234 - t235;
  L[6] = 0.1780275616e-1 * t5 * t231 + 0.1780275616e-1 * t64 * t236;
  t239 = t224 * t222;
  t240 = t219 * t227;
  t241 = -t239 - t240;
  t242 = t185 * t241;
  t248 = t223 - t228;
  L[7] = -0.1780275616e-1 * t5 * t242 + 0.1780275616e-1 * t64 * t217 * t241 + 0.1780275616e-1 * t81 * t248;
  t251 = -t225 + t226;
  t253 = t185 * t224;
  t254 = -t209 - t214;
  t256 = t217 * t251 + t253 * t254;
  t260 = t217 * t224;
  t262 = t185 * t251 - t260 * t254;
  t265 = t81 * t219;
  L[8] = 0.1780275616e-1 * t5 * t256 + 0.1780275616e-1 * t64 * t262 + 0.1780275616e-1 * t265 * t254;
  t268 = -t211 + t212;
  t270 = -t202 - t207;
  t272 = t187 * t268 + t210 * t270;
  t276 = -t210 * t268 + t187 * t270;
  t278 = t217 * t272 + t253 * t276;
  t283 = t185 * t272 - t260 * t276;
  L[9] = 0.1780275616e-1 * t5 * t278 + 0.1780275616e-1 * t64 * t283 + 0.1780275616e-1 * t265 * t276;
  t288 = -t204 + t205;
  t290 = -t192 - t200;
  t292 = t189 * t288 + t203 * t290;
  t296 = -t203 * t288 + t189 * t290;
  t298 = t187 * t292 + t210 * t296;
  t302 = -t210 * t292 + t187 * t296;
  t304 = t217 * t298 + t253 * t302;
  t309 = t185 * t298 - t260 * t302;
  L[10] = 0.1780275616e-1 * t5 * t304 + 0.1780275616e-1 * t64 * t309 + 0.1780275616e-1 * t265 * t302;
  t315 = t220 + t221;
  t319 = t189 * t193 * t315 + t203 * t191 * t315;
  t325 = -t203 * t193 * t315 + t189 * t191 * t315;
  t327 = t187 * t319 + t210 * t325;
  t329 = -t196 + t198;
  t333 = -t210 * t319 + t187 * t325;
  t335 = t219 * t329 - t224 * t333;
  t337 = t217 * t327 - t185 * t335;
  t342 = t185 * t327 + t217 * t335;
  t347 = t224 * t329 + t219 * t333;
  L[11] = 0.1780275616e-1 * t5 * t337 + 0.1780275616e-1 * t64 * t342 + 0.1780275616e-1 * t81 * t347;
  t350 = q[12];
  t351 = sin(t350);
  t352 = q[13];
  t353 = cos(t352);
  t354 = q[14];
  t355 = cos(t354);
  t356 = q[15];
  t357 = cos(t356);
  t358 = 0.1290214844e-1 * t357;
  t359 = sin(t356);
  t360 = 0.7823719626e-1 * t359;
  t361 = -0.3280443429e-3 + t358 + t360;
  t362 = t355 * t361;
  t363 = sin(t354);
  t364 = 0.1682729166e-2 * t363;
  t365 = q[16];
  t366 = cos(t365);
  t367 = q[18];
  t368 = cos(t367);
  t369 = q[19];
  t370 = cos(t369);
  t371 = q[20];
  t372 = cos(t371);
  t373 = q[21];
  t374 = cos(t373);
  t375 = q[22];
  t376 = cos(t375);
  t377 = 0.1609908717e-3 * t376;
  t378 = sin(t375);
  t379 = 0.2594038004e-2 * t378;
  t380 = 0.1370806366e-2 - t377 - t379;
  t381 = t374 * t380;
  t382 = sin(t373);
  t383 = 0.1609908717e-3 * t378;
  t384 = 0.2594038004e-2 * t376;
  t385 = -0.5944304058e-1 + t383 - t384;
  t386 = t382 * t385;
  t387 = -0.1912846705e-2 + t381 + t386;
  t388 = t372 * t387;
  t389 = sin(t371);
  t390 = 0.1083551552e-2 * t389;
  t391 = -0.7205107036e-5 + t388 + t390;
  t392 = t370 * t391;
  t393 = sin(t369);
  t394 = t382 * t380;
  t395 = t374 * t385;
  t396 = -0.3638927977e0 - t394 + t395;
  t397 = t393 * t396;
  t398 = -0.8685296189e-2 + t392 + t397;
  t399 = t368 * t398;
  t400 = sin(t367);
  t401 = t389 * t387;
  t402 = 0.1083551552e-2 * t372;
  t403 = -0.2024021021e-1 + t401 - t402;
  t404 = t400 * t403;
  t405 = -0.1894413517e-1 + t399 - t404;
  t406 = t366 * t405;
  t407 = sin(t365);
  t408 = q[17];
  t409 = sin(t408);
  t410 = t400 * t398;
  t411 = t368 * t403;
  t412 = -0.1245819092e-1 + t410 + t411;
  t413 = t409 * t412;
  t414 = cos(t408);
  t415 = t393 * t391;
  t416 = t370 * t396;
  t417 = -0.8091715883e0 - t415 + t416;
  t418 = t414 * t417;
  t419 = -0.1240711988e-3 + t413 + t418;
  t420 = t407 * t419;
  t421 = q[23];
  t422 = cos(t421);
  t423 = q[25];
  t424 = cos(t423);
  t425 = q[26];
  t426 = cos(t425);
  t427 = q[27];
  t428 = cos(t427);
  t429 = q[28];
  t430 = cos(t429);
  t431 = q[29];
  t432 = cos(t431);
  t433 = 0.1609908717e-3 * t432;
  t434 = sin(t431);
  t435 = 0.2594038004e-2 * t434;
  t436 = 0.1370806366e-2 - t433 - t435;
  t437 = t430 * t436;
  t438 = sin(t429);
  t439 = 0.1609908717e-3 * t434;
  t440 = 0.2594038004e-2 * t432;
  t441 = -0.6028278175e-1 + t439 - t440;
  t442 = t438 * t441;
  t443 = 0.1373880793e-2 + t437 + t442;
  t444 = t428 * t443;
  t445 = sin(t427);
  t446 = 0.1083551552e-2 * t445;
  t447 = -0.7205107036e-5 + t444 - t446;
  t448 = t426 * t447;
  t449 = sin(t425);
  t450 = t438 * t436;
  t451 = t430 * t441;
  t452 = -0.3638927977e0 - t450 + t451;
  t453 = t449 * t452;
  t454 = -0.8685296189e-2 + t448 + t453;
  t455 = t424 * t454;
  t456 = sin(t423);
  t457 = t445 * t443;
  t458 = 0.1083551552e-2 * t428;
  t459 = 0.2024021021e-1 + t457 + t458;
  t460 = t456 * t459;
  t461 = -0.1894413517e-1 + t455 - t460;
  t462 = t422 * t461;
  t463 = sin(t421);
  t464 = q[24];
  t465 = sin(t464);
  t466 = t456 * t454;
  t467 = t424 * t459;
  t468 = 0.1245819092e-1 + t466 + t467;
  t469 = t465 * t468;
  t470 = cos(t464);
  t471 = t449 * t447;
  t472 = t426 * t452;
  t473 = -0.8091715883e0 - t471 + t472;
  t474 = t470 * t473;
  t475 = -0.1240711988e-3 + t469 + t474;
  t476 = t463 * t475;
  t477 = -0.2551806450e0 + t362 - t364 + t406 + t420 + t462 + t476;
  t478 = t353 * t477;
  t479 = sin(t352);
  t480 = 0.1290214844e-1 * t359;
  t481 = 0.7823719626e-1 * t357;
  t482 = t407 * t405;
  t483 = t366 * t419;
  t484 = t463 * t461;
  t485 = t422 * t475;
  t486 = 0.3816203553e1 - t480 + t481 - t482 + t483 - t484 + t485;
  t487 = t479 * t486;
  t488 = -0.1242588400e-1 + t478 + t487;
  t489 = t351 * t488;
  t490 = cos(t350);
  t491 = t363 * t361;
  t492 = 0.1682729166e-2 * t355;
  t493 = t414 * t412;
  t494 = t409 * t417;
  t495 = t470 * t468;
  t496 = t465 * t473;
  t497 = 0.8672745802e-1 + t491 + t492 + t493 - t494 + t495 - t496;
  t498 = t490 * t497;
  t499 = -t489 - t498;
  t502 = t490 * t488;
  t503 = t351 * t497;
  t504 = t502 - t503;
  L[12] = 0.1780275616e-1 * t5 * t499 + 0.1780275616e-1 * t64 * t504;
  t507 = t479 * t477;
  t508 = t353 * t486;
  t509 = -t507 + t508;
  t510 = t490 * t509;
  t513 = t64 * t351;
  t516 = -t478 - t487;
  L[13] = 0.1780275616e-1 * t5 * t510 + 0.1780275616e-1 * t513 * t509 + 0.1780275616e-1 * t81 * t516;
  t520 = -t491 - t492;
  t522 = t362 - t364;
  t524 = t490 * t353 * t520 - t351 * t522;
  t530 = t351 * t353 * t520 + t490 * t522;
  L[14] = 0.1780275616e-1 * t5 * t524 + 0.1780275616e-1 * t64 * t530 - 0.1780275616e-1 * t81 * t479 * t520;
  t537 = -t480 + t481;
  t539 = -t358 - t360;
  t541 = t353 * t355 * t537 + t479 * t539;
  t545 = t490 * t541 - t351 * t363 * t537;
  t551 = t351 * t541 + t490 * t363 * t537;
  t557 = -t479 * t355 * t537 + t353 * t539;
  L[15] = 0.1780275616e-1 * t5 * t545 + 0.1780275616e-1 * t64 * t551 + 0.1780275616e-1 * t81 * t557;
  t560 = -t482 + t483;
  t562 = -t406 - t420;
  t564 = t353 * t560 + t479 * t562;
  t565 = t490 * t564;
  t572 = -t479 * t560 + t353 * t562;
  L[16] = 0.1780275616e-1 * t5 * t565 + 0.1780275616e-1 * t513 * t564 + 0.1780275616e-1 * t81 * t572;
  t576 = t493 - t494;
  t580 = t353 * t407 * t576 + t479 * t366 * t576;
  t582 = -t413 - t418;
  t584 = t490 * t580 - t351 * t582;
  t589 = t351 * t580 + t490 * t582;
  t596 = -t479 * t407 * t576 + t353 * t366 * t576;
  L[17] = 0.1780275616e-1 * t5 * t584 + 0.1780275616e-1 * t64 * t589 + 0.1780275616e-1 * t81 * t596;
  t599 = -t410 - t411;
  t602 = t399 - t404;
  t604 = t366 * t599 + t407 * t409 * t602;
  t609 = -t407 * t599 + t366 * t409 * t602;
  t611 = t353 * t604 + t479 * t609;
  t615 = t490 * t611 - t351 * t414 * t602;
  t621 = t351 * t611 + t490 * t414 * t602;
  t626 = -t479 * t604 + t353 * t609;
  L[18] = 0.1780275616e-1 * t5 * t615 + 0.1780275616e-1 * t64 * t621 + 0.1780275616e-1 * t81 * t626;
  t630 = -t415 + t416;
  t634 = -t392 - t397;
  t636 = t409 * t400 * t630 + t414 * t634;
  t638 = t366 * t368 * t630 + t407 * t636;
  t643 = -t407 * t368 * t630 + t366 * t636;
  t645 = t353 * t638 + t479 * t643;
  t650 = t414 * t400 * t630 - t409 * t634;
  t652 = t490 * t645 - t351 * t650;
  t657 = t351 * t645 + t490 * t650;
  t662 = -t479 * t638 + t353 * t643;
  L[19] = 0.1780275616e-1 * t5 * t652 + 0.1780275616e-1 * t64 * t657 + 0.1780275616e-1 * t81 * t662;
  t666 = -t401 + t402;
  t668 = t388 + t390;
  t670 = t368 * t370 * t666 - t400 * t668;
  t675 = t400 * t370 * t666 + t368 * t668;
  t679 = t409 * t675 - t414 * t393 * t666;
  t681 = t366 * t670 + t407 * t679;
  t685 = -t407 * t670 + t366 * t679;
  t687 = t353 * t681 + t479 * t685;
  t692 = t414 * t675 + t409 * t393 * t666;
  t694 = t490 * t687 - t351 * t692;
  t699 = t351 * t687 + t490 * t692;
  t704 = -t479 * t681 + t353 * t685;
  L[20] = 0.1780275616e-1 * t5 * t694 + 0.1780275616e-1 * t64 * t699 + 0.1780275616e-1 * t81 * t704;
  t707 = t370 * t372;
  t708 = -t394 + t395;
  t710 = -t381 - t386;
  t712 = t707 * t708 + t393 * t710;
  t714 = t400 * t389;
  t716 = t368 * t712 - t714 * t708;
  t719 = t368 * t389;
  t721 = t400 * t712 + t719 * t708;
  t723 = t393 * t372;
  t726 = -t723 * t708 + t370 * t710;
  t728 = t409 * t721 + t414 * t726;
  t730 = t366 * t716 + t407 * t728;
  t734 = -t407 * t716 + t366 * t728;
  t736 = t353 * t730 + t479 * t734;
  t740 = t414 * t721 - t409 * t726;
  t742 = t490 * t736 - t351 * t740;
  t747 = t351 * t736 + t490 * t740;
  t752 = -t479 * t730 + t353 * t734;
  L[21] = 0.1780275616e-1 * t5 * t742 + 0.1780275616e-1 * t64 * t747 + 0.1780275616e-1 * t81 * t752;
  t755 = t383 - t384;
  t757 = t377 + t379;
  t759 = t374 * t755 + t382 * t757;
  t763 = -t382 * t755 + t374 * t757;
  t765 = t707 * t759 + t393 * t763;
  t768 = t368 * t765 - t714 * t759;
  t772 = t400 * t765 + t719 * t759;
  t776 = -t723 * t759 + t370 * t763;
  t778 = t409 * t772 + t414 * t776;
  t780 = t366 * t768 + t407 * t778;
  t784 = -t407 * t768 + t366 * t778;
  t786 = t353 * t780 + t479 * t784;
  t790 = t414 * t772 - t409 * t776;
  t792 = t490 * t786 - t351 * t790;
  t797 = t351 * t786 + t490 * t790;
  t802 = -t479 * t780 + t353 * t784;
  L[22] = 0.1780275616e-1 * t5 * t792 + 0.1780275616e-1 * t64 * t797 + 0.1780275616e-1 * t81 * t802;
  t805 = -t484 + t485;
  t807 = -t462 - t476;
  t809 = t353 * t805 + t479 * t807;
  t810 = t490 * t809;
  t817 = -t479 * t805 + t353 * t807;
  L[23] = 0.1780275616e-1 * t5 * t810 + 0.1780275616e-1 * t513 * t809 + 0.1780275616e-1 * t81 * t817;
  t821 = t495 - t496;
  t825 = t353 * t463 * t821 + t479 * t422 * t821;
  t827 = -t469 - t474;
  t829 = t490 * t825 - t351 * t827;
  t834 = t351 * t825 + t490 * t827;
  t841 = -t479 * t463 * t821 + t353 * t422 * t821;
  L[24] = 0.1780275616e-1 * t5 * t829 + 0.1780275616e-1 * t64 * t834 + 0.1780275616e-1 * t81 * t841;
  t844 = -t466 - t467;
  t847 = t455 - t460;
  t849 = t422 * t844 + t463 * t465 * t847;
  t854 = -t463 * t844 + t422 * t465 * t847;
  t856 = t353 * t849 + t479 * t854;
  t860 = t490 * t856 - t351 * t470 * t847;
  t866 = t351 * t856 + t490 * t470 * t847;
  t871 = -t479 * t849 + t353 * t854;
  L[25] = 0.1780275616e-1 * t5 * t860 + 0.1780275616e-1 * t64 * t866 + 0.1780275616e-1 * t81 * t871;
  t875 = -t471 + t472;
  t879 = -t448 - t453;
  t881 = t465 * t456 * t875 + t470 * t879;
  t883 = t422 * t424 * t875 + t463 * t881;
  t888 = -t463 * t424 * t875 + t422 * t881;
  t890 = t353 * t883 + t479 * t888;
  t895 = t470 * t456 * t875 - t465 * t879;
  t897 = t490 * t890 - t351 * t895;
  t902 = t351 * t890 + t490 * t895;
  t907 = -t479 * t883 + t353 * t888;
  L[26] = 0.1780275616e-1 * t5 * t897 + 0.1780275616e-1 * t64 * t902 + 0.1780275616e-1 * t81 * t907;
  t911 = -t457 - t458;
  t913 = t444 - t446;
  t915 = t424 * t426 * t911 - t456 * t913;
  t920 = t456 * t426 * t911 + t424 * t913;
  t924 = t465 * t920 - t470 * t449 * t911;
  t926 = t422 * t915 + t463 * t924;
  t930 = -t463 * t915 + t422 * t924;
  t932 = t353 * t926 + t479 * t930;
  t937 = t470 * t920 + t465 * t449 * t911;
  t939 = t490 * t932 - t351 * t937;
  t944 = t351 * t932 + t490 * t937;
  t949 = -t479 * t926 + t353 * t930;
  L[27] = 0.1780275616e-1 * t5 * t939 + 0.1780275616e-1 * t64 * t944 + 0.1780275616e-1 * t81 * t949;
  t952 = t426 * t428;
  t953 = -t450 + t451;
  t955 = -t437 - t442;
  t957 = t952 * t953 + t449 * t955;
  t959 = t456 * t445;
  t961 = t424 * t957 - t959 * t953;
  t964 = t424 * t445;
  t966 = t456 * t957 + t964 * t953;
  t968 = t449 * t428;
  t971 = -t968 * t953 + t426 * t955;
  t973 = t465 * t966 + t470 * t971;
  t975 = t422 * t961 + t463 * t973;
  t979 = -t463 * t961 + t422 * t973;
  t981 = t353 * t975 + t479 * t979;
  t985 = t470 * t966 - t465 * t971;
  t987 = t490 * t981 - t351 * t985;
  t992 = t351 * t981 + t490 * t985;
  t997 = -t479 * t975 + t353 * t979;
  L[28] = 0.1780275616e-1 * t5 * t987 + 0.1780275616e-1 * t64 * t992 + 0.1780275616e-1 * t81 * t997;
  t1000 = t439 - t440;
  t1002 = t433 + t435;
  t1004 = t430 * t1000 + t438 * t1002;
  t1008 = -t438 * t1000 + t430 * t1002;
  t1010 = t952 * t1004 + t449 * t1008;
  t1013 = t424 * t1010 - t959 * t1004;
  t1017 = t456 * t1010 + t964 * t1004;
  t1021 = -t968 * t1004 + t426 * t1008;
  t1023 = t465 * t1017 + t470 * t1021;
  t1025 = t422 * t1013 + t463 * t1023;
  t1029 = -t463 * t1013 + t422 * t1023;
  t1031 = t353 * t1025 + t479 * t1029;
  t1035 = t470 * t1017 - t465 * t1021;
  t1037 = t490 * t1031 - t351 * t1035;
  t1042 = t351 * t1031 + t490 * t1035;
  t1047 = -t479 * t1025 + t353 * t1029;
  L[29] = 0.1780275616e-1 * t5 * t1037 + 0.1780275616e-1 * t64 * t1042 + 0.1780275616e-1 * t81 * t1047;
  L[30] = 0.9999999997e0;
  L[31] = 0.0e0;
  L[32] = 0.0e0;
  t1050 = 0.18035e-5 + t38 + t52 + t216 + t230 + t489 + t498;
  t1054 = 0.1220627955e2 + t70 + t71 + t239 + t240 - t507 + t508;
  L[33] = 0.1780275616e-1 * t81 * t1050 - 0.1780275616e-1 * t64 * t1054;
  t1057 = 0.5787582033e0 + t65 - t66 + t234 - t235 + t502 - t503;
  t1060 = t59 * t1050;
  t1063 = t62 * t1054;
  L[34] = -0.1780275616e-1 * t57 * t1057 + 0.1780275616e-1 * t5 * t1060 + 0.1780275616e-1 * t5 * t1063;
  t1066 = t61 * t4;
  t1069 = t61 * t56;
  t1072 = -t1069 * t59 - t2 * t62;
  t1077 = -t1069 * t62 + t2 * t59;
  L[35] = -0.1780275616e-1 * t1066 * t1057 + 0.1780275616e-1 * t1072 * t1050 + 0.1780275616e-1 * t1077 * t1054;
  t1082 = -t1072;
  L[36] = 0.1780275616e-1 * t1066 * t53 + 0.1780275616e-1 * t1082 * t67;
  t1090 = -t1077;
  L[37] = -0.1780275616e-1 * t1066 * t73 + 0.1780275616e-1 * t1082 * t39 * t72 + 0.1780275616e-1 * t1090 * t82;
  t1097 = t1090 * t41;
  L[38] = 0.1780275616e-1 * t1066 * t90 + 0.1780275616e-1 * t1082 * t96 + 0.1780275616e-1 * t1097 * t88;
  L[39] = 0.1780275616e-1 * t1066 * t112 + 0.1780275616e-1 * t1082 * t117 + 0.1780275616e-1 * t1097 * t110;
  L[40] = 0.1780275616e-1 * t1066 * t138 + 0.1780275616e-1 * t1082 * t143 + 0.1780275616e-1 * t1097 * t136;
  L[41] = 0.1780275616e-1 * t1066 * t171 + 0.1780275616e-1 * t1082 * t176 + 0.1780275616e-1 * t1090 * t181;
  L[42] = 0.1780275616e-1 * t1066 * t231 + 0.1780275616e-1 * t1082 * t236;
  L[43] = -0.1780275616e-1 * t1066 * t242 + 0.1780275616e-1 * t1082 * t217 * t241 + 0.1780275616e-1 * t1090 * t248;
  t1133 = t1090 * t219;
  L[44] = 0.1780275616e-1 * t1066 * t256 + 0.1780275616e-1 * t1082 * t262 + 0.1780275616e-1 * t1133 * t254;
  L[45] = 0.1780275616e-1 * t1066 * t278 + 0.1780275616e-1 * t1082 * t283 + 0.1780275616e-1 * t1133 * t276;
  L[46] = 0.1780275616e-1 * t1066 * t304 + 0.1780275616e-1 * t1082 * t309 + 0.1780275616e-1 * t1133 * t302;
  L[47] = 0.1780275616e-1 * t1066 * t337 + 0.1780275616e-1 * t1082 * t342 + 0.1780275616e-1 * t1090 * t347;
  L[48] = 0.1780275616e-1 * t1066 * t499 + 0.1780275616e-1 * t1082 * t504;
  t1160 = t1082 * t351;
  L[49] = 0.1780275616e-1 * t1066 * t510 + 0.1780275616e-1 * t1160 * t509 + 0.1780275616e-1 * t1090 * t516;
  L[50] = 0.1780275616e-1 * t1066 * t524 + 0.1780275616e-1 * t1082 * t530 - 0.1780275616e-1 * t1090 * t479 * t520;
  L[51] = 0.1780275616e-1 * t1066 * t545 + 0.1780275616e-1 * t1082 * t551 + 0.1780275616e-1 * t1090 * t557;
  L[52] = 0.1780275616e-1 * t1066 * t565 + 0.1780275616e-1 * t1160 * t564 + 0.1780275616e-1 * t1090 * t572;
  L[53] = 0.1780275616e-1 * t1066 * t584 + 0.1780275616e-1 * t1082 * t589 + 0.1780275616e-1 * t1090 * t596;
  L[54] = 0.1780275616e-1 * t1066 * t615 + 0.1780275616e-1 * t1082 * t621 + 0.1780275616e-1 * t1090 * t626;
  L[55] = 0.1780275616e-1 * t1066 * t652 + 0.1780275616e-1 * t1082 * t657 + 0.1780275616e-1 * t1090 * t662;
  L[56] = 0.1780275616e-1 * t1066 * t694 + 0.1780275616e-1 * t1082 * t699 + 0.1780275616e-1 * t1090 * t704;
  L[57] = 0.1780275616e-1 * t1066 * t742 + 0.1780275616e-1 * t1082 * t747 + 0.1780275616e-1 * t1090 * t752;
  L[58] = 0.1780275616e-1 * t1066 * t792 + 0.1780275616e-1 * t1082 * t797 + 0.1780275616e-1 * t1090 * t802;
  L[59] = 0.1780275616e-1 * t1066 * t810 + 0.1780275616e-1 * t1160 * t809 + 0.1780275616e-1 * t1090 * t817;
  L[60] = 0.1780275616e-1 * t1066 * t829 + 0.1780275616e-1 * t1082 * t834 + 0.1780275616e-1 * t1090 * t841;
  L[61] = 0.1780275616e-1 * t1066 * t860 + 0.1780275616e-1 * t1082 * t866 + 0.1780275616e-1 * t1090 * t871;
  L[62] = 0.1780275616e-1 * t1066 * t897 + 0.1780275616e-1 * t1082 * t902 + 0.1780275616e-1 * t1090 * t907;
  L[63] = 0.1780275616e-1 * t1066 * t939 + 0.1780275616e-1 * t1082 * t944 + 0.1780275616e-1 * t1090 * t949;
  L[64] = 0.1780275616e-1 * t1066 * t987 + 0.1780275616e-1 * t1082 * t992 + 0.1780275616e-1 * t1090 * t997;
  L[65] = 0.1780275616e-1 * t1066 * t1037 + 0.1780275616e-1 * t1082 * t1042 + 0.1780275616e-1 * t1090 * t1047;
  L[66] = 0.0e0;
  L[67] = 0.9999999997e0;
  L[68] = 0.0e0;
  L[69] = 0.1780275616e-1 * t1090 * t1050 + 0.1780275616e-1 * t1072 * t1054;
  L[70] = -0.1780275616e-1 * t1069 * t1057 + 0.1780275616e-1 * t1066 * t1060 + 0.1780275616e-1 * t1066 * t1063;
  L[71] = 0.1780275616e-1 * t5 * t1057 + 0.1780275616e-1 * t64 * t1050 + 0.1780275616e-1 * t81 * t1054;
  t1280 = t4 * t59;
  L[72] = -0.1780275616e-1 * t56 * t53 + 0.1780275616e-1 * t1280 * t67;
  t1289 = t4 * t62;
  L[73] = 0.1780275616e-1 * t56 * t7 * t72 + 0.1780275616e-1 * t1280 * t39 * t72 + 0.1780275616e-1 * t1289 * t82;
  L[74] = -0.1780275616e-1 * t56 * t90 + 0.1780275616e-1 * t1280 * t96 + 0.1780275616e-1 * t1289 * t41 * t88;
  L[75] = -0.1780275616e-1 * t56 * t112 + 0.1780275616e-1 * t1280 * t117 + 0.1780275616e-1 * t1289 * t41 * t110;
  L[76] = -0.1780275616e-1 * t56 * t138 + 0.1780275616e-1 * t1280 * t143 + 0.1780275616e-1 * t1289 * t41 * t136;
  L[77] = -0.1780275616e-1 * t56 * t171 + 0.1780275616e-1 * t1280 * t176 + 0.1780275616e-1 * t1289 * t181;
  L[78] = -0.1780275616e-1 * t56 * t231 + 0.1780275616e-1 * t1280 * t236;
  L[79] = 0.1780275616e-1 * t56 * t185 * t241 + 0.1780275616e-1 * t1280 * t217 * t241 + 0.1780275616e-1 * t1289 * t248;
  L[80] = -0.1780275616e-1 * t56 * t256 + 0.1780275616e-1 * t1280 * t262 + 0.1780275616e-1 * t1289 * t219 * t254;
  L[81] = -0.1780275616e-1 * t56 * t278 + 0.1780275616e-1 * t1280 * t283 + 0.1780275616e-1 * t1289 * t219 * t276;
  L[82] = -0.1780275616e-1 * t56 * t304 + 0.1780275616e-1 * t1280 * t309 + 0.1780275616e-1 * t1289 * t219 * t302;
  L[83] = -0.1780275616e-1 * t56 * t337 + 0.1780275616e-1 * t1280 * t342 + 0.1780275616e-1 * t1289 * t347;
  L[84] = -0.1780275616e-1 * t56 * t499 + 0.1780275616e-1 * t1280 * t504;
  t1362 = t56 * t490;
  L[85] = -0.1780275616e-1 * t1362 * t509 + 0.1780275616e-1 * t1280 * t351 * t509 + 0.1780275616e-1 * t1289 * t516;
  L[86] = -0.1780275616e-1 * t56 * t524 + 0.1780275616e-1 * t1280 * t530 - 0.1780275616e-1 * t1289 * t479 * t520;
  L[87] = -0.1780275616e-1 * t56 * t545 + 0.1780275616e-1 * t1280 * t551 + 0.1780275616e-1 * t1289 * t557;
  L[88] = -0.1780275616e-1 * t1362 * t564 + 0.1780275616e-1 * t1280 * t351 * t564 + 0.1780275616e-1 * t1289 * t572;
  L[89] = -0.1780275616e-1 * t56 * t584 + 0.1780275616e-1 * t1280 * t589 + 0.1780275616e-1 * t1289 * t596;
  L[90] = -0.1780275616e-1 * t56 * t615 + 0.1780275616e-1 * t1280 * t621 + 0.1780275616e-1 * t1289 * t626;
  L[91] = -0.1780275616e-1 * t56 * t652 + 0.1780275616e-1 * t1280 * t657 + 0.1780275616e-1 * t1289 * t662;
  L[92] = -0.1780275616e-1 * t56 * t694 + 0.1780275616e-1 * t1280 * t699 + 0.1780275616e-1 * t1289 * t704;
  L[93] = -0.1780275616e-1 * t56 * t742 + 0.1780275616e-1 * t1280 * t747 + 0.1780275616e-1 * t1289 * t752;
  L[94] = -0.1780275616e-1 * t56 * t792 + 0.1780275616e-1 * t1280 * t797 + 0.1780275616e-1 * t1289 * t802;
  L[95] = -0.1780275616e-1 * t1362 * t809 + 0.1780275616e-1 * t1280 * t351 * t809 + 0.1780275616e-1 * t1289 * t817;
  L[96] = -0.1780275616e-1 * t56 * t829 + 0.1780275616e-1 * t1280 * t834 + 0.1780275616e-1 * t1289 * t841;
  L[97] = -0.1780275616e-1 * t56 * t860 + 0.1780275616e-1 * t1280 * t866 + 0.1780275616e-1 * t1289 * t871;
  L[98] = -0.1780275616e-1 * t56 * t897 + 0.1780275616e-1 * t1280 * t902 + 0.1780275616e-1 * t1289 * t907;
  L[99] = -0.1780275616e-1 * t56 * t939 + 0.1780275616e-1 * t1280 * t944 + 0.1780275616e-1 * t1289 * t949;
  L[100] = -0.1780275616e-1 * t56 * t987 + 0.1780275616e-1 * t1280 * t992 + 0.1780275616e-1 * t1289 * t997;
  L[101] = -0.1780275616e-1 * t56 * t1037 + 0.1780275616e-1 * t1280 * t1042 + 0.1780275616e-1 * t1289 * t1047;
  L[102] = 0.0e0;
  L[103] = 0.0e0;
  L[104] = 0.9999999997e0;
  L[105] = 0.1780275616e-1 * t1289 * t1050 - 0.1780275616e-1 * t1280 * t1054;
  L[106] = -0.1780275616e-1 * t4 * t1057 - 0.1780275616e-1 * t56 * t59 * t1050 - 0.1780275616e-1 * t56 * t62 * t1054;
  L[107] = 0.0e0;

  for(int i=0;i<3;i++)
    {
      for(int j=0;j<6;j++)
	m_JacobianOfTheCoM(i,j) = L[i*36+j+30];

      for(int j=0;j<30;j++)
	m_JacobianOfTheCoM(i,j+6) = L[i*36+j];
      
      for(int j=36;j<m_NbDofs;j++)
	m_JacobianOfTheCoM(i,j) = 0.0;
    }
}

const MAL_MATRIX(,double) &DynamicMultiBody::jacobianCenterOfMass() const
{
  return m_JacobianOfTheCoM;
  
}


