/* @doc \file Common Tools for unitary testing

   Copyright (c) 2010
   @author Olivier Stasse
   JRL-Japan, CNRS/AIST
 
   All rights reserved.

   Please refers to file License.txt for details on the license.

*/
#include <fstream>
#include "CommonTools.h"
using namespace std;
using namespace dynamicsJRLJapan;

namespace dynamicsJRLJapan {

  double filterprecision(double adb)
  {
    if (fabs(adb)<1e-8)
      return 0.0;
    return adb;
  }

  void dv3d(vector3d &av3d, ostream &os) 
  {
    for(unsigned int i=0;i<3;i++)
      os << filterprecision(av3d(i)) << " ";
    os << endl;
  }

  void dm3d(const matrix3d &todisplay, ostream &os, string shifttab)
  {
    for(unsigned int i=0;i<3;i++)
      {
	for(unsigned int j=0;j<3;j++)
	  {
	    os << filterprecision(MAL_S3x3_MATRIX_ACCESS_I_J(todisplay,i,j)) << " " ;
	  }
	os << endl;
	if (i!=2)
	  os << shifttab;
      }
  }

  void dm4d(const matrix4d &todisplay, ostream &os, string shifttab)
  {
    for(unsigned int i=0;i<4;i++)
      {
	for(unsigned int j=0;j<4;j++)
	  {
	    os << MAL_S4x4_MATRIX_ACCESS_I_J(todisplay,i,j) << " " ;
	  }
	os << endl;
	if (i!=3)
	  os << shifttab;
      }
  }


  void dm4dv(matrix4d & todisplay, ostream &os)
  {
    os << "(";
    for(unsigned int i=0;i<4;i++)
      {
	for(unsigned int j=0;j<4;j++)
	  {
	    os << filterprecision(MAL_S4x4_MATRIX_ACCESS_I_J(todisplay,i,j)) ;
	    if ((i!=3) || (j!=3))
	      os << ",";
	  }
      }
    os << ")";
  }

  void DisplayMatrix(MAL_MATRIX(,double) &aJ,ostream &os)
  {
    for(unsigned int i=0;i<MAL_MATRIX_NB_ROWS(aJ);i++)
      {
	for(unsigned int j=0;j<MAL_MATRIX_NB_COLS(aJ);j++)
	  {
	    if (aJ(i,j)==0.0)
	      os << "0 "; 
	    else
	      os << filterprecision(aJ(i,j)) << " ";
	  }
	os << std::endl;
      }

  }

  void DisplayMatrix(const MAL_MATRIX(,double) &aJ, ostream &os)
  {
    
    for(unsigned int i=0;i<MAL_MATRIX_NB_ROWS(aJ);i++)
      {
	for(unsigned int j=0;j<MAL_MATRIX_NB_COLS(aJ);j++)
	  {
	    if (aJ(i,j)==0.0)
	      os << "0 "; 
	    else
	      {
		double adb;
		adb = aJ(i,j);
		os << filterprecision(adb) << " ";
	      }
	  }
	os << std::endl;
      }

  }

  void DisplayTorques(CjrlHumanoidDynamicRobot *aHDR, 
		      string &shifttab, 
		      ostream &tcout)
  {
    if (aHDR!=0)
      {
	const matrixNxP& Torques = aHDR->currentTorques();
	tcout << shifttab << "Torques :" ;
	DisplayMatrix(Torques,tcout);
	tcout << std::endl;
      }
  }

  void DisplayForces(CjrlHumanoidDynamicRobot *aHDR, 
		     string &shifttab, 
		     ostream &tcout)
  {
    if (aHDR!=0)
      {
	const matrixNxP& Forces = aHDR->currentForces();
	tcout << shifttab << "Forces :";
	DisplayMatrix(Forces,tcout);
	tcout << std::endl;
      }
  }

  void DisplayActuated(CjrlHumanoidDynamicRobot *aHDR, 
		       string &shifttab, 
		       ostream &tcout)
  {
    if (aHDR!=0)
      {
	std::vector<CjrlJoint *> ActuatedJoints = aHDR->getActuatedJoints();
	tcout << shifttab << "Actuated Joints (by rank) :";
	for(unsigned int i=0;i<ActuatedJoints.size();i++)
	  {
	    tcout << ActuatedJoints[i]->rankInConfiguration() << " ";
	  }
	tcout << std::endl;
      }
  }

  void DisplayBody(CjrlBody *aBody, string &shifttab,ostream &tcout)
  {
  
    tcout << shifttab << "Related body informations" << endl;
    vector3d alcm = aBody->localCenterOfMass();
    tcout << shifttab << "Local center of Mass : ";
    dv3d(alcm,tcout);
    matrix3d aim = aBody->inertiaMatrix();
    tcout << shifttab << "Inertia Matrix: " ;
    dm3d(aim,tcout,shifttab);
    tcout << shifttab << "mass: " << aBody->mass() << endl;
      
  }
  
  void DisplayHand(CjrlHand *ajrlHand,string &shifttab,ostream &tcout)
  {
    vector3d outCenter,outThumbAxis, outForeFinger,
      outPalmNormal;
    ajrlHand->getCenter(outCenter);
    tcout << shifttab << "Center:" 
	  << outCenter(0) << " " 
	  << outCenter(1) << " "
	  << outCenter(2) << endl;
  
    ajrlHand->getThumbAxis(outThumbAxis);
    tcout << shifttab << "Center:" 
	  << outThumbAxis(0) << " " 
	  << outThumbAxis(1) << " "
	  << outThumbAxis(2) << endl;
  
    ajrlHand->getForeFingerAxis(outForeFinger);
    tcout << shifttab << "Center:" 
	  << outForeFinger(0) << " " 
	  << outForeFinger(1) << " "
	  << outForeFinger(2) << endl;
  
    ajrlHand->getPalmNormal(outPalmNormal);
    tcout << shifttab << "Center:" 
	  << outForeFinger(0) << " " 
	  << outForeFinger(1) << " "
	  << outForeFinger(2) << endl;

    
    RecursiveDisplayOfJoints((CjrlJoint *)ajrlHand->associatedWrist(),
			     tcout, 2, 0);
				 
  }

  void DisplayFoot(CjrlFoot *aFoot,string &shifttab,ostream &tcout)
  {
    vector3d AnklePositionInLocalFrame,
      SoleCenterInLocalFrame,
      ProjectionCenterLocalFrameInSole;

    double SoleLength, SoleWidth;
    aFoot->getSoleSize(SoleLength,SoleWidth);
  
    tcout << shifttab << "SoleLength: " << SoleLength << " " 
	  << "SoleWidth: " << SoleWidth << std::endl;
  
    aFoot->getAnklePositionInLocalFrame(AnklePositionInLocalFrame);
    tcout << shifttab << "AnklePositionInLocalFrame=("
	  << AnklePositionInLocalFrame(0) << " , "
	  << AnklePositionInLocalFrame(1) << " , "
	  << AnklePositionInLocalFrame(2) << ")" << std::endl;

    aFoot->getSoleCenterInLocalFrame(SoleCenterInLocalFrame);
    tcout << shifttab << "SoleCenterInLocalFrame=("
	  << SoleCenterInLocalFrame(0) << " , "
	  << SoleCenterInLocalFrame(1) << " , "
	  << SoleCenterInLocalFrame(2) << " ) " << std::endl;

    aFoot->getSoleCenterInLocalFrame(ProjectionCenterLocalFrameInSole);
    tcout << shifttab << "ProjectCenterLocalFrameInSole=("
	  << ProjectionCenterLocalFrameInSole(0) << " , " 
	  << ProjectionCenterLocalFrameInSole(1) << " , " 
	  << ProjectionCenterLocalFrameInSole(2) << " ) " << std::endl;

  }

  void RecursiveDisplayOfJoints(CjrlJoint *aJoint, 
				ostream &tcout,
				unsigned int verbosedisplay,
				unsigned int ldepth)
  {
    if (aJoint==0)
      return;

    int NbChildren = aJoint->countChildJoints();
    string shifttab="";
    for(unsigned int i=0;i<ldepth;i++)
      shifttab+=" ";
  
    tcout << shifttab << "Rank : " 
	  << aJoint->rankInConfiguration() << endl;
    tcout << shifttab << "Number of child  :" 
	  << aJoint->countChildJoints() << endl;
    tcout << shifttab << "Nb of degree of freedom " 
	  <<  aJoint->numberDof() << endl;
  

    if (verbosedisplay>1)
      {
	tcout << shifttab << "Initial Position " ;
	matrix4d iP = aJoint->initialPosition();
	dm4d(iP,tcout,shifttab);
      
	tcout << shifttab << "CurrentTransformation ";
	matrix4d cT = aJoint->currentTransformation();
	dm4d(cT,tcout,shifttab);

	if (verbosedisplay>2)
	  {
	    // Limits
	    tcout << shifttab << "llimit: " 
		  << aJoint->lowerBound(0)*180/M_PI << " " 
		  << shifttab << "ulimit: " 
		  << aJoint->upperBound(0)*180/M_PI << " " << endl;
	    
	    tcout << shifttab << "lvlimit: " 
		  << aJoint->lowerVelocityBound(0)*180/M_PI << " " 
		  << shifttab << "uvlimit: " 
		  << aJoint->upperVelocityBound(0)*180/M_PI << " " << endl;
	    
	    // Path from the root to this joint.
	    std::vector<CjrlJoint*> JointsFromRootToHere = aJoint->jointsFromRootToThis();
	    tcout << shifttab << "Nb of nodes: " << JointsFromRootToHere.size() << endl
		  << shifttab << "Joint from root to here:" << endl << shifttab;
	    for(unsigned int i=0;i<JointsFromRootToHere.size();i++)
	      tcout << JointsFromRootToHere[i]->rankInConfiguration() << " ";
	    tcout << endl;
	    
	    if (verbosedisplay>3)
	      {
		// Current state of the joint.
		
		// Rigid velocity:
		CjrlRigidVelocity aRV = aJoint->jointVelocity();
		tcout << shifttab << "Linear Velocity ";
		vector3d av3d = aRV.linearVelocity();
		dv3d(av3d,tcout);
		tcout << shifttab << "Angular Velocity ";
		av3d = aRV.rotationVelocity();
		dv3d(av3d,tcout);
		
		// Rigit Acceleration.
		CjrlRigidAcceleration aRA = aJoint->jointAcceleration();
		tcout << shifttab << "Linear Acceleration ";
		av3d = aRA.linearAcceleration();
		dv3d(av3d,tcout);
		tcout << shifttab << "Angular Acceleration ";
		av3d = aRA.rotationAcceleration();
		dv3d(av3d,tcout);
		
		CjrlBody * aBody = aJoint->linkedBody();
		DisplayBody(aBody,shifttab,tcout);
	      }
	  }
      }
    
    if (NbChildren!=0)
      {
	tcout << shifttab << "***********************************************" << endl;
	tcout << shifttab << " Display Now information related to children :" << endl;
      
	for(int i=0;i<NbChildren;i++)
	  {
	    // Returns a const so we have to force the casting/
	    RecursiveDisplayOfJoints(aJoint->childJoint(i),
				     tcout,verbosedisplay,ldepth+1); 
	  }
      }
  }
  void DisplayLinearVelocity(CjrlHumanoidDynamicRobot *aHDR,
			   ostream &tcout)
  {
    std::vector<CjrlJoint *> aVec = aHDR->jointVector();
    for(unsigned int i=0;i<aVec.size();i++)
      {
	CjrlRigidVelocity aRA =aVec[i]->jointVelocity(); 
	vector3d av3d = aRA.linearVelocity();
	dv3d(av3d,tcout);
      }
  }

  void DisplayAngularVelocity(CjrlHumanoidDynamicRobot *aHDR,
			   ostream &tcout)
  {
    std::vector<CjrlJoint *> aVec = aHDR->jointVector();
    for(unsigned int i=0;i<aVec.size();i++)
      {
	CjrlRigidVelocity aRA =aVec[i]->jointVelocity(); 
	vector3d av3d = aRA.rotationVelocity();
	dv3d(av3d,tcout);
      }
  }

  void DisplayLinearAcceleration(CjrlHumanoidDynamicRobot *aHDR,
			   ostream &tcout)
  {
    std::vector<CjrlJoint *> aVec = aHDR->jointVector();
    for(unsigned int i=0;i<aVec.size();i++)
      {
	CjrlRigidAcceleration aRA =aVec[i]->jointAcceleration(); 
	vector3d av3d = aRA.linearAcceleration();
	dv3d(av3d,tcout);
      }
  }

  void DisplayAngularAcceleration(CjrlHumanoidDynamicRobot *aHDR,
			   ostream &tcout)
  {
    std::vector<CjrlJoint *> aVec = aHDR->jointVector();
    for(unsigned int i=0;i<aVec.size();i++)
      {
	CjrlRigidAcceleration aRA =aVec[i]->jointAcceleration(); 
	vector3d av3d = aRA.rotationAcceleration();
	dv3d(av3d,tcout);
      }
  }
  
  
  void DisplayEndEffectors(CjrlHumanoidDynamicRobot *aHDR,
			   ostream &tcout)
  {
     // Test rank of the hands.
    tcout << "Rank of the right hand "<< endl;
    tcout << aHDR->rightWrist()->rankInConfiguration() << endl;
    CjrlHand *rightHand = aHDR->rightHand();
    string empty("");
    DisplayHand(rightHand,empty,tcout);
    
    tcout << "Rank of the left hand "<< endl;
    tcout << aHDR->leftWrist()->rankInConfiguration() << endl;
    CjrlHand *leftHand = aHDR->leftHand();
    DisplayHand(leftHand,empty,tcout);
    
    // Test rank of the feet.
    tcout << "Rank of the right foot "<< endl;
    tcout << aHDR->rightFoot()->associatedAnkle()->rankInConfiguration() << endl;
    CjrlFoot *rightFoot = aHDR->rightFoot();
    DisplayFoot(rightFoot,empty,tcout);
    
    tcout << "Rank of the left foot "<< endl;
    tcout << aHDR->leftFoot()->associatedAnkle()->rankInConfiguration() << endl;
    CjrlFoot *leftFoot = aHDR->leftFoot();
    DisplayFoot(leftFoot,empty,tcout);

  }
  void DisplayHumanoid(CjrlHumanoidDynamicRobot *aHDR,
		       ostream &os)
  {
    os << "NbOfDofs :" << aHDR->numberDof() << std::endl;
    RecursiveDisplayOfJoints(aHDR->rootJoint(),
			     os,10);
    
    DisplayEndEffectors(aHDR,os);
    std::string empty("");
    DisplayActuated(aHDR,empty,os);

    os << "total mass " << aHDR->mass() 
       << " COM: " << aHDR->positionCenterOfMass() << endl;
  }

  bool CompareTwoFiles(char *RefFileName, char *OurFileName, char *ReportFile)
  {
    std::ifstream reffile(RefFileName,ifstream::in), 
      ourfile(OurFileName,ifstream::in);
    std::ofstream  reportfile(ReportFile,ofstream::out);
    unsigned int NbLine=0;
    bool readingok=true;

    if (!reportfile.is_open())
      cout << "Pb opening " << ReportFile<< endl;

    // Go through the reference file
    while(!reffile.eof())
      {

	char refline[25536],ourline[25536];
	reffile.getline(refline,25536);
	ourfile.getline(ourline,25536);

	// Check that the two files are read the same
	// manner.
	if (reffile.gcount()!=ourfile.gcount())
	  {
	    reportfile << "Error reading at line " << NbLine 
		       << endl << "ref (count): " << reffile.gcount() 
		       << endl << "our (count): " << reffile.gcount() 
		       << endl << "ref: " << refline 
		       << endl << "our: " << ourline <<endl;

	    readingok=false;
	  }
	else
	  {
	    // If they are read the same way
	    // check if they provide the same data.
	    for(int i=0;i<reffile.gcount();i++)
	      if (refline[i]!=ourline[i])
		{
		  readingok=false;
		  reportfile << "Error at column " << i << endl;
		  break;
		}
	  }

	NbLine++;
      }

    if (readingok)
      {
	reportfile << "Comparison successfull." << endl;
      }
    // Close files.
    if (reffile.is_open())
      reffile.close();

    if (ourfile.is_open())
      ourfile.close();
    
    if (reportfile.is_open())
      reportfile.close();
    
    return readingok;
  }
};
