#include <string>
#include <fstream>

#include <MatrixAbstractLayer/MatrixAbstractLayer.h>
#include "dynamicsJRLJapan/Joint.h"
#include "dynamicsJRLJapan/HumanoidDynamicMultiBody.h"
#include "robotDynamics/jrlRobotDynamicsObjectConstructor.h"

#include "jrlMathTools/jrlConstants.h"

using namespace std;
using namespace dynamicsJRLJapan;

void LocalInversionMatrix4dHomogeneous(matrix4d &a, matrix4d &inva)
{
  inva(0,0) = a(0,0); inva(0,1) = a(1,0); inva(0,2) = a(2,0); 
  inva(1,0) = a(0,1); inva(1,1) = a(1,1); inva(1,2) = a(2,1); 
  inva(2,0) = a(0,2); inva(2,1) = a(1,2); inva(2,2) = a(2,2); 

  inva(0,3) = -(inva(0,0)*a(0,3)+inva(0,1)*a(1,3)+inva(0,2)*a(2,3));
  inva(1,3) = -(inva(1,0)*a(0,3)+inva(1,1)*a(1,3)+inva(1,2)*a(2,3));
  inva(2,3) = -(inva(2,0)*a(0,3)+inva(2,1)*a(1,3)+inva(2,2)*a(2,3));
  
  inva(3,0) = inva(3,1) = inva(3,2) = 0.0;
  inva(3,3) = 1.0;
}

void ExtractRefWaist(ifstream &RefStateFile,
		     double *WaistFromRef,
		     double *RotationFreeFlyer,
		     double *RefData)
{
  if (!RefStateFile.eof())
    {
      
      // Read the position and the orientation of the waist.
      for(unsigned int i=0;i<100;i++)
	{
	  RefStateFile >> RefData[i];
	  if ((i>=89) && (i<98))
	    RotationFreeFlyer[i-89] = RefData[i];
	  
	  const double & nx = RotationFreeFlyer[2*3+2];
	  const double & ny = RotationFreeFlyer[2*3+1];
	  
	  WaistFromRef[3] = atan2(ny,nx);
	  WaistFromRef[4] = atan2(-RotationFreeFlyer[2*3+0],
				  sqrt(ny*ny+nx*nx));
	  WaistFromRef[5] = atan2(RotationFreeFlyer[1*3+0],
				  RotationFreeFlyer[0*3+0]);
	  
	}
      
      WaistFromRef[0] = RefData[86];
      WaistFromRef[1] = RefData[87];
      WaistFromRef[2] = RefData[88];
    }
}

void ExtractActualWaist(Joint *LeftFoot2,
			Joint *RightFoot2,
			Joint *Waist2,
			matrix4d &AbsSupportFootPos,
			double * WaistFromRef,
			double * RotationFreeFlyer,
			int NbIt,
			double* WaistFromActual,
			int &PreviousSupportFoot)
{

  int CurrentSupportFoot=PreviousSupportFoot;

  matrix4d CurrentSupportFootPosInWorld,
    CurrentWaistPosInSupportFoot, CurrentAbsWaistPos;
  
  matrix4d TrLF2 = LeftFoot2->currentTransformation();
  matrix4d TrRF2 = RightFoot2->currentTransformation();
  matrix4d CurrentWaistInWorld2 = Waist2->currentTransformation();
  if (
      (TrLF2(2,3)<TrRF2(2,3)) &&
      (fabs(TrRF2(2,3)-0.105)>1e-3))
    {
      CurrentSupportFootPosInWorld = TrLF2;
      CurrentSupportFoot = 1;
      cout << "Choice 1" << endl;
    }
  else
    {
      if ((TrLF2(2,3)>TrRF2(2,3)) &&
	  (fabs(TrLF2(2,3)-0.105)>1e-3))
	{
	  CurrentSupportFootPosInWorld = TrRF2;
	  CurrentSupportFoot = -1;
	  cout << "Choice 2" << endl;
	}
      else
	{
	  if (PreviousSupportFoot==1)
	    {
	      CurrentSupportFootPosInWorld = TrLF2;
	      cout << "Choice 3" << endl;
	    }
	  else
	    {
	      CurrentSupportFootPosInWorld = TrRF2;
	      cout << "Choice 4" << endl;
	    }
	}
    }
  cout << "CurrentWaistInWorld2" << CurrentWaistInWorld2 << endl;
  cout << "CurrentSupportFootPosInWorld" << CurrentSupportFootPosInWorld<< endl;

  matrix4d CurrentWorldPosInSupportFoot;

  LocalInversionMatrix4dHomogeneous(CurrentSupportFootPosInWorld,CurrentWorldPosInSupportFoot);
  cout << "CurrentWorldPosInSupportFoot" << CurrentWorldPosInSupportFoot << endl;
  MAL_S4x4_C_eq_A_by_B(CurrentWaistPosInSupportFoot,CurrentWorldPosInSupportFoot,CurrentWaistInWorld2);
  cout << "CurrentWaistPosInSupportFoot : " << CurrentWaistPosInSupportFoot << endl;

  if (NbIt==0)
    {
      AbsSupportFootPos =  CurrentSupportFootPosInWorld;
    }
  else
    {
      matrix4d tmp;
      
      if ((CurrentSupportFoot==1) && (PreviousSupportFoot==-1))
	{
	  LocalInversionMatrix4dHomogeneous(TrRF2,tmp);
	  // Put Waist in support foot pos during Change support.
	  // we moved from right support foot to left support foot.

	  // Put Waist in absolute ref (right).
	  AbsSupportFootPos = MAL_S4x4_RET_A_by_B(AbsSupportFootPos,
						  tmp);

	  // Put Current support foot in absolute ref.
	  AbsSupportFootPos = MAL_S4x4_RET_A_by_B(AbsSupportFootPos,
						  CurrentSupportFootPosInWorld);

	  cout << "Choice 5" <<endl;
	}
      
      if ((CurrentSupportFoot==-1) && (PreviousSupportFoot==1))
	{
	  LocalInversionMatrix4dHomogeneous(TrLF2,tmp);
	  AbsSupportFootPos = MAL_S4x4_RET_A_by_B(AbsSupportFootPos,
						  tmp);
	  AbsSupportFootPos = MAL_S4x4_RET_A_by_B(AbsSupportFootPos,
						  CurrentSupportFootPosInWorld);
	  cout << "Choice 6" <<endl;
	}
      
    }
  cout << "AbsSupportFootPos : " << AbsSupportFootPos << endl;
  MAL_S4x4_C_eq_A_by_B(CurrentAbsWaistPos,AbsSupportFootPos,CurrentWaistPosInSupportFoot);
  cout << "CurrentAbsWaistPos: " << CurrentAbsWaistPos << endl;

  {
    const double & nx = CurrentAbsWaistPos(2,2);
    const double & ny = CurrentAbsWaistPos(2,1);
    
    WaistFromActual[3] = atan2(ny,nx);
    WaistFromActual[4] = atan2(-CurrentAbsWaistPos(2,0),
			       sqrt(ny*ny+nx*nx));
    WaistFromActual[5] = atan2(CurrentAbsWaistPos(1,0),
			       CurrentAbsWaistPos(0,0));
    WaistFromActual[0] = CurrentAbsWaistPos(0,3);
    WaistFromActual[1] = CurrentAbsWaistPos(1,3);
    WaistFromActual[2] = CurrentAbsWaistPos(2,3);
  }

  
  PreviousSupportFoot = CurrentSupportFoot;
  
}

void SaveWaistPositions(double *WaistRef,
			double *WaistActual)
{  
  ofstream RebuildWaist;
  RebuildWaist.open("RebuildWaist.dat",ofstream::app);
  for(unsigned int i=0;i<6;i++)
    RebuildWaist  << WaistRef[i] << " ";

  for(unsigned int i=0;i<6;i++)
    RebuildWaist  << WaistActual[i] << " ";
    
  RebuildWaist << endl;

  RebuildWaist.close();
      
}

int main(int argc, char *argv[])
{
  string aSpecificitiesFileName;
  string aPath;
  string aName;
  string aMapFromJointToRank;

  string RefLogFile;
  string ActualLogFile;
  if (argc!=5)
    {
      const char *openhrphome="OPENHRPHOME";
      char *value = 0;
      value = getenv(openhrphome);
      if (value==0)
	{
	  cerr << " This program takes 6 arguments: " << endl;
	  cerr << "./TestHumanoidDynamicRobot PATH_TO_VRML_FILE VRML_FILE_NAME "<< endl;
	  cerr << " PATH_TO_SPECIFICITIES_XML PATH PATH_TO_MAP_JOINT_2_RANK" << endl;
	  cerr << " ReferenceLogFile ActualLogFile" << endl;
	  exit(-1);
	}	
      else
	{
	  aPath=value;
	  aPath+="Controller/IOserver/robot/HRP2JRL/model/";
	  aName="HRP2JRLmain.wrl";
	  aSpecificitiesFileName = value;
	  aSpecificitiesFileName +="Controller/IOserver/robot/HRP2JRL/etc/";
	  aSpecificitiesFileName += "HRP2Specificities.xml";
	  aMapFromJointToRank = value;
	  aMapFromJointToRank += "Controller/IOserver/robot/HRP2JRL/etc/";
	  aMapFromJointToRank += "HRP2LinkJointRank.xml";
	  
	}

      if (argc==3)
	{
	  RefLogFile = argv[1];
	  ActualLogFile = argv[2];
	}
    }	
  else 
    {
      aPath=argv[1];
      aName=argv[2];
      aSpecificitiesFileName = argv[3];
      aMapFromJointToRank = argv[4];
    }


  CjrlRobotDynamicsObjectConstructor<
  dynamicsJRLJapan::DynamicMultiBody, 
    dynamicsJRLJapan::HumanoidDynamicMultiBody, 
    dynamicsJRLJapan::JointFreeflyer, 
    dynamicsJRLJapan::JointRotation,
    dynamicsJRLJapan::JointTranslation,
    dynamicsJRLJapan::Body> aRobotDynamicsObjectConstructor;
  
  CjrlHumanoidDynamicRobot * aHDR = aRobotDynamicsObjectConstructor.createhumanoidDynamicRobot();
  
  HumanoidDynamicMultiBody *aHDMB;
  aHDMB = dynamic_cast<dynamicsJRLJapan::HumanoidDynamicMultiBody*>(aHDR);

  CjrlHumanoidDynamicRobot * aHDR2 = aRobotDynamicsObjectConstructor.createhumanoidDynamicRobot();
  HumanoidDynamicMultiBody *aHDMB2;
  aHDMB2 = dynamic_cast<dynamicsJRLJapan::HumanoidDynamicMultiBody*>(aHDR2);

  if (aHDMB==0)
    { 
      cerr<< "Dynamic cast on HDR failed " << endl;
      exit(-1);
  }
  cout << "Robot's model file:" << aPath << aName << endl;
  cout << "Specificities file:" << aSpecificitiesFileName << endl;
  cout << "Map from joint to rank:" << aMapFromJointToRank << endl;

  aHDMB->parserVRML(aPath,aName,(char *)aMapFromJointToRank.c_str());
  aHDMB->SetHumanoidSpecificitiesFile(aSpecificitiesFileName);

  aHDMB2->parserVRML(aPath,aName,(char *)aMapFromJointToRank.c_str());
  aHDMB2->SetHumanoidSpecificitiesFile(aSpecificitiesFileName);
  
  // Display tree of the joints.

  int NbOfDofs = aHDMB->numberDof();
  std::cout << "NbOfDofs :" << NbOfDofs << std::endl;
  MAL_VECTOR_DIM(aCurrentConf,double,NbOfDofs);
  int lindex=0;
  for(int i=0;i<6;i++)
    aCurrentConf[lindex++] = 0.0;
  
  for(int i=0;i<(NbOfDofs-6 < 40 ? NbOfDofs-6 : 40) ;i++)
    aCurrentConf[lindex++] = 0.0;
  
  aHDMB->currentConfiguration(aCurrentConf);

  aHDMB2->currentConfiguration(aCurrentConf);

  Joint * LeftFoot = (Joint *)aHDMB->leftFoot();
  Joint * RightFoot = (Joint *)aHDMB->rightFoot();

  Joint * LeftFoot2 = (Joint *)aHDMB2->leftFoot();
  Joint * RightFoot2 = (Joint *)aHDMB2->rightFoot();

  Joint * LeftWrist = (Joint *)aHDMB->leftWrist();
  Joint * RightWrist = (Joint *)aHDMB->rightWrist();

  Joint * Waist2 = (Joint *)aHDMB2->waist();

  // Read the data file.
  ifstream ActualStateFile;
  ActualStateFile.open((char *)ActualLogFile.c_str(),ifstream::in);
  if (!ActualStateFile.is_open())
    {
      cerr << "Unable to open actual state file: " << 
	ActualLogFile << endl;
      exit(-1);
    }

  ifstream RefStateFile;
  RefStateFile.open((char *)RefLogFile.c_str(),ifstream::in);
  if (!RefStateFile.is_open())
    {
      cerr << "Unable to open reference state file: " << 
	RefLogFile << endl;
      exit(-1);
    }
  
  ofstream RebuildZMP;
  RebuildZMP.open("RebuildZMP.dat",ofstream::out);
  RebuildZMP.close();

  ofstream RebuildForces;
  RebuildForces.open("RebuildForces.dat",ofstream::out);
  RebuildForces.close();

  ofstream RebuildWaist;
  RebuildWaist.open("RebuildWaist.dat",ofstream::out);
  RebuildWaist.close();

  aHDMB->SetTimeStep(0.005);
  aHDMB->setComputeAcceleration(true);
  aHDMB->setComputeBackwardDynamics(true);
  aHDMB->setComputeZMP(true);

  aHDMB2->SetTimeStep(0.005);
  aHDMB2->setComputeAcceleration(false);
  aHDMB2->setComputeBackwardDynamics(false);
  aHDMB2->setComputeZMP(false);

  // Read the first line of actual state:
  ofstream ASD("ActualStateDescription.dat");
  if (ASD.is_open())
    {
      for(unsigned int i=0;i<126;i++)
	{
	  string tmp;
	  ActualStateFile>> tmp;
	  ASD  << i << " : " << tmp << endl;
	}
      ASD.close();
    }

  ofstream RSD("ReferenceStateDescription.dat");
  if (RSD.is_open())
    {
      for(unsigned int i=0;i<95;i++)
	{
	  string tmp;
	  RefStateFile>> tmp;
	  RSD  << i << " : " << tmp << endl;
	}
      RSD.close();
    }

  unsigned long int NbIt=0;
  double WaistFromRef[6];
  double WaistFromActual[6];
  matrix4d AbsSupportFootPos;
  // We set the first support foot as being the left one.
  int PreviousSupportFoot=-1;

  while(!ActualStateFile.eof())
    {
      double NormalForces[2]={0.0,0.0};
      double ActualData[131];
      double RefData[100];
  
      double RotationFreeFlyer[9];

      for(unsigned int i=0;i<131;i++)
	{
	  ActualStateFile >> ActualData[i];

	  if (i<40)
	    aCurrentConf[i+6] = ActualData[i];
	  if (i==82)
	    NormalForces[1]=ActualData[i];

	  if (i==88)
	    NormalForces[0]=ActualData[i];
	}

      ExtractRefWaist(RefStateFile,WaistFromRef,RotationFreeFlyer,RefData);

      for(unsigned int i=0;i<6;i++)
	aCurrentConf[i]=WaistFromRef[i];

      for(unsigned int i=0;i<40;i++)
	aCurrentConf[i+6]=RefData[i];

      aHDMB2->currentConfiguration(aCurrentConf);
      aHDMB2->computeForwardKinematics();

      ExtractActualWaist(LeftFoot2,
			 RightFoot2,
			 Waist2,
			 AbsSupportFootPos,
			 WaistFromRef,
			 RotationFreeFlyer,
			 NbIt,
			 WaistFromActual,
			 PreviousSupportFoot);
			 
      SaveWaistPositions(WaistFromRef, WaistFromActual);

      if (1)
	{
	  for(unsigned int i=0;i<6;i++)
	    aCurrentConf[i] = WaistFromActual[i];
	}
      else
	{
	   for(unsigned int i=0;i<6;i++)
	     aCurrentConf[i] = WaistFromRef[i];
	}
      
      aHDMB->currentConfiguration(aCurrentConf);
      aHDMB->computeForwardKinematics();

      vector3d ZMPval;
      ZMPval = aHDMB->zeroMomentumPoint();
     
      matrix4d TrLF = LeftFoot->currentTransformation();
      matrix4d TrRF = RightFoot->currentTransformation();
      matrix4d TrLF2 = LeftFoot2->currentTransformation();
      matrix4d TrRF2 = RightFoot2->currentTransformation();

      double lnorm  = NormalForces[0] + NormalForces[1];
      
      RebuildZMP.open("RebuildZMP.dat",ofstream::app);
      RebuildZMP << (TrLF(0,3) * NormalForces[0] + TrRF(0,3) * NormalForces[1])/ lnorm  << " "
		 << (TrLF(1,3) * NormalForces[0] + TrRF(1,3) * NormalForces[1])/ lnorm  << " " 
		 << ZMPval(0) << " " << ZMPval(1) << " " 
		 << TrLF(0,3) << " " << TrLF(1,3) << " " << TrLF(2,3) << " " 
		 << TrRF(0,3) << " " << TrRF(1,3) << " " << TrRF(2,3) << " "
		 << AbsSupportFootPos(0,3) << " " 
		 << AbsSupportFootPos(1,3) << " " 
		 << AbsSupportFootPos(2,3) << " "
		 << PreviousSupportFoot << " " 
		 << TrLF2(0,3) << " " << TrLF2(1,3) << " " << TrLF2(2,3) << " " 
		 << TrRF2(0,3) << " " << TrRF2(1,3) << " " << TrRF2(2,3) << endl;
      //92
      RebuildZMP.close();

      RebuildForces.open("RebuildForces.dat",ofstream::app);
      for(unsigned int i=0;i<3;i++)
	RebuildForces << ((DynamicBody *)(LeftWrist->linkedBody()))->m_Force(i) << " " ;
      for(unsigned int i=0;i<3;i++)
	RebuildForces << ((DynamicBody *)(RightWrist->linkedBody()))->m_Force(i) << " ";
      RebuildForces << endl;
      RebuildForces.close();
      
      NbIt++;
    }
  
  ActualStateFile.close();
  
  delete aHDMB;
  
}
