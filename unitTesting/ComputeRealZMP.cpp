/*
 * Copyright 2010, 
 *
 * Florent Lamiraux
 * Olivier Stasse,
 * 
 *
 * JRL/LAAS, CNRS/AIST
 *
 * This file is part of dynamicsJRLJapan.
 * dynamicsJRLJapan is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * dynamicsJRLJapan is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Lesser Public License for more details.
 * You should have received a copy of the GNU Lesser General Public License
 * along with dynamicsJRLJapan.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  Research carried out within the scope of the Associated
 *  International Laboratory: Joint Japanese-French Robotics
 *  Laboratory (JRL)
 *
 */

#include <string>
#include <fstream>

#include "jrl/mal/matrixabstractlayer.hh"
#include "jrl/dynamics/dynamicsfactory.hh"

using namespace std;
using namespace dynamicsJRLJapan;

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

void ExtractActualWaist(const CjrlJoint *LeftFoot2,
			const CjrlJoint *RightFoot2,
			CjrlJoint *Waist2,
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
      (MAL_S4x4_MATRIX_ACCESS_I_J(TrLF2,2,3)<
       MAL_S4x4_MATRIX_ACCESS_I_J(TrRF2,2,3)) &&
      (fabs(MAL_S4x4_MATRIX_ACCESS_I_J(TrRF2,2,3)-0.105)>1e-3))
    {
      CurrentSupportFootPosInWorld = TrLF2;
      CurrentSupportFoot = 1;
      cout << "Choice 1" << endl;
    }
  else
    {
      if ((MAL_S4x4_MATRIX_ACCESS_I_J(TrLF2,2,3)>
	   MAL_S4x4_MATRIX_ACCESS_I_J(TrRF2,2,3)) &&
	  (fabs(MAL_S4x4_MATRIX_ACCESS_I_J(TrLF2,2,3)-0.105)>1e-3))
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

  MAL_S4x4_INVERSE(CurrentSupportFootPosInWorld,CurrentWorldPosInSupportFoot,double);

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
	  MAL_S4x4_INVERSE(TrRF2,tmp,double);
	  //LocalInversionMatrix4dHomogeneous(TrRF2,tmp);
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
	  MAL_S4x4_INVERSE(TrLF2,tmp,double);
	  //LocalInversionMatrix4dHomogeneous(TrLF2,tmp);
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
    const double & nx = MAL_S4x4_MATRIX_ACCESS_I_J(CurrentAbsWaistPos,2,2);
    const double & ny = MAL_S4x4_MATRIX_ACCESS_I_J(CurrentAbsWaistPos,2,1);
    
    WaistFromActual[3] = atan2(ny,nx);
    WaistFromActual[4] = atan2(-MAL_S4x4_MATRIX_ACCESS_I_J(CurrentAbsWaistPos,2,0),
			       sqrt(ny*ny+nx*nx));
    WaistFromActual[5] = atan2(MAL_S4x4_MATRIX_ACCESS_I_J(CurrentAbsWaistPos,1,0),
			       MAL_S4x4_MATRIX_ACCESS_I_J(CurrentAbsWaistPos,0,0));
    WaistFromActual[0] = MAL_S4x4_MATRIX_ACCESS_I_J(CurrentAbsWaistPos,0,3);
    WaistFromActual[1] = MAL_S4x4_MATRIX_ACCESS_I_J(CurrentAbsWaistPos,1,3);
    WaistFromActual[2] = MAL_S4x4_MATRIX_ACCESS_I_J(CurrentAbsWaistPos,2,3);
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
  string aMapFromCjrlJointToRank;

  string RefLogFile;
  string ActualLogFile;
  if (argc!=5)
    {
      const char *envrobotpath="ROBOTPATH";
      char *robotpath = 0;
      const char *envrobotname="ROBOT";
      char *robotname = 0;
      robotpath = getenv(envrobotpath);
      robotname = getenv(envrobotname);

      if ((robotpath==0) || (robotname==0))
	{
	  cerr << " This program takes 6 arguments: " << endl;
	  cerr << "./TestHumanoidDynamicRobot PATH_TO_VRML_FILE VRML_FILE_NAME "<< endl;
	  cerr << " PATH_TO_SPECIFICITIES_XML PATH PATH_TO_MAP_JOINT_2_RANK" << endl;
	  cerr << " ReferenceLogFile ActualLogFile" << endl;
	  exit(-1);
	}	
      else
	{
	  aPath=robotpath;
	  aName=robotname;
	  aName+="main.wrl";
	  aSpecificitiesFileName = robotpath;
	  aSpecificitiesFileName +="/../etc/";
	  aSpecificitiesFileName += robotname;
	  aSpecificitiesFileName += "Specificities.xml";
	  aMapFromCjrlJointToRank = robotpath;
	  aMapFromCjrlJointToRank += "/../etc/";
	  aMapFromCjrlJointToRank += robotname;
	  aMapFromCjrlJointToRank += "JointRank.xml";
	  
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
      aMapFromCjrlJointToRank = argv[4];
    }
  
  dynamicsJRLJapan::ObjectFactory dynFactory;
  CjrlHumanoidDynamicRobot * aHDR  = dynFactory.createHumanoidDynamicRobot();
  CjrlHumanoidDynamicRobot * aHDR2 = dynFactory.createHumanoidDynamicRobot();

  if (aHDR==0)
    { 
      cerr<< "Dynamic cast on HDR failed " << endl;
      exit(-1);
  }
  cout << "Robot's model file:" << aPath << aName << endl;
  cout << "Specificities file:" << aSpecificitiesFileName << endl;
  cout << "Map from joint to rank:" << aMapFromCjrlJointToRank << endl;
  string RobotFileName = aPath + aName;
  parseOpenHRPVRMLFile(*aHDR,RobotFileName,
		       aMapFromCjrlJointToRank,aSpecificitiesFileName);
  parseOpenHRPVRMLFile(*aHDR2,RobotFileName,
		       aMapFromCjrlJointToRank,aSpecificitiesFileName);

  // Display tree of the joints.

  int NbOfDofs = aHDR->numberDof();
  std::cout << "NbOfDofs :" << NbOfDofs << std::endl;
  MAL_VECTOR_DIM(aCurrentConf,double,NbOfDofs);
  int lindex=0;
  for(int i=0;i<6;i++)
    aCurrentConf[lindex++] = 0.0;
  
  for(int i=0;i<(NbOfDofs-6 < 41 ? NbOfDofs-6 : 40) ;i++)
    aCurrentConf[lindex++] = 0.0;
  
  aHDR->currentConfiguration(aCurrentConf);

  aHDR2->currentConfiguration(aCurrentConf);

  const CjrlJoint * LeftFoot = aHDR->leftFoot()->associatedAnkle();
  const CjrlJoint * RightFoot = aHDR->rightFoot()->associatedAnkle();

  const CjrlJoint * LeftFoot2 = aHDR2->leftFoot()->associatedAnkle();
  const CjrlJoint * RightFoot2 = aHDR2->rightFoot()->associatedAnkle();

  CjrlJoint * Waist2 = aHDR2->waist();

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

  // Set properties for the first model.
  {
    string inProperty[4]={"TimeStep","ComputeAcceleration",
			  "ComputeBackwardDynamics", "ComputeZMP"};
    string inValue[4]={"0.005","true","true","true"};
    for(unsigned int i=0;i<4;i++)
      aHDR->setProperty(inProperty[i],inValue[i]);
  }
  // Set properties for the second model.
  {
    string inProperty[4]={"TimeStep","ComputeAcceleration",
			  "ComputeBackwardDynamics", "ComputeZMP"};
    string inValue[4]={"0.005","false","false","false"};
    for(unsigned int i=0;i<4;i++)
      aHDR2->setProperty(inProperty[i],inValue[i]);

  }

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

      aHDR2->currentConfiguration(aCurrentConf);
      aHDR2->computeForwardKinematics();

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
      
      aHDR->currentConfiguration(aCurrentConf);
      aHDR->computeForwardKinematics();

      vector3d ZMPval;
      ZMPval = aHDR->zeroMomentumPoint();
     
      matrix4d TrLF = LeftFoot->currentTransformation();
      matrix4d TrRF = RightFoot->currentTransformation();
      matrix4d TrLF2 = LeftFoot2->currentTransformation();
      matrix4d TrRF2 = RightFoot2->currentTransformation();

      double lnorm  = NormalForces[0] + NormalForces[1];
      
      RebuildZMP.open("RebuildZMP.dat",ofstream::app);
      RebuildZMP << (MAL_S4x4_MATRIX_ACCESS_I_J(TrLF,0,3) * NormalForces[0] + 
		     MAL_S4x4_MATRIX_ACCESS_I_J(TrRF,0,3) * NormalForces[1])/ lnorm  << " "
		 << (MAL_S4x4_MATRIX_ACCESS_I_J(TrLF,1,3) * NormalForces[0] + 
		     MAL_S4x4_MATRIX_ACCESS_I_J(TrRF,1,3) * NormalForces[1])/ lnorm  << " " 
		 << ZMPval(0) << " " << ZMPval(1) << " " 
		 << MAL_S4x4_MATRIX_ACCESS_I_J(TrLF,0,3) << " " 
		 << MAL_S4x4_MATRIX_ACCESS_I_J(TrLF,1,3) << " " 
		 << MAL_S4x4_MATRIX_ACCESS_I_J(TrLF,2,3) << " " 
		 << MAL_S4x4_MATRIX_ACCESS_I_J(TrRF,0,3) << " " 
		 << MAL_S4x4_MATRIX_ACCESS_I_J(TrRF,1,3) << " " 
		 << MAL_S4x4_MATRIX_ACCESS_I_J(TrRF,2,3) << " "
		 << MAL_S4x4_MATRIX_ACCESS_I_J(AbsSupportFootPos,0,3) << " " 
		 << MAL_S4x4_MATRIX_ACCESS_I_J(AbsSupportFootPos,1,3) << " " 
		 << MAL_S4x4_MATRIX_ACCESS_I_J(AbsSupportFootPos,2,3) << " "
		 << PreviousSupportFoot << " " 
		 << MAL_S4x4_MATRIX_ACCESS_I_J(TrLF2,0,3) << " " 
		 << MAL_S4x4_MATRIX_ACCESS_I_J(TrLF2,1,3) << " " 
		 << MAL_S4x4_MATRIX_ACCESS_I_J(TrLF2,2,3) << " " 
		 << MAL_S4x4_MATRIX_ACCESS_I_J(TrRF2,0,3) << " " 
		 << MAL_S4x4_MATRIX_ACCESS_I_J(TrRF2,1,3) << " " 
		 << MAL_S4x4_MATRIX_ACCESS_I_J(TrRF2,2,3) << endl;
      //92
      RebuildZMP.close();

      
      NbIt++;
    }
  
  ActualStateFile.close();
  
  delete aHDR;
  
}
