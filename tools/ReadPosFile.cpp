/*
 * Copyright 2010,
 *
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

/* Store configuration file names*/
typedef struct 
{
  string SpecificitiesFileName;
  string Path;
  string Name;
  string MapFromCjrlJointToRank;
  string RobotFileName;
  string PosFileName;
} ConfigurationFiles_t;

/* X-Y-Z Euler angles position, derivative, second derivative */
class EulerAngles_t {
public:
  vector3d xyz;
  vector3d dxdydz;
  vector3d ddxddyddz;

  void computeAngularVelocity(vector3d w)
  {
    matrix3d Rz,Ry,Rx;
    matrix3d dRz,dRy,dRx,Final;

    double cz=0.0, sz=0.0, cy=0.0, sy=0.0,
      cx=0.0, sx=0.0;

    cx = cos(xyz(0)); sx = sin(xyz(0));
    cy = cos(xyz(1)); sy = sin(xyz(1));
    cz = cos(xyz(2)); sz = sin(xyz(2));

    Rx(1,1) = Rx(2,2) = cx; 
    Rx(0,0) = 1.0;
    Rx(1,2) = -sx; Rx(2,1) = sx;

    Ry(0,0) = cy; Rx(0,2) = sy;
    Ry(1,1) = 1.0;
    Ry(2,0) = -sy; Ry(2,2) = cy;

    Rz(0,0) = cz; Rz(0,1) = -sz;
    Rz(1,0) = sz; Rz(1,1) = cz;
    Rz(2,2) = 1.0;

    dRz(0,0) = 1.0;
    dRz(1,1) = -dxdydz(2)*sz; dRz(1,2) = dxdydz(2)*cz;
    dRz(2,1) = -dRz(1,2); dRz(2,2) = dRz(1,1);

    dRy(0,0) = -dxdydz(1)*sy; dRy(0,2) = dxdydz(1)*cy;
    dRy(1,1) = 1.0;
    dRy(2,1) = -dxdydz(1)*cy; dRy(2,2) = -dxdydz(1)*sy;
    
    dRx(0,0) = -dxdydz(0)*sy; dRy(0,1) = -dxdydz(0)*cy;
    dRx(1,0) = dxdydz(0)*cy; dRy(1,1) = -dxdydz(0)*sy;
    dRx(2,2) = 1.0;
    
    Final = dRz * Ry * Rx + Rz * dRy * Rx + Rz * Ry * dRx;
    //cout << "xyz-euler: " << xyz << endl;
    //cout << "dxdydz-euler: " << dxdydz << endl;
    w(0) = Final(2,1);
    w(1) = Final(0,2);
    w(2) = Final(1,0);
  }
};

/* Store the current state of a robot */
class RobotState_t 
{
private:
  unsigned int m_NbDofs;
  EulerAngles_t m_EulerAngles;

public:
  MAL_VECTOR(aCurrentConf,double);
  MAL_VECTOR(aPrevCurrentConf,double);
  MAL_VECTOR(aCurrentSpeed,double);
  MAL_VECTOR(aPrevCurrentSpeed,double);
  MAL_VECTOR(aCurrentAcceleration,double);

  
  void setNbDofs(unsigned int NbDofs)
  { 
    m_NbDofs = NbDofs;
    MAL_VECTOR_RESIZE(aCurrentConf,m_NbDofs);
    MAL_VECTOR_RESIZE(aPrevCurrentConf,m_NbDofs);
    MAL_VECTOR_RESIZE(aCurrentSpeed,m_NbDofs);
    MAL_VECTOR_RESIZE(aPrevCurrentSpeed,m_NbDofs);
    MAL_VECTOR_RESIZE(aCurrentAcceleration,m_NbDofs);    
  };

  void reset()
  {
    int lindex=0;
    for(unsigned int i=0;i<m_NbDofs;i++)
      aCurrentConf[lindex++] = 0.0;

  }

  void updateConfFromFreeFlyer(vector<double> &Waist)
  {
    if (MAL_VECTOR_SIZE(aCurrentConf)<6)
      throw("RobotState_t::UpdateConfFromFreeFlyer: current configuration <6.");

    for(unsigned int i=0;i<Waist.size();i++)
      aCurrentConf[i]=Waist[i];
  }

  void updateConfFromActualData(vector<double> ActualData)
  {
    
    if (ActualData.size()<m_NbDofs-6)
      throw("RobotState_t::UpdateConfFromActualData: ActualData < NbOfDofs-6 of the robot.");

    for(unsigned int i=0;i<m_NbDofs-6;i++)
      aCurrentConf[i+6] = ActualData[i+1];
  }

  void updateRobot(CjrlHumanoidDynamicRobot *aHDR)
  {
    aHDR->currentConfiguration(aCurrentConf);
    aHDR->currentVelocity(aCurrentSpeed);
    aHDR->currentAcceleration(aCurrentAcceleration);
  }

  void updateCurrentRobotState(double SamplingPeriod,
			       long unsigned int & NbIt)
  {
    if (NbIt>=1)
      {
	for(unsigned int i=0;i<6;i++)
	  aCurrentSpeed[i]=(aCurrentConf[i]-aPrevCurrentConf[i])/SamplingPeriod;
	
	for(unsigned int i=0;i<3;i++)
	  {
	    // Set the current euler angles values.
	    m_EulerAngles.xyz(i) = aCurrentConf[i+3];
	    // Set the current euler angles derivatives.
	    m_EulerAngles.dxdydz(i) = aCurrentSpeed[i+3];
	  }
	vector3d w;

	// Compute proper angular velocity.
	m_EulerAngles.computeAngularVelocity(w);

	for(unsigned int i=0;i<3;i++)
	  aCurrentSpeed[i+3]=w(i);
	
	for(unsigned int i=6;i<36;i++)
	  aCurrentSpeed[i]=(aCurrentConf[i]-aPrevCurrentConf[i])/SamplingPeriod;
      }
    else
      for(unsigned int i=0;i<36;i++)
	aCurrentSpeed[i]=0.0;
    
    if (NbIt>=2)
      {
	for(unsigned int i=0;i<6;i++)
	  aCurrentAcceleration[i]=(aCurrentSpeed[i]-aPrevCurrentSpeed[i])/SamplingPeriod;
	
	for(unsigned int i=6;i<36;i++)
	  aCurrentAcceleration[i]=(aCurrentSpeed[i]-aPrevCurrentSpeed[i])/SamplingPeriod;
      }
    else
      for(unsigned int i=0;i<36;i++)
	aCurrentAcceleration[i]=0.0;
  }

};


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


void ExtractActualWaist(CjrlHumanoidDynamicRobot * aHDR2,
			matrix4d &AbsSupportFootPos,
			vector<double> & ,
			double * ,
			int NbIt,
			vector<double>& WaistFromActual,
			int &PreviousSupportFoot)
{
  CjrlJoint *Waist2 = aHDR2->waist();

  int CurrentSupportFoot=PreviousSupportFoot;

  matrix4d CurrentSupportFootPosInWorld,
    CurrentWaistPosInSupportFoot, CurrentAbsWaistPos;

  matrix4d TrLF2 = aHDR2->leftFoot()->associatedAnkle()->currentTransformation();
  matrix4d TrRF2 = aHDR2->rightFoot()->associatedAnkle()->currentTransformation();
  matrix4d CurrentWaistInWorld2 = Waist2->currentTransformation();
  if (
      (MAL_S4x4_MATRIX_ACCESS_I_J(TrLF2,2,3)<
       MAL_S4x4_MATRIX_ACCESS_I_J(TrRF2,2,3)) &&
      (fabs(MAL_S4x4_MATRIX_ACCESS_I_J(TrRF2,2,3)-MAL_S4x4_MATRIX_ACCESS_I_J(TrLF2,2,3))>1e-19))
    {
      CurrentSupportFootPosInWorld = TrLF2;
      CurrentSupportFoot = 1;
    }
  else
    {
      if ((MAL_S4x4_MATRIX_ACCESS_I_J(TrLF2,2,3)>
	   MAL_S4x4_MATRIX_ACCESS_I_J(TrRF2,2,3)) &&
	  (fabs(MAL_S4x4_MATRIX_ACCESS_I_J(TrLF2,2,3)-
		MAL_S4x4_MATRIX_ACCESS_I_J(TrRF2,2,3))>1e-19))
	{
	  CurrentSupportFootPosInWorld = TrRF2;
	  CurrentSupportFoot = -1;

	}
      else
	{
	  if (PreviousSupportFoot==1)
	    {
	      CurrentSupportFootPosInWorld = TrLF2;
	    }
	  else
	    {
	      CurrentSupportFootPosInWorld = TrRF2;
	    }
	}
    }

  matrix4d CurrentWorldPosInSupportFoot;

  MAL_S4x4_INVERSE(CurrentSupportFootPosInWorld,CurrentWorldPosInSupportFoot,double);

  //  cout << "CurrentWorldPosInSupportFoot" << CurrentWorldPosInSupportFoot << endl;
  MAL_S4x4_C_eq_A_by_B(CurrentWaistPosInSupportFoot,CurrentWorldPosInSupportFoot,CurrentWaistInWorld2);
  //  cout << "CurrentWaistPosInSupportFoot : " << CurrentWaistPosInSupportFoot << endl;

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
	}

      if ((CurrentSupportFoot==-1) && (PreviousSupportFoot==1))
	{
	  MAL_S4x4_INVERSE(TrLF2,tmp,double);
	  //LocalInversionMatrix4dHomogeneous(TrLF2,tmp);
	  AbsSupportFootPos = MAL_S4x4_RET_A_by_B(AbsSupportFootPos,
						  tmp);
	  AbsSupportFootPos = MAL_S4x4_RET_A_by_B(AbsSupportFootPos,
						  CurrentSupportFootPosInWorld);
	}

    }
  MAL_S4x4_C_eq_A_by_B(CurrentAbsWaistPos,AbsSupportFootPos,CurrentWaistPosInSupportFoot);

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


void SaveWaistPositions(vector<double> &WaistRef,
			vector<double> &WaistActual,
			int PreviousSupportFoot)
{
  ofstream RebuildWaist;
  RebuildWaist.open("RebuildWaist.dat",ofstream::app);
  for(unsigned int i=0;i<6;i++)
    RebuildWaist  << WaistRef[i] << " ";

  for(unsigned int i=0;i<6;i++)
    RebuildWaist  << WaistActual[i] << " ";

  RebuildWaist << PreviousSupportFoot << " ";
  RebuildWaist << endl;

  RebuildWaist.close();

}

void dealWithArguments(int argc, char *argv[],
		       ConfigurationFiles_t &SetOfConfigurationFiles)
{
  if (argc!=6)
    {
      const char *envrobotpath="ROBOTPATH";
      char *robotpath = 0;
      const char *envrobotname="ROBOT";
      char *robotname = 0;
      robotpath = getenv(envrobotpath);
      robotname = getenv(envrobotname);

      if ((robotpath==0) || (robotname==0))
	{
	  cerr << " This program takes 5 arguments: " << endl;
	  cerr << "./TestHumanoidDynamicRobot PATH_TO_VRML_FILE VRML_FILE_NAME "<< endl;
	  cerr << " PATH_TO_SPECIFICITIES_XML PATH PATH_TO_MAP_JOINT_2_RANK" << endl;
	  cerr << " PosFileName" << endl;
	  exit(-1);
	}
      else
	{
	  SetOfConfigurationFiles.Path=robotpath;
	  SetOfConfigurationFiles.Name=robotname;
	  SetOfConfigurationFiles.Name+="JRLmainSmallOld.wrl";
	  SetOfConfigurationFiles.SpecificitiesFileName = robotpath;
	  SetOfConfigurationFiles.SpecificitiesFileName += robotname;
	  SetOfConfigurationFiles.SpecificitiesFileName += "SpecificitiesSmallOld.xml";
	  SetOfConfigurationFiles.MapFromCjrlJointToRank = robotpath;
	  SetOfConfigurationFiles.MapFromCjrlJointToRank += robotname;
	  SetOfConfigurationFiles.MapFromCjrlJointToRank += "LinkJointRankSmallOld.xml";
	}

      if (argc==2)
	{
	  SetOfConfigurationFiles.PosFileName = argv[1];
	}
    }
  else
    {
      SetOfConfigurationFiles.Path=argv[1];
      SetOfConfigurationFiles.Name=argv[2];
      SetOfConfigurationFiles.SpecificitiesFileName = argv[3];
      SetOfConfigurationFiles.MapFromCjrlJointToRank = argv[4];
      SetOfConfigurationFiles.PosFileName = argv[5];
    }
}

void createRobots(ConfigurationFiles_t & aSetOfConfigurationFiles,
		  CjrlHumanoidDynamicRobot * &aHDR,
		  CjrlHumanoidDynamicRobot * &aHDR2 )
{
  // Instanciate robots.
  dynamicsJRLJapan::ObjectFactory dynFactory;
  aHDR  = dynFactory.createHumanoidDynamicRobot();
  aHDR2 = dynFactory.createHumanoidDynamicRobot();

  if (aHDR==0)
    {
      cerr<< "Dynamic cast on HDR failed " << endl;
      exit(-1);
    }

  cout << "Robot's model file:" 
       << aSetOfConfigurationFiles.Path 
       << aSetOfConfigurationFiles.Name << endl;
  cout << "Specificities file:" 
       << aSetOfConfigurationFiles.SpecificitiesFileName << endl;
  cout << "Map from joint to rank:" 
       << aSetOfConfigurationFiles.MapFromCjrlJointToRank << endl;
  aSetOfConfigurationFiles.RobotFileName = aSetOfConfigurationFiles.Path + 
    aSetOfConfigurationFiles.Name;
  parseOpenHRPVRMLFile(*aHDR,
		       aSetOfConfigurationFiles.RobotFileName,
		       aSetOfConfigurationFiles.MapFromCjrlJointToRank,
		       aSetOfConfigurationFiles.SpecificitiesFileName);
  parseOpenHRPVRMLFile(*aHDR2,
		       aSetOfConfigurationFiles.RobotFileName,
		       aSetOfConfigurationFiles.MapFromCjrlJointToRank,
		       aSetOfConfigurationFiles.SpecificitiesFileName);
}

void initializeRobots(CjrlHumanoidDynamicRobot * aHDR,
		      CjrlHumanoidDynamicRobot * aHDR2 )

{
  // Initialize the various properties of the Robots.
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
}

void saveZMP(CjrlHumanoidDynamicRobot *aHDR,
	     CjrlHumanoidDynamicRobot *aHDR2,
	     RobotState_t &robotState,
	     vector<double> ActualData)
{
  ofstream RebuildZMP;
  ofstream RebuildZMPW;

  const CjrlJoint * LeftFoot = aHDR->leftFoot()->associatedAnkle();
  const CjrlJoint * RightFoot = aHDR->rightFoot()->associatedAnkle();

  const CjrlJoint * LeftFoot2 = aHDR2->leftFoot()->associatedAnkle();
  const CjrlJoint * RightFoot2 = aHDR2->rightFoot()->associatedAnkle();
  CjrlJoint * Waist1 = aHDR->waist();

  vector3d ZMPval;
  ZMPval = aHDR->zeroMomentumPoint();
  vector3d COMval;
  COMval = aHDR->positionCenterOfMass();
      
  matrix4d TrLF = LeftFoot->currentTransformation();
  matrix4d TrRF = RightFoot->currentTransformation();
  matrix4d TrLF2 = LeftFoot2->currentTransformation();
  matrix4d TrRF2 = RightFoot2->currentTransformation();
  
  
  RebuildZMP.open("RebuildZMP.dat",ofstream::app);
  RebuildZMPW.open("RebuildZMPW.dat",ofstream::app);
  
  matrix4d waMwo, waMankle;
  double   woZfoot, waZfoot;
  
  matrix4d woMwa    = Waist1->currentTransformation();
  matrix4d woMankle = RightFoot->currentTransformation();
  
  MAL_S4x4_INVERSE(woMwa,waMwo,double);
  MAL_S4x4_C_eq_A_by_B(waMankle,waMwo,woMankle);
  
  woZfoot = -0.105*MAL_S4x4_MATRIX_ACCESS_I_J(woMankle,2,2) + MAL_S4x4_MATRIX_ACCESS_I_J(woMankle,2,3);
  waZfoot = -0.105*MAL_S4x4_MATRIX_ACCESS_I_J(waMankle,2,2) + MAL_S4x4_MATRIX_ACCESS_I_J(waMankle,2,3);
  
  MAL_S4_VECTOR(woZMP, double);
  MAL_S4_VECTOR(waZMP, double);
  
  woZMP(0) = ZMPval(0); woZMP(1)=ZMPval(1); woZMP(2)=woZfoot; woZMP(3)=1;
  waZMP = waMwo * woZMP;
  
  RebuildZMPW << ZMPval(0) << " " << ZMPval(1) << " 0. " 
	      << COMval(0) << " " << COMval(1) << " " << COMval(2) << " " 
    	      << robotState.aCurrentConf(0) << " " 
	      << robotState.aCurrentConf(1) << " " 
	      << robotState.aCurrentConf(2) << " " 
	      << robotState.aCurrentSpeed(0) << " " 
	      << robotState.aCurrentSpeed(1) << " " 
	      << robotState.aCurrentSpeed(2) << " " 
	      << robotState.aCurrentAcceleration(0) << " " 
	      << robotState.aCurrentAcceleration(1) << " " 
	      << robotState.aCurrentAcceleration(2) << " " 
	      << endl; // wrt World TMOULARD
  
  //RebuildZMP << waZMP(0) << " " << waZMP(1) << " 0." << endl; // wrt Waist TMOULARD
  RebuildZMP << ActualData[0] << " " << waZMP(0) << " " << waZMP(1) << " " << waZMP(2) << endl; // wrt Waist ORAMOS
  
  RebuildZMP.close();
  RebuildZMPW.close();
}

int main(int argc, char *argv[])
{
  // Handle arguments
  ConfigurationFiles_t aSetOfConfigurationFiles;
  dealWithArguments(argc, argv, aSetOfConfigurationFiles);

  // Build the two robots.
  dynamicsJRLJapan::ObjectFactory dynFactory;
  CjrlHumanoidDynamicRobot * aHDR  = 0, * aHDR2= 0;

  createRobots(aSetOfConfigurationFiles,aHDR,aHDR2);

  // Set their specific properties.
  // HDR2 is used to compute the geometrical relation
  // between feet and waist.
  // HDR does the dynamic computation.
  initializeRobots(aHDR,aHDR2);

  // Initialize Robot State.
  RobotState_t robotState;

  int NbOfDofs = aHDR->numberDof();
  
  robotState.setNbDofs(NbOfDofs);

  std::cout << "NbOfDofs :" << NbOfDofs << std::endl;
  robotState.reset();

  aHDR->currentConfiguration(robotState.aCurrentConf);
  aHDR2->currentConfiguration(robotState.aCurrentConf);

  // Added here (oscar)
  //CjrlJoint * Waist1 = aHDR->waist();


  // Read the data file.
  ifstream PosFile;
  PosFile.open(aSetOfConfigurationFiles.PosFileName.c_str(),
	       ifstream::in);
  if (!PosFile.is_open())
    {
      cerr << "Unable to open position file: " <<
	PosFile << endl;
      exit(-1);
    }

  ofstream RebuildZMP;
  RebuildZMP.open("RebuildZMP.dat",ofstream::out);
  RebuildZMP.close();

  ofstream RebuildZMPW;
  RebuildZMPW.open("RebuildZMPW.dat",ofstream::out);
  RebuildZMPW.close();

  ofstream RebuildForces;
  RebuildForces.open("RebuildForces.dat",ofstream::out);
  RebuildForces.close();

  ofstream RebuildWaist;
  RebuildWaist.open("RebuildWaist.dat",ofstream::out);
  RebuildWaist.close();


  double SamplingPeriod = 0.005;

  unsigned long int NbIt=0;
  vector<double> WaistFromRef,WaistFromActual;
  WaistFromRef.resize(6);
  WaistFromActual.resize(6);
  for(unsigned int i=0;i<6;i++)
    {WaistFromRef[i] = WaistFromActual[i] = 0.0;}
  
  matrix4d AbsSupportFootPos;
  // We set the first support foot as being the left one.
  int PreviousSupportFoot=-1;
  vector<double> ActualData;
  ActualData.resize(41);
  
  while(!PosFile.eof())
    {
      double RotationFreeFlyer[9];

      for(unsigned int i=0;i<41;i++)
	{
	  PosFile >> ActualData[i];
	}

      // Uses new data with free flyer to the origin
      robotState.updateConfFromFreeFlyer(WaistFromRef);      
      robotState.updateConfFromActualData(ActualData);

      // To update the position of WaistFromActual 
      // in the world reference frame
      robotState.updateRobot(aHDR2);
      aHDR2->computeForwardKinematics();
      ExtractActualWaist(aHDR2,
			 AbsSupportFootPos,
			 WaistFromRef,
			 RotationFreeFlyer,
			 NbIt,
			 WaistFromActual,
			 PreviousSupportFoot);
      SaveWaistPositions(WaistFromRef, WaistFromActual,PreviousSupportFoot);

      // Update robot state with the new waist position.
      robotState.updateConfFromFreeFlyer(WaistFromActual);
      robotState.updateCurrentRobotState(SamplingPeriod,NbIt);

      // and then update HDR and do dynamical computation
      robotState.updateRobot(aHDR);
      aHDR->computeForwardKinematics();

      // Save ZMP from HDR in two files with world or waist coordinates.
      saveZMP(aHDR,aHDR2,robotState,ActualData);

      robotState.aPrevCurrentConf = robotState.aCurrentConf;
      robotState.aPrevCurrentSpeed = robotState.aCurrentSpeed;

      NbIt++;
    }

  PosFile.close();
  delete aHDR;

}
