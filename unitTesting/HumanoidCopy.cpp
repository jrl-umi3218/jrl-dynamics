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
/* @doc \file Object to copy two humanoid robots 
   through the abstract interface. */

#include "HumanoidCopy.h"

using namespace dynamicsJRLJapan;

HumanoidCopy::HumanoidCopy()
{
}

HumanoidCopy::~HumanoidCopy()
{
}

void HumanoidCopy::CopyAndInstanciateBody(CjrlJoint *initJoint, 
					  CjrlJoint *newJoint)
{
  CjrlBody * CopiedBody =newJoint->linkedBody();
  CjrlBody * OriginalBody = initJoint->linkedBody();
  
  // Instanciate Copied Body if needed.
  if (CopiedBody==0)
    {
      CopiedBody= robotDynamicsObjectConstructor.createBody();
      newJoint->setLinkedBody(*CopiedBody);
    }

  // Copying the mass.
  double origmass= OriginalBody->mass();
  CopiedBody->mass(origmass);
  
  // Copying the inertia matrix.
  const matrix3d anInertiaMatrix = OriginalBody->inertiaMatrix();
  CopiedBody->inertiaMatrix(anInertiaMatrix);
  
  // Copying the local center of mass.
  const vector3d aLocalCenterOfMass = OriginalBody->localCenterOfMass();
  CopiedBody->localCenterOfMass(aLocalCenterOfMass);
	  
}

void HumanoidCopy::CopyLocalFieldsOfJoints(CjrlJoint *initJoint, CjrlJoint *newJoint)
{
  if ((initJoint==0) || (newJoint==0))
    return;

  for (unsigned int i=0;
       i<initJoint->numberDof();
       i++)
    {
      newJoint->lowerBound(i,initJoint->lowerBound(i));
      newJoint->upperBound(i,initJoint->upperBound(i));

      newJoint->lowerVelocityBound(i,initJoint->lowerVelocityBound(i));
      newJoint->upperVelocityBound(i,initJoint->upperVelocityBound(i));
    }
}

void HumanoidCopy::CopyFoot(CjrlFoot *InitFoot,
			    CjrlFoot * &NewFoot)
{

  const CjrlJoint * anAnkle = m_JointsMap[(CjrlJoint *)InitFoot->associatedAnkle()];
  NewFoot = robotDynamicsObjectConstructor.createFoot(anAnkle);

  double outLength, outWidth;
  InitFoot->getSoleSize(outLength,outWidth);
  NewFoot->setSoleSize(outLength,outWidth);
  
  vector3d data;
  InitFoot->getAnklePositionInLocalFrame(data);
  NewFoot->setAnklePositionInLocalFrame(data);  
}

void HumanoidCopy::CopyHand(CjrlHand *InitHand,
			    CjrlHand *&NewHand)
{
 
  const CjrlJoint *aWrist = m_JointsMap[(CjrlJoint *)InitHand->associatedWrist()];
  NewHand = robotDynamicsObjectConstructor.createHand(aWrist);

  vector3d data;
  InitHand->getCenter(data); NewHand->setCenter(data);
  InitHand->getThumbAxis(data); NewHand->setThumbAxis(data);
  InitHand->getForeFingerAxis(data); NewHand->setForeFingerAxis(data);
  InitHand->getPalmNormal(data); NewHand->setPalmNormal(data); 
}

void HumanoidCopy::CopySemantic(CjrlHumanoidDynamicRobot* aHDR,
				CjrlHumanoidDynamicRobot* a2HDR)
{
  // Chest.
  CjrlJoint * InitData = aHDR->chest();
  CjrlJoint * NewData = m_JointsMap[InitData];
  a2HDR->chest(NewData);

  // Waist
  InitData = aHDR->waist();
  NewData = m_JointsMap[InitData];
  a2HDR->waist(NewData);
  
  // Left Wrist
  InitData = aHDR->leftWrist();
  NewData = m_JointsMap[InitData];
  a2HDR->leftWrist(NewData);

  // Right wrist
  InitData = aHDR->rightWrist();
  NewData = m_JointsMap[InitData];
  a2HDR->rightWrist(NewData);

  // Left Ankle
  InitData = aHDR->leftAnkle();
  NewData = m_JointsMap[InitData];
  a2HDR->leftAnkle(NewData);

  // Right Ankle
  InitData = aHDR->rightAnkle();
  NewData = m_JointsMap[InitData];
  a2HDR->rightAnkle(NewData);
  
  // Gaze
  InitData = aHDR->gazeJoint();
  NewData = m_JointsMap[InitData];
  a2HDR->gazeJoint(NewData);


}

void HumanoidCopy::CopyEndEffectors(CjrlHumanoidDynamicRobot* aHDR,
				    CjrlHumanoidDynamicRobot* a2HDR)
{
  CjrlFoot *NewFoot=0;
  CopyFoot(aHDR->rightFoot(), NewFoot);
  a2HDR->rightFoot(NewFoot);
  
  CopyFoot(aHDR->leftFoot(), NewFoot);
  a2HDR->leftFoot(NewFoot);

  CjrlHand *NewHand=0;
  CopyHand(aHDR->rightHand(), NewHand);
  a2HDR->rightHand(NewHand);
  
  CopyHand(aHDR->leftHand(), NewHand);
  a2HDR->leftHand(NewHand);
}

void HumanoidCopy::CopyActuatedJoints(CjrlHumanoidDynamicRobot* aHDR,
				      CjrlHumanoidDynamicRobot* a2HDR)
{
  std::vector<CjrlJoint *> InitActuatedJoints = aHDR->getActuatedJoints();
  std::vector<CjrlJoint *> NewActuatedJoints;
  NewActuatedJoints.resize(InitActuatedJoints.size());
  for(unsigned int i=0;i<InitActuatedJoints.size();i++)
    {
      NewActuatedJoints[i] = m_JointsMap[InitActuatedJoints[i]];
    }
  aHDR->setActuatedJoints(NewActuatedJoints);
}


void HumanoidCopy::recursiveMultibodyCopy(CjrlJoint *initJoint, CjrlJoint *newJoint)
{
  int lNbOfChildren= initJoint->countChildJoints();

  // stop test
  if (lNbOfChildren == 0) return ;

  for(int li=0;li<lNbOfChildren;li++) {
    CjrlJoint *Child = initJoint->childJoint(li) ;

    if (Child != 0) {
      
      CjrlJoint* a2newJoint=0;
      const matrix4d pose = Child->initialPosition();
      a2newJoint = robotDynamicsObjectConstructor.createJointRotation(pose);

      m_JointsMap[Child] = a2newJoint;
      CopyLocalFieldsOfJoints(Child,a2newJoint);

      newJoint->addChildJoint(*a2newJoint);
      CopyAndInstanciateBody(Child, a2newJoint);

      recursiveMultibodyCopy(Child, a2newJoint) ;
    }
  }
}


void HumanoidCopy::PerformCopyFromJointsTree(CjrlHumanoidDynamicRobot* aHDR,
					     CjrlHumanoidDynamicRobot* a2HDR)
{
  m_JointsMap.clear();

  CjrlJoint* InitJoint = aHDR->rootJoint() ;
  
  CjrlJoint* newJoint=0;
  matrix4d pose=InitJoint->initialPosition();
  newJoint = robotDynamicsObjectConstructor.createJointFreeflyer(pose);
  m_JointsMap[InitJoint] = newJoint;
  CopyLocalFieldsOfJoints(InitJoint, newJoint);

  CopyAndInstanciateBody(InitJoint,newJoint);
  a2HDR->rootJoint(*newJoint);
  
  recursiveMultibodyCopy(InitJoint, newJoint) ;

  // Copy hands and feet.
  CopyEndEffectors(aHDR,a2HDR);

  CopySemantic(aHDR,a2HDR);
  
  a2HDR->initialize();
  // Copy the bodies.
  std::vector<CjrlJoint *> VecOfInitJoints = aHDR->jointVector();
  std::vector<CjrlJoint *> VecOfCopyJoints = a2HDR->jointVector();
  
  if (VecOfInitJoints.size()!=VecOfCopyJoints.size())
    {
      std::cout << "Problem while copying the joints. size : "<< VecOfInitJoints.size() 
		<< " size copy : " << VecOfCopyJoints.size()  << std::endl;
      std::cout << std::endl << std::endl << "There is a probleme the new joints vector is not updated" << std::endl << std::endl ;
      exit(-1);
    }
}

