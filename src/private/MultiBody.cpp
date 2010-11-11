/*
 * Copyright 2010,
 *
 * Adrien Escande,
 * Oussama Kanoun
 * Francois Keith
 * Abderrahmane Kheddar,
 * Florent Lamiraux
 * Olivier Stasse,
 * Ramzi Sellouati
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

/* Multibody object used to compute :
   - Center Of Mass,
   - Zero Momentum Point.

   OS: Updated the names of the contributors, the documentation
   and added a sample file for WalkPlugin
   OS (21/12/2006): removed any reference to non-homogeneous matrix
   library.
   OS (10/01/2007): Put the abstract layer for small matrix library.


   JRL-Japan, CNRS/AIST

   Please refers to file License.txt for details on the license.

*/
#include <map>

#include "Debug.h"
#include "MultiBody.h"
#include "SpiritVRMLReader.h"
#include "JointRotationPrivate.h"
#include "JointTranslationPrivate.h"
#include "JointFreeFlyerPrivate.h"

using namespace dynamicsJRLJapan;

// surcharge operateur
bool dynamicsJRLJapan::operator==(const dynamicsJRLJapan::matching a1,
				  const dynamicsJRLJapan::matching a2)
{
  return (a1.body==a2.body);
}



// Caution: This operator is specific to OpenGL matrices: it transposes the
// matrix before multiplication.
MAL_S3_VECTOR_TYPE(double) operator * (double* m, MAL_S3_VECTOR_TYPE(double) v)
{
  MAL_S3_VECTOR_TYPE(double) result;

  result[0] = m[0]*v[0] + m[4]*v[1] +  m[8]*v[2]+ m[12];
  result[1] = m[1]*v[0] + m[5]*v[1] +  m[9]*v[2]+ m[13];
  result[2] = m[2]*v[0] + m[6]*v[1] + m[10]*v[2]+ m[14];
  return result;
}


void Matrix2AxisAngle(float R[16],double Axis[3], double & Angle)
{
  double q[4];
  double sum_x, sum_y, sum_z, sum_w, sum_max, S;

  sum_w = 1 + R[0] + R[5] + R[10];
  sum_x = 1 + R[0] - R[5] - R[10];
  sum_y = 1 + R[5] - R[0] - R[10];
  sum_z = 1 + R[10] - R[0] - R[5];
  sum_max = max(max(sum_w,sum_x), max(sum_y, sum_z));

  if (sum_max == sum_w)
    {
      S = sqrt(sum_w)*2;
      q[0] = (R[9] - R[6])/S;
      q[1] = (R[2] - R[8])/S;
      q[2] = (R[4] - R[1])/S;
      q[3] = 0.25*S;
    }
   else if (sum_max == sum_x)
    {
      S  = sqrt(sum_x) * 2;
      q[0] = 0.25 * S;
      q[1] = (R[4] + R[1] ) / S;
      q[2] = (R[2] + R[8] ) / S;
      q[3] = (R[9] - R[6] ) / S;
    }
  else if (sum_max == sum_y)
    {
      S  = sqrt(sum_y) * 2;
      q[0] = (R[4] + R[1] ) / S;
      q[1] = 0.25 * S;
      q[2] = (R[9] + R[6] ) / S;
      q[3] = (R[2] - R[8] ) / S;
    }
  else
    {
      S  = sqrt(sum_z) * 2;
      q[0] = (R[2] + R[8] ) / S;
      q[1] = (R[9] + R[6] ) / S;
      q[2] = 0.25 * S;
      q[3] = (R[4] - R[1] ) / S;
    }

  // Conversion from quaternions to axis and angles
  double sum;
  double cos_a,  sin_a;

  // Normalize
  sum = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];
  sum = sqrt(sum);

  if (sum==0)
    {
      Angle = Axis[0] = Axis[1] = Axis[2] = 0.0;
      return;
    }

  for(int i=0;i<4;i++)
    q[i] /= sum;

  cos_a = q[3];
  Angle = acos( cos_a ) * 2;
  sin_a = sqrt( 1.0 - cos_a * cos_a );
  if ( fabs( sin_a ) < 0.0005 )
    sin_a = 1;

  Axis[0] = -q[0] / sin_a;
  Axis[1] = -q[1] / sin_a;
  Axis[2] = -q[2] / sin_a;
}



MultiBody::MultiBody(void)
{
    m_mass = 0.0;
}


MultiBody::~MultiBody(void)
{
  for(unsigned int li=0;li<listInternalLinks.size();li++)
    delete listInternalLinks[li].aJoint;

  for(unsigned int li=0;li<listBodies.size();li++)
    delete listBodies[li];
}

void MultiBody::addBody(Body &b)
{
  m_mass += b.getMass();
  listBodies.push_back(&b);
  links.push_back(vector<matching>());
}

Body * MultiBody::lastBody()
{
  return listBodies[listBodies.size()-1];
}

void MultiBody::addLink(Body &corps1, Body &corps2, internalLink & l)
{
  // search for index of first body in listBodies
  unsigned int index1;
  for (index1 = 0; index1<listBodies.size(); index1++) {
    if (corps1.getLabel() == listBodies[index1]->getLabel()) {
      break;
    }
  }

  // search for index of second body in listBodies
  unsigned int index2;
  for (index2 = 0; index2<listBodies.size(); index2++) {
    if (corps2.getLabel() == listBodies[index2]->getLabel()) {
      break;
    }
  }

  l.indexCorps1 = index1;
  l.indexCorps2 = index2;
  listInternalLinks.push_back(l);	// add link in the list

  // create relation between  corps2, liaison
  matching a2 = {index2, listInternalLinks.size()-1};
  links[index1].push_back(a2);
  // create relation between  corps1, liaison
  matching a1 = {index1, listInternalLinks.size()-1};
  links[index2].push_back(a1);
}

void MultiBody::deleteLinkBetween(Body &corps1, Body &corps2)
{
  // search index for the first body in listBodies
  unsigned int c1;
  for (c1 = 0; c1<listBodies.size(); c1++) {
    if (corps1.getLabel() == listBodies[c1]->getLabel()) {
      break;
    }
  }

  // search index for the second body in listBodies
  unsigned int c2;
  for (c2 = 0; c2<listBodies.size(); c2++) {
    if (corps2.getLabel() == listBodies[c2]->getLabel()) {
      break;
    }
  }

  matching a1 = {c2, 0};
  matching a2 = {c1, 0};
  // search the iteratory of the link among links of body c1.
  vector<matching>::iterator it = find(links[c1].begin(), links[c1].end(), a1);
  int index = it->link;
  listInternalLinks[index].indexCorps1 = -1;
  listInternalLinks[index].indexCorps2 = -1;
  listInternalLinks[index].label = -1;
  links[c1].erase(it);
  // search the iteratory of the link among links of body c2.
  it = find(links[c2].begin(), links[c2].end(), a2);
  links[c2].erase(it);
}

void MultiBody::removeLink(int index)
{
  int c1 = listInternalLinks[index].indexCorps1;
  int c2 = listInternalLinks[index].indexCorps2;
  matching a1 = {c2, 0};
  matching a2 = {c1, 0};
  //recherche de l'iterateur de la liaison parmis les links de corps1
  vector<matching>::iterator it = find(links[c1].begin(), links[c1].end(), a1);
  listInternalLinks[index].indexCorps1 = -1;
  listInternalLinks[index].indexCorps2 = -1;
  listInternalLinks[index].label = -1;
  links[c1].erase(it);		//et on l'enleve
  //recherche de l'iterateur de la liaison parmis les links de corps2
  it = find(links[c2].begin(), links[c2].end(), a2);
  links[c2].erase(it);		//et on l'enleve
}

void MultiBody::removeLinkLabel(int label)
{
  //recherche de l'index de la liaison
  unsigned int i;
  for (i=0; i<listInternalLinks.size(); i++) {
    if (listInternalLinks[i].label == label)
      break;
  }
  if (i==listInternalLinks.size())
    return;

  removeLink(i);
}

void MultiBody::removeBody(Body &b)
{
  unsigned int i;
  for (i=0; i<listBodies.size(); i++) {
    if (listBodies[i]->getLabel() == b.getLabel())
      break;
  }
  if (i==listBodies.size())
    return;

  removeBody(i);
}
void MultiBody::removeBody(int index)
{
  int j = 0;
  for (unsigned int i=0; i<links[index].size(); j++) {
    removeLink(links[index][i].link);
  }
  listBodies[index]->setLabel(-1);

}
void MultiBody::removeBodyLabel(int label)
{
  unsigned int i;
  for (i=0; i<listBodies.size(); i++) {
    if (listBodies[i]->getLabel() == label)
      break;
  }
  if (i==listBodies.size())
    return;

  removeBody(i);
}


MAL_S3_VECTOR_TYPE(double) MultiBody::getPositionCoM(void)
{
  return (positionCoMPondere/m_mass);
}

void MultiBody::display()
{
  displayBodies();
  displayLinks();
}

void MultiBody::displayBodies()
{
  for (unsigned int i=0; i<listBodies.size(); i++) {
    cout << "corps "<< i << " : \n";
    listBodies[i]->Display(cout);
    for (unsigned int j=0; j<links[i].size(); j++) {
      cout << "    lie a corps " << links[i][j].body << " par liaison "
	   << links[i][j].link << " (label " << listInternalLinks[links[i][j].link].label <<")\n";
    }
    cout << "\n";
  }
  cout << "\n";
}

void MultiBody::displayLinks(void) {
  for (unsigned int i=0; i<listInternalLinks.size(); i++) {
    cout << "Name: "<< listInternalLinks[i].aJoint->getName()
	 << " JointID in VRML "
	 << listInternalLinks[i].aJoint->getIDinActuated() << " " ;
    cout << "Link type:  " << listInternalLinks[i].aJoint->type()
	 << "  label "<< listInternalLinks[i].label
	 << "  bouding body "
	 << listInternalLinks[i].indexCorps1
	 << " to body  " << listInternalLinks[i].indexCorps2 << "\n";
    cout << "translationStatique : " << endl;
    MAL_S3_VECTOR_TYPE(double) aStaticTranslation;
    listInternalLinks[i].aJoint->getStaticTranslation(aStaticTranslation);
    cout << aStaticTranslation << endl;
    if (listInternalLinks[i].aJoint->type() > 0) {
      cout << "    axis : " << endl;
      cout << listInternalLinks[i].aJoint->axis();
      cout << endl;
    }
  }
  cout << "\n";
}

void dynamicsJRLJapan::AxisAngle2Matrix(const vector3d &AnAxis, double aQuantity, matrix3d &R)
{
  double c = cos(aQuantity);
  if (fabs(c)<1e-8)
    c=0.0;
  double s = sin(aQuantity);
  if (fabs(s)<1e-8)
    s=0.0;

  const double v = 1.0-c;
  const double xv  = AnAxis[0]*AnAxis[0]*v;
  const double yv  = AnAxis[1]*AnAxis[1]*v;
  const double zv  = AnAxis[2]*AnAxis[2]*v;
  const double xyv = AnAxis[0]*AnAxis[1]*v;
  const double yzv = AnAxis[1]*AnAxis[2]*v;
  const double zxv = AnAxis[2]*AnAxis[0]*v;
  const double xs  = AnAxis[0]*s;
  const double ys  = AnAxis[1]*s;
  const double zs  = AnAxis[2]*s;
  MAL_S3x3_MATRIX_ACCESS_I_J(R,0,0) = xv+c;
  MAL_S3x3_MATRIX_ACCESS_I_J(R,0,1) = xyv - zs;
  MAL_S3x3_MATRIX_ACCESS_I_J(R,0,2) = zxv + ys;
  MAL_S3x3_MATRIX_ACCESS_I_J(R,1,0) = xyv + zs;
  MAL_S3x3_MATRIX_ACCESS_I_J(R,1,1) = yv + c;
  MAL_S3x3_MATRIX_ACCESS_I_J(R,1,2) = yzv - xs;
  MAL_S3x3_MATRIX_ACCESS_I_J(R,2,0) = zxv - ys;
  MAL_S3x3_MATRIX_ACCESS_I_J(R,2,1) = yzv + xs;
  MAL_S3x3_MATRIX_ACCESS_I_J(R,2,2) = zv + c;
}

int MultiBody::NbOfLinks() const
{
  return listInternalLinks.size();
}

int MultiBody::NbOfJoints() const
{
  return listInternalLinks.size();
}


MultiBody & MultiBody::operator=(const MultiBody &rhs)
{

  listInternalLinks.clear();
  links.clear();
  listBodies.clear();
  std::map<DynamicBodyPrivate *,DynamicBodyPrivate *> MapBodiesFromOriginalToNew;
  std::map<JointPrivate *, JointPrivate *>
    MapJointsFromOriginalToNew;

  internalLink CurrentLink ;

  for(unsigned int i=0;
      i<rhs.listBodies.size();
      i++)
    {
      addBody(*rhs.listBodies[i]);
      ODEBUG("Body:" << *listBodies[i]);
      DynamicBodyPrivate *Origin = (DynamicBodyPrivate *)(rhs.listBodies[i]);
      DynamicBodyPrivate *Destination = (DynamicBodyPrivate *)(listBodies[i]);
      if ((Origin!=0) && (Destination!=0))
	MapBodiesFromOriginalToNew[Origin] = Destination;
      else
	{
	  exit(-1);
	}
    }

  for(unsigned int i=0;
      i<rhs.listInternalLinks.size();
      i++)
    {

      if (rhs.listInternalLinks[i].aJoint->type()==JointPrivate::REVOLUTE_JOINT)
	{
	  JointRotationPrivate * aJRP = dynamic_cast<JointRotationPrivate*>(rhs.listInternalLinks[i].aJoint);
	  CurrentLink.aJoint = new JointRotationPrivate(*aJRP);
	}
      else if (rhs.listInternalLinks[i].aJoint->type()==JointPrivate::FREE_JOINT)
	{
	  JointFreeflyerPrivate * aJFP = dynamic_cast<JointFreeflyerPrivate*>(rhs.listInternalLinks[i].aJoint);
	  CurrentLink.aJoint = new JointFreeflyerPrivate(*aJFP);
	}
      else if (rhs.listInternalLinks[i].aJoint->type()==JointPrivate::PRISMATIC_JOINT)
	{
	  JointTranslationPrivate * aJTP = dynamic_cast<JointTranslationPrivate *>(rhs.listInternalLinks[i].aJoint);
	  CurrentLink.aJoint = new JointTranslationPrivate(*aJTP);
	}
      ODEBUG("=================================");
      ODEBUG("Original Joint " << endl
	     << *rhs.listInternalLinks[i].aJoint );
      ODEBUG( "Copied Joint "<< endl
	      << *CurrentLink.aJoint );
      ODEBUG("=================================");
      addLink(*listBodies[rhs.listInternalLinks[i].indexCorps1],
	      *listBodies[rhs.listInternalLinks[i].indexCorps2],
	      CurrentLink);

      MapJointsFromOriginalToNew[rhs.listInternalLinks[i].aJoint] = CurrentLink.aJoint;

    }

  ODEBUG("Now bound objects together.");
  for(unsigned int i=0;
      i<rhs.listInternalLinks.size();
      i++)
  {
    // Now bound objects together.
    DynamicBodyPrivate * OriginalBody = rhs.listInternalLinks[i].aJoint->linkedDBody();
    ODEBUG(" i:"<<i);
    CurrentLink.aJoint->setLinkedDBody(MapBodiesFromOriginalToNew[OriginalBody]);
    ODEBUG(" Set linkedDBody");
    ODEBUG("rhs.listInternalLinks[i].aJoint:" << rhs.listInternalLinks[i].aJoint
	    << " " << MapJointsFromOriginalToNew.size());
    JointPrivate *OriginalFatherJoint = 0;
    JointPrivate *FatherJointInNewTree = 0;
    if (rhs.listInternalLinks[i].aJoint!=0)
      {
	OriginalFatherJoint = (JointPrivate *)rhs.listInternalLinks[i].aJoint->parentJoint();
	if (OriginalFatherJoint!=0)
	  {
	    FatherJointInNewTree = MapJointsFromOriginalToNew[OriginalFatherJoint];
	  }
      }

    ODEBUG("Found father in new tree.");
    CurrentLink.aJoint->SetFatherJoint(FatherJointInNewTree);
    ODEBUG(" Set FatherJoint.");

  }

  m_mass = rhs.m_mass;

  ODEBUG("Finished bindings.");
  return *this;
}

