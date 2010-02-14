/* @doc Data structures used to store Specificities Information

   Copyright (c) 2010, 

   @author : 
   Olivier Stasse
   
   JRL-Japan, CNRS/AIST

   All rights reserved.
   
   Please refers to file License.txt for details on the license.

*/

#ifndef _HUMANOID_SPECIFICITIES_DATA_H_
#define _HUMANOID_SPECIFICITIES_DATA_H_

/*! Framework specfic includes */
#include <Debug.h>

/*! Boost specific includes */
#include <boost/config/warning_disable.hpp>
#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/include/phoenix_core.hpp>
#include <boost/spirit/include/phoenix_operator.hpp>
#include <boost/spirit/include/phoenix_fusion.hpp>
#include <boost/spirit/include/phoenix_stl.hpp>
#include <boost/fusion/include/adapt_struct.hpp>
#include <boost/variant/recursive_variant.hpp>
#include <boost/foreach.hpp>

/*! System Includes */
#include <iostream>
#include <fstream>
#include <string>
#include <vector>


namespace dynamicsJRLJapan {
  namespace HumanoidSpecificitiesData {

    namespace fusion = boost::fusion;
    namespace phoenix = boost::phoenix;
    namespace qi = boost::spirit::qi;
    namespace ascii = boost::spirit::ascii;

    /*! Articular serial chain */
    typedef struct s_SerialChain
    {
      
      unsigned int nbOfJoints;
      std::vector<unsigned int> jointID;
      
      friend std::ostream & operator<< (std::ostream &out, const struct s_SerialChain &sc);
    } SerialChain;
    
    std::ostream & operator<< (std::ostream &out, const struct s_SerialChain &sc)
      {
	out << "Nb Of Joints:" << sc.nbOfJoints << std::endl;
	for(unsigned int i=0;i<sc.jointID.size();i++)
	  out << sc.jointID[i] << " " ;
	out << std::endl;
	return out;
      }

    /*! Foot description */
    typedef struct s_FootNode
    {
      // Foot size 
      double sizeX;
      double sizeY;
      double sizeZ;

      // Ankle Position
      std::vector<double> anklePosition;
      
      // Serial Chain specifying the foot.
      SerialChain sChain;

      friend std::ostream  & operator<< (std::ostream &out, const struct s_FootNode &fn);
    } FootNode;

    std::ostream  & operator<< (std::ostream &out, const FootNode &fn)
      {
	out << "sizeX: " << fn.sizeX << std::endl;
	out << "sizeY: " << fn.sizeY << std::endl;
	out << "sizeZ: " << fn.sizeZ << std::endl;
	for(unsigned int i=0;i<fn.anklePosition.size();i++)
	  out <<fn.anklePosition[i]<< " ";
	out <<std::endl;
	
	out <<fn.sChain;
	return out;
      }

    /*! Waist description:
     Connection from waist to legs.*/
    typedef struct s_WaistNode
    {
      // Position to left hip.
      std::vector<double> leftWaistToHip;
      std::vector<double> rightWaistToHip;

      SerialChain sChain;

      friend std::ostream  & operator<< (std::ostream &out, const struct s_WaistNode &wn);
    } WaistNode;

    std::ostream  & operator<< (std::ostream &out, const WaistNode &wn)
      {
	for(unsigned int i=0;i<wn.leftWaistToHip.size();i++)
	  out << wn.leftWaistToHip[i] << " ";
	out << std::endl;

	for(unsigned int i=0;i<wn.rightWaistToHip.size();i++)
	  out << wn.rightWaistToHip[i] << " ";
	out << std::endl;
	return out;
      }

    /*! Legs description */
    typedef struct s_LegNode
    {
      // Hip length
      std::vector<double> hipLength;

      double femurLength;
      double tibiaLength;

      SerialChain sChain;

      friend std::ostream  & operator<< (std::ostream &out, const struct s_LegNode &wn);

    } LegNode;

    std::ostream  & operator<< (std::ostream &out, const LegNode &wn)
      {
	for(unsigned int i=0;i<wn.hipLength.size();i++)
	  out << wn.hipLength[i] << " ";
	out << std::endl;
	
	out << wn.femurLength << std::endl;
	out << wn.tibiaLength << std::endl;

	out << wn.sChain <<std::endl;
	return out;
      }    

    /*! Hand description */
    typedef struct s_HandNode
    {
      std::vector<double> center;
      std::vector<double> okayAxis;
      std::vector<double> showingAxis;
      std::vector<double> palmAxis;

      friend std::ostream  & operator<< (std::ostream &out, const struct s_HandNode &hn);
      
    } HandNode;

    std::ostream  & operator<< (std::ostream &out, const HandNode &hn)
      {
	for(unsigned int i=0;i<hn.center.size();i++)
	  out << "center: " << hn.center[i] << " ";
	out << std::endl;

	for(unsigned int i=0;i<hn.okayAxis.size();i++)
	  out << "okayAxis: " <<hn.okayAxis[i] << " ";
	out << std::endl;

	for(unsigned int i=0;i<hn.showingAxis.size();i++)
	  out << "showingAxis:" << hn.showingAxis[i] << " ";
	out << std::endl;

	for(unsigned int i=0;i<hn.palmAxis.size();i++)
	  out << "palmAxis:" << hn.palmAxis[i] << " ";
	out << std::endl;
	return out;
      }
    /*! Arms description */
    typedef struct s_ArmNode
    {
      double upperArmLength;
      double foreArmLength;

      SerialChain sChain;

      friend     std::ostream  & operator<< (std::ostream &out, const struct s_ArmNode &an);
    } ArmNode;

    std::ostream  & operator<< (std::ostream &out, const ArmNode &an)
      {
	out << "upperArmLength: " << an.upperArmLength << std::endl;
	out << "foreArmLength: " << an.foreArmLength << std::endl;
	out << an.sChain << std::endl;
	return out;
      }

    typedef struct s_HumanoidNode
    {
      std::string Name;

      FootNode rightFoot;
      FootNode leftFoot;

      WaistNode waist;

      LegNode rightLeg;
      LegNode leftLeg;

      HandNode rightHand;
      HandNode leftHand;

      ArmNode rightArm;
      ArmNode leftArm;

      SerialChain head;
      SerialChain chest;

      friend std::ostream  & operator<< (std::ostream &out, const struct s_HumanoidNode &humn);

    } HumanoidNode;
    
    std::ostream  & operator<< (std::ostream &out, const HumanoidNode &humn)
      {
	out << "Name:" << humn.Name <<std::endl;
	out << "rightFoot: "<< humn.rightFoot << std::endl;
	out << "leftFoot: "<< humn.leftFoot << std::endl;

	out << "Waist:" << humn.waist << std::endl;

	out << "rightLeg:" << humn.rightLeg << std::endl;
	out << "leftLeg:" << humn.leftLeg << std::endl;
	
	out << "rightHand:" << humn.rightHand << std::endl;
	out << "leftHand:" << humn.leftHand << std::endl;

	out << "rightArm:" << humn.rightArm << std::endl;
	out << "leftArm:" << humn.leftArm << std::endl;
	
	out << "Head: " <<humn.head << std::endl;
	out << "Chest: " <<humn.chest << std::endl;
	return out;
      }
  };
};


#endif
