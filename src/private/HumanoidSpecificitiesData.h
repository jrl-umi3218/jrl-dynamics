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
    typedef struct
    {
      unsigned int nbOfJoints;
      std::vector<unsigned int> jointID;
    } SerialChain;

    
    /*! Foot description */
    typedef struct 
    {
      // Foot size 
      double sizeX;
      double sizeY;
      double sizeZ;

      // Ankle Position
      std::vector<double> anklePosition;
      
      // Serial Chain specifying the foot.
      SerialChain sChain;

    } FootNode;


    /*! Waist description:
     Connection from waist to legs.*/
    typedef struct
    {
      // Position to left hip.
      std::vector<double> leftWaistToHip;
      std::vector<double> rightWaistToHip;

      SerialChain sChain;
    } WaistNode;

    /*! Legs description */
    typedef struct
    {
      // Hip length
      std::vector<double> hipLength;

      double femurLength;
      double tibiaLength;

      SerialChain sChain;
    } LegNode;
    
    /*! Hand description */
    typedef struct
    {
      std::vector<double> center;
      std::vector<double> okayAxis;
      std::vector<double> showingAxis;
      std::vector<double> palmAxis;
      
    } HandNode;
    /*! Arms description */
    typedef struct
    {
      double upperArmLength;
      double foreArmLength;

      SerialChain sChain;
    } ArmNode;

    typedef struct
    {
      std::string Name;

      FootNode rightFoot;
      FootNode leftFoot;

      WaistNode waist;

      LegNode rightLeg;
      LegNode leftLeg;

      HandNode rightHand;
      HandNode leftHand;

      SerialChain head;
      SerialChain chest;
    } HumanoidNode;

  };
};

namespace dhs=dynamicsJRLJapan::HumanoidSpecificitiesData;

// We need to tell fusion about our mini_xml struct
    // to make it a first-class fusion citizen
    BOOST_FUSION_ADAPT_STRUCT(
			      dhs::SerialChain,
			      (unsigned int, nbOfJoints)
			      (std::vector<unsigned int>, jointID)
			      )

    BOOST_FUSION_ADAPT_STRUCT(
			      dhs::FootNode,
			      (double, sizeX)
			      (double, sizeY)
			      (double, sizeZ)
			      (std::vector<double>, anklePosition)
			      (dhs::SerialChain, sChain)
			      )

    BOOST_FUSION_ADAPT_STRUCT(dhs::WaistNode,
			      (std::vector<double>, leftWaistToHip)
			      (std::vector<double>, rightWaistToHip)
			      (dhs::SerialChain, sChain)
			      )
    BOOST_FUSION_ADAPT_STRUCT(dhs::LegNode,
			      (std::vector<double>,hipLength)
			      (double, femurLength)
			      (double, tibiaLength)
			      (dhs::SerialChain, sChain)
			      )
    BOOST_FUSION_ADAPT_STRUCT(dhs::HandNode,
			      (std::vector<double>, okayAxis)
			      (std::vector<double>, showingAxis)
			      (std::vector<double>, palmAxis)
			      )
    BOOST_FUSION_ADAPT_STRUCT(dhs::ArmNode,
			      (double, upperArmLength)
			      (double, foreArmLength)
			      (dhs::SerialChain, sChain)
			      )
    BOOST_FUSION_ADAPT_STRUCT(dhs::HumanoidNode,
			      (std::string, Name)
			      (dhs::FootNode, rightFoot)
			      (dhs::FootNode, leftFoot)
			      (dhs::WaistNode, waist)
			      (dhs::LegNode, rightLeg)
			      (dhs::LegNode, leftLeg)
			      (dhs::HandNode, rightHand)
			      (dhs::HandNode, leftHand)
			      (dhs::SerialChain, head)
			      (dhs::SerialChain, chest)
			      )

#endif
