#include <string>
#include <iostream>
#include <fstream>

#include <jrl/dynamics/dynamicsfactory.hh>
#include "DynMultiBodyPrivate.h"


#include <jrl/dynamics/metapodModelMaker.hh>

#define BOOST_TEST_MODULE MULTIBODY_CONVERTER

#include <boost/test/unit_test.hpp>
#include <boost/test/output_test_stream.hpp>

using namespace std;
using namespace dynamicsJRLJapan;

int main()
{
  MultibodyConverter::Converter mb;

  string aSpecificitiesFileName;
  string aPath;
  string aName;
  string aMapFromJointToRank;

  // Specify the name the robot class will be given
  std::string robotName = "simple_humanoid";

  // Specify wrml model files
  aPath="./";
  aName="sample.wrl";
  aSpecificitiesFileName = "sampleSpecificities.xml";
  aMapFromJointToRank = "sampleLinkJointRank.xml";

  // Generate .cc and .hh files
  mb.generateCCFile(aSpecificitiesFileName,
                    aPath,
                    aName,
                    aMapFromJointToRank,
                    robotName);
}
