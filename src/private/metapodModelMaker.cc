// Copyright (C) 2009 by Maxime REIS.
//
// This file is part of the multibody-converter.
//
// multibody-converter is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// multibody-converter is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with multibody-converter.  If not, see <http://www.gnu.org/licenses/>.


/**
 * \file src/multibody-converter.cc
 *
 * \brief Implementation of Converter.
 */

#include "jrl/dynamics/metapodModelMaker.hh"

//using namespace dynamicsJRLJapan;

namespace MultibodyConverter
{
  Converter::Converter ()
  {
  }

  Converter::~Converter ()
  {
  }

  // Returns the name of the joint associated with given body
  std::string Converter::getJoint(dynamicsJRLJapan::DynamicBodyPrivate *b)
  {
    return b->getJointPrivate()->getName();
  }

  // Returns the name of the body parent of given body, if there is one, or
  // "NP" otherwise
  std::string Converter::getParent(dynamicsJRLJapan::DynamicBodyPrivate *b)
  {
    if(b->getJointPrivate()->parentJoint())
    {
      dynamicsJRLJapan::DynamicBodyPrivate *p =
        (dynamicsJRLJapan::DynamicBodyPrivate*)b->getJointPrivate()->parentJoint()->linkedBody();
      return p->getName();
    }
    else
      return "NP";
  }

  // Adds a body class from a jrl-dynamics dynamic body in body.hh, and
  // initialize its variables in initialization.cc
  void Converter::addBody(dynamicsJRLJapan::DynamicBodyPrivate *b,
                          std::ofstream & body_hh,
                          std::ofstream & lib_cc)
  {
    stringstream ss;
    int label = b->getLabel()-1;
    int parentLabel = b->getLabelMother();
    int has_parent = parentLabel?1:0;
    double mass = b->getMass();
    vector3d CoM = b->localCenterOfMass();
    string name = b->getName();
    matrix3d inertie = b->getInertie();
    string jointName = getJoint(b);
    string parentName = getParent(b);

    body_hh
      << "    CREATE_BODY(" << name << ", " << has_parent << ", " << parentName
        << ", " << jointName << ");" << endl;
    lib_cc
      << "    // Initialization of " << name << ";" << endl
      << "    INITIALIZE_BODY(" << name << ")\n"
      << "    const std::string " << name << "::name = \"" << name << "\";" << endl
      << "    const int " << name << "::label = " << label << ";" << endl
      << "    const FloatType " << name << "::mass = " << mass << ";" << endl
      << "    const vector3d " << name << "::CoM = vector3d("
        << CoM[0] << ", " << CoM[1] << ", " << CoM[2] << ");" << endl
      << "    const matrix3d " << name << "::inertie = matrix3dMaker(" << endl
      << "      " << inertie(0,0) << ", " << inertie(0,1) << ", " << inertie(0,2)
        << "," << endl
      << "      " << inertie(1,0) << ", " << inertie(1,1) << ", " << inertie(1,2)
        << "," << endl
      << "      " << inertie(2,0) << ", " << inertie(2,1) << ", " << inertie(2,2)
        << ");" << endl
      << "    Spatial::Inertia " << name << "::I = spatialInertiaMaker("
        << name << "::mass,\n                                                         "
        << name << "::CoM,\n                                                         "
        << name << "::inertie);\n\n";
  }

  // Adds a node in the Tree typedef
  void Converter::addNode(int index,
                          std::vector< std::vector< int > > children,
                          std::vector< std::string > joints,
                          std::ofstream & robot_hh,
                          CjrlDynamicRobot * aHDR,
                          int depth)
  {
    dynamicsJRLJapan::DynMultiBodyPrivate * robot
      = dynamic_cast< dynamicsJRLJapan::DynMultiBodyPrivate* >(aHDR);

    std::stringstream ss_tab;
    if(index != 0)
    {
      ss_tab << "              ";
    }
    for(int i=0; i<depth; i++)
    {
      ss_tab << "      ";
    }
    std::string tab = ss_tab.str();

    if(children[index][0])
    {
      if(index == 0)
      {
        robot_hh
          << tab << "Node< " << robot->getListOfBodies()[index+1]->getName()
            << ",\n              ";
      }
      else
      {
        robot_hh
          << tab << "  Node< " << robot->getListOfBodies()[index+1]->getName()
            << "," << endl;
      }
      robot_hh
        << tab << "        "
          << robot->getListOfBodies()[index+1]->getJointPrivate()->getName()
          << "," << endl;
      int j=0;
      while(children[index][j])
      {
        depth++;
        addNode(children[index][j], children, joints, robot_hh, aHDR, depth);
        if(children[index][j+1])
          robot_hh << ",";
        robot_hh << endl;
        j++;
        depth--;
      }
      robot_hh << tab << "  >";
    }
    else
    {
      if(index == 1)
        robot_hh << "        ";
      robot_hh
        << tab << "  Node< " << robot->getListOfBodies()[index+1]->getName() << ","
        << robot->getListOfBodies()[index+1]->getJointPrivate()->getName() << " >" << endl;
    }
      
  }

  // Creates the Tree typename
  void Converter::makeTree(CjrlDynamicRobot * aHDR,
                           std::ofstream & robot_hh,
                           int nbBodies)
  {
    dynamicsJRLJapan::DynamicBodyPrivate *currentBody=0;
    dynamicsJRLJapan::DynMultiBodyPrivate * robot
      = dynamic_cast< dynamicsJRLJapan::DynMultiBodyPrivate* >(aHDR);
    std::vector< std::string > joints;
    std::vector< std::vector<int> > children;
    children.resize(nbBodies);

    for(int i=0; i<nbBodies; i++)
    {
      currentBody = robot->getListOfBodies()[i+1];
      joints.push_back(currentBody->getJointPrivate()->getName());
      int j = currentBody->getLabelMother() - 1;
      if(j!=-1)
        children[j].push_back(i);
    }
    for(int i=0; i<nbBodies; i++)
      children[i].push_back(0);

    int index = 0;

    robot_hh
      << endl
      << "        // Definition of the multibody tree as a type.\n"
      << "        typedef ";
    addNode(index, children, joints, robot_hh, aHDR);
    robot_hh << " Tree;" << endl
 << endl;
  }

  // Adds a joint class from a jrl-dynamics dynamic joint, in joint.hh, and
  // initialize its variables in initialization.cc
  void Converter::addJoint(dynamicsJRLJapan::DynamicBodyPrivate *b,
                           std::ofstream & joint_hh,
                           std::ofstream & lib_cc)
  {
    dynamicsJRLJapan::JointPrivate * j = b->getJointPrivate();
    dynamicsJRLJapan::Spatial::PluckerTransform Xt = j->getXL();
    matrix3d R = Xt.R();
    vector3d p = Xt.p();
    int rankInConf = j->rankInConfiguration();
    stringstream ss;
    int label = b->getLabel();
    ss << "j" << label;
    string joint = ss.str();
    int type = j->type();
    ss.str(std::string());
    string name = j->getName();

    switch(type)
    {
      case  1: // REVOLUTE_JOINT
        joint_hh
          << "    JOINT_REVOLUTE(" << name << ");" << endl;
        break;
      case -1: // FREE_FLYER
        joint_hh
          << "    JOINT_FREE_FLYER(" << name << ");" << endl;
    }
    lib_cc
      << "    // Init " << name << endl;
    switch(type)
    {
      case  1: // REVOLUTE_JOINT
        lib_cc << "    INITIALIZE_JOINT_REVOLUTE(" << name << ");\n";
        break;
      case -1: // FREE_FLYER
        lib_cc << "    INITIALIZE_JOINT_FREE_FLYER(" << name << ");\n";
        break;
    }
    lib_cc
      << "    const std::string " << name << "::name = \"" << name << "\";\n"
      << "    const int " << name << "::label = " << label << ";" << endl
      << "    const int " << name << "::positionInConf = " << rankInConf << ";\n"
      << "    const Spatial::Transform " << name << "::Xt = Spatial::Transform(" << endl
      << "      matrix3dMaker(" << R(0,0) << ", " << R(1,0) << ", " << R(2,0) << ",\n"
      << "                    " << R(0,1) << ", " << R(1,1) << ", " << R(2,1) << ",\n"
      << "                    " << R(0,2) << ", " << R(1,2) << ", " << R(2,2) << "),\n"
      << "      vector3d(" << p[0] << ", " << p[1] << ", " << p[2] << "));\n"
 << endl;
  }

  // Parse the jrl-dynamic dynamic robot model, and builds the joint and body
  // classes accordlingly, along with the Tree type modeling the robot
  void Converter::buildRobot(CjrlDynamicRobot * aHDR,
                             std::ofstream & joint_hh,
                             std::ofstream & body_hh,
                             std::ofstream & lib_cc)
  {
    int labelTheRoot = 1;
    dynamicsJRLJapan::DynamicBodyPrivate *currentBody=0;
    int currentNode = labelTheRoot;
    dynamicsJRLJapan::DynMultiBodyPrivate * robot
      = dynamic_cast< dynamicsJRLJapan::DynMultiBodyPrivate* >(aHDR);
  
    do
    {
      currentBody = robot->getListOfBodies()[currentNode];
      addJoint(currentBody, joint_hh, lib_cc);
      addBody(currentBody, body_hh, lib_cc);
      int step = 0;
      int NextNode = 0;
      do
      {
        switch(step)
        {
          case 0:
            NextNode = currentBody->child;
            step++;
            break;
          case 1:
            NextNode = currentBody->sister;
            step++;
            break;
          case 2:
            NextNode = currentBody->getLabelMother();
            if(NextNode >= 0)
            {
              currentNode = NextNode;
              currentBody = robot->getListOfBodies()[currentNode];
              NextNode = currentBody->sister;
            }
            else
              NextNode = labelTheRoot;
        }
      }
      while(NextNode == -1);
      currentNode = NextNode;
    }
    while(currentNode!=labelTheRoot);
  }

//*
  // Generate the necessary c++ files for a standalone templated RNEA
  void Converter::generateCCFile(std::string & aSpecificitiesFileName,
                                 std::string & aPath,
                                 std::string & aName,
                                 std::string & aMapFromJointToRank,
                                 const std::string & robotName)
  {
    dynamicsJRLJapan::ObjectFactory aRobotDynamicsObjectConstructor;
    CjrlHumanoidDynamicRobot * aHDR
      = aRobotDynamicsObjectConstructor.createHumanoidDynamicRobot();

    if (aHDR==0)
    {
      cerr<< "Dynamic cast on HDR failed " << endl;
      exit(-1);
    }

    std::stringstream ss_body, ss_joint, ss_robot, ss_lib_cc, ss_lib_hh;
    std::stringstream ss_path;
    ss_path << "models/" << robotName;
    boost::filesystem::create_directory("models");
    boost::filesystem::create_directory(ss_path.str());

    ss_body << ss_path.str() << "/body.hh";
    ss_joint << ss_path.str() << "/joint.hh";
    ss_robot << ss_path.str() << "/robot.hh";
    ss_lib_hh << ss_path.str() << "/" << robotName << ".hh";
    ss_lib_cc << ss_path.str() << "/" << robotName << ".cc";
    std::ofstream body_hh(ss_body.str().c_str(), std::ofstream::out),
                  joint_hh(ss_joint.str().c_str(), std::ofstream::out),
                  robot_hh(ss_robot.str().c_str(), std::ofstream::out),
                  lib_hh(ss_lib_hh.str().c_str(), std::ofstream::out),
                  lib_cc(ss_lib_cc.str().c_str(), std::ofstream::out);


    string RobotFileName = aPath+aName;
    dynamicsJRLJapan::parseOpenHRPVRMLFile(*aHDR,
                                           RobotFileName,
                                           aMapFromJointToRank,
                                           aSpecificitiesFileName);

    int nbDof, nbBodies;
    computeNbDof(aHDR, &nbDof, &nbBodies);

    makeLibraryHeader(lib_hh, robotName);
    addLicense(body_hh);
    addLicense(joint_hh);
    addLicense(robot_hh);
    addLicense(lib_cc);

    joint_hh
      << "/*\n"
      << " * This file is part of the " << robotName << " robot model.\n"
      << " * It contains the definition of all the robot joints.\n"
      << " */\n\n"
      << "#ifndef METAPOD_" << robotName << "_JOINT_HH\n"
      << "# define METAPOD_" << robotName << "_JOINT_HH\n\n"
      << "# include \"metapod/tools/jointmacros.hh\"\n\n"
      << "namespace metapod\n"
      << "{\n"
      << "  namespace " << robotName << "\n"
      << "  {\n";
    body_hh
      << "/*\n"
      << " * This file is part of the " << robotName << " robot model.\n"
      << " * It contains the definition of all the robot bodies.\n"
      << " */\n\n"
      << "#ifndef METAPOD_" << robotName << "_BODY_HH\n"
      << "# define METAPOD_" << robotName << "_BODY_HH\n\n"
      << "# include \"metapod/tools/bodymacros.hh\"\n\n"
      << "namespace metapod\n"
      << "{\n"
      << "  namespace " << robotName << "\n"
      << "  {\n";
    robot_hh
      << "/*\n"
      << " * This file is part of the " << robotName << " robot model.\n"
      << " * It defines the tree structure of the robot.\n"
      << " */\n\n"
      << "#ifndef METAPOD_" << robotName << "_ROBOT_HH\n"
      << "# define METAPOD_" << robotName << "_ROBOT_HH\n\n"
      << "# include \"metapod/tools/common.hh\"\n"
      << "# include \"joint.hh\"\n"
      << "# include \"body.hh\"\n"
      << std::endl
      << "namespace metapod\n"
      << "{\n"
      << "  namespace " << robotName << "\n"
      << "  {\n"
      << "    // Model of the robot. Contains data at the global robot level"
        << " and the tree\n"
      << "    // of Body/Joint\n"
      << "    class METAPOD_DLLEXPORT Robot\n"
      << "    {\n"
      << "      public:\n"
      << "        // Global constants or variable of the robot\n"
      << "        enum { NBDOF = " << nbDof << " };\n"
      << "        static Eigen::Matrix< FloatType, NBDOF, NBDOF > H;\n"
      << "        typedef Eigen::Matrix< FloatType, NBDOF, 1 > confVector;\n";
    lib_cc
      << "/*\n"
      << " * This file is part of the " << robotName << " robot model.\n"
      << " * It contains the initialization of all the robot bodies and joints.\n"
      << " */\n\n"
      << "# include \"" << robotName << ".hh\"\n\n"
      << "template struct metapod::crba< metapod::" << robotName
        << "::Robot , true >;\n"
      << "template struct metapod::rnea< metapod::" << robotName
        << "::Robot , true >;\n"
      << "template struct metapod::crba< metapod::" << robotName
        << "::Robot , false >;\n"
      << "template struct metapod::rnea< metapod::" << robotName
        << "::Robot , false >;\n"
      << "\n"
      << "namespace metapod\n"
      << "{\n"
      << "  namespace " << robotName << std::endl
      << "  {\n"
      << "    // Initialization of the robot global constants\n"
      << "    Eigen::Matrix< FloatType, Robot::NBDOF, Robot::NBDOF > Robot::H;\n\n";

    buildRobot(aHDR, joint_hh, body_hh, lib_cc);
    makeTree(aHDR, robot_hh, nbBodies);

    joint_hh
      << "  } // end of namespace " << robotName << std::endl
      << "} // end of namespace metapod\n\n"
      << "#endif";
    body_hh
      << "  } // end of namespace " << robotName << std::endl
      << "} // end of namespace metapod\n\n"
      << "#endif";
    robot_hh
      << "                    > Tree;\n"
      << "    };\n"
      << "  } // end of namespace " << robotName << std::endl
      << "} // end of namespace metapod\n\n"
      << "#endif";
    lib_cc
      << "  } // end of namespace " << robotName << std::endl
      << "} // end of namespace metapod";
    delete aHDR;
  }
//*/

  void Converter::addLicense(std::ofstream & of)
  {
    of
      << "// Copyright 2011, 2012,\n"
      << "//\n"
      << "// Maxime Reis\n"
      << "//\n"
      << "// JRL/LAAS, CNRS/AIST\n"
      << "//\n"
      << "// This file is part of metapod.\n"
      << "// metapod is free software: you can redistribute it and/or modify\n"
      << "// it under the terms of the GNU Lesser General Public License as published by\n"
      << "// the Free Software Foundation, either version 3 of the License, or\n"
      << "// (at your option) any later version.\n"
      << "//\n"
      << "// metapod is distributed in the hope that it will be useful,\n"
      << "// but WITHOUT ANY WARRANTY; without even the implied warranty of\n"
      << "// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the\n"
      << "// GNU Lesser General Public License for more details.\n"
      << "// You should have received a copy of the GNU Lesser General Public License\n"
      << "// along with metapod.  If not, see <http://www.gnu.org/licenses/>.\n\n";
  }

  void Converter::computeNbDof(CjrlDynamicRobot * aHDR,
                              int * nbDof_,
                              int * nbBodies_)
  {
    int labelTheRoot = 1;
    dynamicsJRLJapan::DynamicBodyPrivate *currentBody=0;
    int currentNode = labelTheRoot;
    dynamicsJRLJapan::DynMultiBodyPrivate * robot
      = dynamic_cast< dynamicsJRLJapan::DynMultiBodyPrivate* >(aHDR);
    int nbBodies = 0;
    int nbDof = 0;
  
    do
    {
      nbBodies++;
      currentBody = robot->getListOfBodies()[currentNode];
      nbDof += currentBody->getJointPrivate()->numberDof();
      int step = 0;
      int NextNode = 0;
      do
      {
        switch(step)
        {
          case 0:
            NextNode = currentBody->child;
            step++;
            break;
          case 1:
            NextNode = currentBody->sister;
            step++;
            break;
          case 2:
            NextNode = currentBody->getLabelMother();
            if(NextNode >= 0)
            {
              currentNode = NextNode;
              currentBody = robot->getListOfBodies()[currentNode];
              NextNode = currentBody->sister;
            }
            else
              NextNode = labelTheRoot;
        }
      }
      while(NextNode == -1);
      currentNode = NextNode;
    }
    while(currentNode!=labelTheRoot);
    *nbDof_ = nbDof;
    *nbBodies_ = nbBodies;
  }

  void Converter::makeLibraryHeader(std::ofstream & lib_hh,
                                    const std::string & robotName)
  {
    addLicense(lib_hh);
    lib_hh
      << "/*\n"
      << " * This file is part of the " << robotName << " robot model.\n"
      << " * It is the header of the corresponding library.\n"
      << " */\n\n"
      << "#ifndef METAPOD_" << robotName << "_JOINT_HH\n"
      << "# define METAPOD_" << robotName << "_JOINT_HH\n\n"
      << "# include \"metapod/tools/common.hh\"\n"
      << "# include \"metapod/algos/rnea.hh\"\n"
      << "# include \"metapod/algos/crba.hh\"\n\n"
      << "# include \"robot.hh\"\n\n"
      << "extern template struct metapod::crba< metapod::" << robotName
        << "::Robot , true >;\n"
      << "extern template struct metapod::rnea< metapod::" << robotName
        << "::Robot , true >;\n"
      << "extern template struct metapod::crba< metapod::" << robotName
        << "::Robot , false >;\n"
      << "extern template struct metapod::rnea< metapod::" << robotName
        << "::Robot , false >;\n\n"
      << "#endif";
  }
} // end of namespace MultibodyConverter.
