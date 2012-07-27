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
 * \brief Declaration of Converter.
 */

#ifndef MULTIBODY_CONVERTER_CONVERTER_HH
# define MULTIBODY_CONVERTER_CONVERTER_HH

# include <iostream> 
# include <fstream>

# include "jrl/mal/matrixabstractlayer.hh"
# include <abstract-robot-dynamics/dynamic-robot.hh>
# include "JointPrivate.h"
# include "MultiBody.h"
# include "DynamicBodyPrivate.h"
# include "DynMultiBodyPrivate.h"
# include <jrl/dynamics/dynamicsfactory.hh>

# include <boost/filesystem/operations.hpp>

namespace MultibodyConverter
{
  /// \brief This class has been generated automatically
  /// See Doxygen documentation to learn how to document your classes:
  /// http://www.stack.nl/~dimitri/doxygen/
  class Converter
  {
  public:
    /// Main constructor.
    Converter ();
    /// Destructor.
    ~Converter ();

    /** \brief Make header for the generated .cc file */
    void addLicense(std::ofstream & of);
    /** \brief Add a body in the output .cc file from a jrl-dynamic body */
    void addBody(dynamicsJRLJapan::DynamicBodyPrivate *b,
                 std::ofstream & body_hh,
                 std::ofstream & lib_cc);
    std::string getJoint(dynamicsJRLJapan::DynamicBodyPrivate *b);
    std::string getParent(dynamicsJRLJapan::DynamicBodyPrivate *b);
    /** \brief Add a joint in the output .cc file from a jrl-dynamics joint */
    void addJoint(dynamicsJRLJapan::DynamicBodyPrivate *b,
                  std::ofstream & joint_hh,
                  std::ofstream & lib_cc);
    /** \brief Browse a jrl-dynamics multibody to extract and write the joints
      and bodies in the output .cc file */
    void buildRobot(CjrlDynamicRobot * aHDR,
                    std::ofstream & joint_hh,
                    std::ofstream & body_hh,
                    std::ofstream & lib_cc);
    /** \brief Parse VRML file and create a jrl-dynamics multibody to convert */
    CjrlDynamicRobot * parseVRML(CjrlHumanoidDynamicRobot * aHDR,
                                 std::string aSpecificitiesFileName,
                                 std::string aPath, std::string aName,
                                 std::string aMapFromJointToRank);
    /** \brief Write and initialise a robot model in a .cc file from VRML files */
    void generateCCFile(std::string & aSpecificitiesFileName,
                        std::string & aPath,
                        std::string & aName,
                        std::string & aMapFromJointToRank,
                        const std::string & robotName);

    void addNode(int index,
                 std::vector< std::vector< int > > children,
                 std::vector< std::string > joints,
                 std::ofstream & model_hh,
                 CjrlDynamicRobot * aHDR,
                 int depth = 0);
    void makeTree(CjrlDynamicRobot * aHDR, ofstream & model_hh, int nbBodies);
    void computeNbDof(CjrlDynamicRobot * aHDR,
                     int * nbDof_,
                     int * nbBodies_);
    void makeLibraryHeader(std::ofstream & lib_hh,
                           const std::string & robotName);
  };


} // end of namespace MultibodyConverter.

#endif //! MULTIBODY_CONVERTER_CONVERTER_HH_
