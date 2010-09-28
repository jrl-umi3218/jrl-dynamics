jrl-mal
=======

This software implements the interfaces for dynamics computation
specified in the [abstract-robot-dynamics package][abstract-robot-dynamics].

The algorithm implemented by this package is the [recursive
Newton-Euler algorithm][RNEA].

It also provides tools to parse an extended VRML file into a robot as
defined by the abstract robot dynamics specifications.


Setup
-----

To compile this package, it is recommended to create a separate build
directory:

    mkdir _build
    cd _build
    cmake [OPTIONS] ..
    make install

Please note that CMake produces a `CMakeCache.txt` file which should
be deleted to reconfigure a package from scratch.


### Dependencies

The matrix abstract layer depends on several packages which
have to be available on your machine.

 - Libraries:
   - Boost (>= 1.33.1)
     Boost Spirit is used to parse VRML robot models.
   - [jrl-mal][jrl-mal] (>= 1.8.3)
     The matrix abstract layer to abstract the numerical types used by
     jrl-dynamic in order to make it easily pluggable into a larger robotics
     framework.
   - [abstract-robot-dynamics][abstract-robot-dynamics] (>= 1.16)
     The abstract robot dynamics interface are implemented by this package.
 - System tools:
   - CMake (>=2.6)
   - pkg-config
   - usual compilation tools (GCC/G++, make, etc.)


Release notes
-------------

The main goal of the recent modifications is a strict compliance
to [abstract-robot-dynamics][abstract-robot-dynamics].
Therefore any code relying on direct access to jrl-dynamic old
structures will not compile with this new version.

Moreover, this version includes a normalization of all the joint to
revolute around the x-axis. This implies a modification of all the
local frames.

Finally they are some modifications on the parser which allow to get
geometrical information on the parsed robot model.

Depending on the transformation applied to the VRML files you might
have to compensate for some local rotation (for instance the left
grippers of HRP-2 10 & 14).

The parser provides the rotation on the local frames to compensate for
those transformations.

In tools, you can find a program called CreateVRMLPoseFile.
It takes the usual files plus an additional one specifying
the pose of the robot. From those files, it generates a VRML file
that can be imported in Blender.



[abstract-robot-dynamics]: http://github.com/laas/abstract-robot-dynamics
[jrl-mal]: http://github.com/jrl-umi3218/jrl-mal

[RNEA]: http://www.scholarpedia.org/article/Robot_dynamics
