/*! \file GeometricData.h Geometric data for robots when available.
  

  Copyright (c) 2010
  @author Olivier Stasse
  
  JRL-Japan, CNRS/AIST
  
  All rights reserved.
  
  Please see License.txt for more informations on the license related to this software.
*/

#ifndef _DYNAMICS_JRL_JAPAN_GEOMETRIC_DATA_H_
#define _DYNAMICS_JRL_GEOMETRIC_DATA_H_

#if defined (WIN32)
#  ifdef dynamicsJRLJapan_EXPORTS 
#    define DYN_JRL_JAPAN_EXPORT __declspec(dllexport)
#  else  
#    define DYN_JRL_JAPAN_EXPORT __declspec(dllimport)
#  endif 
#else
#  define DYN_JRL_JAPAN_EXPORT
#endif

#include <vector>
#include <string>

#include <MatrixAbstractLayer/MatrixAbstractLayer.h>
#include "robotDynamics/jrlHumanoidDynamicRobot.h"
#include "robotDynamics/jrlRobotDynamicsObjectConstructor.h"

namespace dynamicsJRLJapan
{
  namespace Geometry
  {
    struct DYN_JRL_JAPAN_EXPORT Material
    {
      float ambientIntensity;
      float diffuseColor[3];
      float emissiveColor[3];
      float specularColor[3];
      float shininess[3];
      float transparency;
      
    Material():
      ambientIntensity(0.2),
	diffuseColor({0.8,0.8,0.8}),
	emissiveColor({0.0,0.0,0.0}),
	shininess({0.2}),
	specularColor({0.0,0.0,0.0}),
	transparency({0})
      {};

    };

    struct DYN_JRL_JAPAN_EXPORT Texture
    {
      std::string FileName;
    }
    
    struct DYN_JRL_JAPAN_EXPORT TextureTransform
    {
      float center[2];
      float rotation;
      float scale[2];
      float translation[2];

    TextureTransform():
      center({0.0,0.0}),
	rotation(0),
	scale({1.0,1.0}),
	translation({0.0,0.0})
      {};
    }

    
    class DYN_JRL_JAPAN_EXPORT Appearance
    {
    private:
      
      Material         m_Material;
      Texture          m_Texture;
      TextureTransform m_TextureTransform;

    public:
      Appearance();
      ~Appearance();
      /*! \name Setter and getters 
	@{
       */
      void setMaterial(Material &aMaterial);
      const Material &getMaterial() const;

      void setTexture(Texture &aTexture);
      const Texture &getTexture() const;
      
      void setTextureTransform(TextureTransform &aTexture);
      const TextureTransform &getTextureTransform() const;
      /*! @} */
    };

    class DYN_JRL_JAPAN_EXPORT Shape 
    {
      
    };
  };
};

#endif /*! _DYNAMICS_JRL_GEOMETRIC_DATA_H_ */
