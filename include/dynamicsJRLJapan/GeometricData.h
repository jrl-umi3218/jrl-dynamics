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

    typedef std::vector<int> polygonIndex;

    struct DYN_JRL_JAPAN_EXPORT IndexedFaceSet
    {
      bool ccw;
      std::vector<int> colorIndex;
      bool colorPerVertex;
      bool convex;
      std::vector<polygonIndex> coordIndex;
      float creaseAngle;
      std::vector<int> normalIndex;
      bool normalPerVertex;
      bool solid;
      std::vector<int> texCoordIndex;      
      std::vector<vector3d> coord;
    };    

    struct DYN_JRL_JAPAN_EXPORT Material
    {
      float ambientIntensity;
      float diffuseColor[3];
      float emissiveColor[3];
      float shininess;
      float specularColor[3];
      float transparency;
      
      Material();

    };

    struct DYN_JRL_JAPAN_EXPORT Texture
    {
      std::string FileName;
    };
    
    struct DYN_JRL_JAPAN_EXPORT TextureTransform
    {
      float center[2];
      float rotation;
      float scale[2];
      float translation[2];

      TextureTransform();
    };

    
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
      Material &getMaterial();

      void setTexture(Texture &aTexture);
      Texture &getTexture();
      
      void setTextureTransform(TextureTransform &aTextureTransform);
      TextureTransform &getTextureTransform();
      /*! @} */
    };

    class DYN_JRL_JAPAN_EXPORT Shape 
    {
    private:
      Appearance m_Appearance;
      IndexedFaceSet m_IndexedFaceSet;

    public:
      Shape();
      ~Shape();
       void setAppearance(Appearance &anAppearance);
       Appearance & getAppearance();

       void setIndexedFaceSet(IndexedFaceSet &anIndexedFaceSet);
       IndexedFaceSet & getIndexedFaceSet();

     };

   };

   class DYN_JRL_JAPAN_EXPORT BodyGeometricalData
   {
   private:
     matrix3d m_RotationForDisplay;
     std::vector < std::string > m_URLs;
     std::vector < Geometry::Shape > m_Shapes;

  public:
    BodyGeometricalData();
    ~BodyGeometricalData();

    const matrix3d & getRotationForDisplay();
    void setRotationForDisplay(const matrix3d &RotationForDisplay);

    const std::vector< std::string > & getURLs();
    void resetURL( );
    void addURL(const std::string & URLtoVRML);
    
    const std::vector<Geometry::Shape > & getShapes();
    void addShape(Geometry::Shape &aShape);
  };

};

#endif /*! _DYNAMICS_JRL_GEOMETRIC_DATA_H_ */
