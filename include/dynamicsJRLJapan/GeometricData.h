/*! \file GeometricData.h Geometric data for robots when available.
 *
 * Copyright 2010, 
 *
 * Olivier Stasse
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
#include <ostream>

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

      void reset();
	
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

      friend std::ostream & operator<<(std::ostream &, const Material &);
    };

    std::ostream &operator<<(std::ostream &,const Material &);

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
      const Material &getMaterial() const ;

      void setTexture(Texture &aTexture);
      Texture & getTexture();
      const Texture &getTexture() const;
      
      void setTextureTransform(TextureTransform &aTextureTransform);
      TextureTransform & getTextureTransform();
      const TextureTransform &getTextureTransform() const;
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
       const Appearance & getAppearance() const;
       Appearance & getAppearance();

       void setIndexedFaceSet(IndexedFaceSet &anIndexedFaceSet);
       const IndexedFaceSet & getIndexedFaceSet() const ;
       IndexedFaceSet & getIndexedFaceSet();
       
       void reset();
     };

   };

   class DYN_JRL_JAPAN_EXPORT BodyGeometricalData
   {
   private:
     matrix3d m_RotationForDisplay;
     std::vector < std::string > m_URLs;
     std::vector < Geometry::Shape > m_Shapes;
     std::string m_BodyName;
     std::string m_RelatedJointName;

  public:

    BodyGeometricalData();
    ~BodyGeometricalData();

    const matrix3d & getRotationForDisplay() const;
    void setRotationForDisplay(const matrix3d &RotationForDisplay);

    const std::vector< std::string > & getURLs() const;
    void resetURL( );
    void addURL(const std::string & URLtoVRML);
    
    const std::vector<Geometry::Shape > & getShapes() const;
    void addShape(Geometry::Shape aShape);
    
    BodyGeometricalData & operator=(const BodyGeometricalData & );

    void setBodyName(const std::string &);
    const std::string & getBodyName() const;
    std::string & getBodyName();

    void setRelatedJointName(const std::string &);
    const std::string & getRelatedJointName() const;
    std::string & getRelatedJointName();
   };

};

#endif /*! _DYNAMICS_JRL_GEOMETRIC_DATA_H_ */
