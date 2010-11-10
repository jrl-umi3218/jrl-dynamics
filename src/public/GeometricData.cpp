/*
 * Copyright 2010,
 *
 * Olivier Stasse,
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
#include <jrl/dynamics/geometricdata.hh>


namespace dynamicsJRLJapan
{
  namespace Geometry
  {

    /*! Material object. */
    Material::Material()
    {
      ambientIntensity = 0.2;
      diffuseColor[0] = 0.8;
      diffuseColor[1] = 0.8;
      diffuseColor[2] = 0.8;
      emissiveColor[0] = 0.0;
      emissiveColor[1] = 0.0;
      emissiveColor[2] = 0.0;
      shininess = 0.2;
      specularColor[0] =
	specularColor[1] =
	specularColor[2] = 0.0;
      transparency=0.0;
    }

    std::ostream & operator<<(std::ostream &os, const Material &a)
    {
      os << "ambientIntensity:"  << a.ambientIntensity << std::endl;
      os << "diffuseColor:"
	 << a.diffuseColor[0] << " "
	 << a.diffuseColor[1] << " "
	 << a.diffuseColor[2] << std::endl;
      os << "emissiveColor:"
	 << a.emissiveColor[0] << " "
	 << a.emissiveColor[1] << " "
	 << a.emissiveColor[2] << std::endl;
      os << "shininess:" << a.shininess<< std::endl;
      os << "specularColor:"
	 << a.specularColor[0] << " "
	 << a.specularColor[1] << " "
	 << a.specularColor[2] << std::endl;
      os << "transparency:" << a.transparency << std::endl;
      return os;
    }

    /*! Texture Transform */
    TextureTransform::TextureTransform()
    {
      center[0] = center[1] = center[2] = 0.0;
      rotation = 0.0;
      scale[0] = scale[1] = 1.0;
      translation[0] = 0.0;
      translation[1] = 0.0;

    }

    void IndexedFaceSet::reset()
    {
      colorIndex.clear();
      coordIndex.clear();
      normalIndex.clear();
      texCoordIndex.clear();
      coord.clear();
    }

    /*! Appearance object */
    Appearance::Appearance()
    {}
    Appearance::~Appearance()
    {}

    void Appearance::setMaterial(Material &aMaterial)
    {
      m_Material = aMaterial;
    }

    const Material & Appearance::getMaterial() const
    {
      return m_Material;
    }

    Material & Appearance::getMaterial()
    {
      return m_Material;
    }

    void Appearance::setTexture(Texture &aTexture)
    {
      m_Texture = aTexture;
    }

    const Texture & Appearance::getTexture() const
    {
      return m_Texture;
    }

    Texture & Appearance::getTexture()
    {
      return m_Texture;
    }

    void Appearance::setTextureTransform(TextureTransform &aTextureTransform)
    {
      m_TextureTransform = aTextureTransform;
    }

    const TextureTransform & Appearance::getTextureTransform() const
    {
      return m_TextureTransform;
    }

    TextureTransform & Appearance::getTextureTransform()
    {
      return m_TextureTransform;
    }

    /*! Shape object.*/
    Shape::Shape()
    {}

    Shape::~Shape()
    {}

    void Shape::setAppearance(Appearance &anAppearance)
    {
      m_Appearance = anAppearance;
    }

    const Appearance & Shape::getAppearance() const
    {
      return m_Appearance;
    }

    Appearance & Shape::getAppearance()
    {
      return m_Appearance;
    }


    void Shape::setIndexedFaceSet(IndexedFaceSet &anIndexedFaceSet)
    {
      m_IndexedFaceSet = anIndexedFaceSet;
    }

    const IndexedFaceSet & Shape::getIndexedFaceSet() const
    {
      return m_IndexedFaceSet;
    }

    IndexedFaceSet & Shape::getIndexedFaceSet()
    {
      return m_IndexedFaceSet;
    }

    void Shape::reset()
    {
      m_IndexedFaceSet.reset();
    }

  }


  BodyGeometricalData::BodyGeometricalData():
    m_RotationForDisplay(1,0,0, 0,1,0, 0,0,1),
    m_URLs(0),
    m_Shapes(0)
  {}


  BodyGeometricalData::~BodyGeometricalData()
  {
  }

  const matrix3d & BodyGeometricalData::getRotationForDisplay() const
  {
    return m_RotationForDisplay;
  }

  void BodyGeometricalData::setRotationForDisplay(const matrix3d & RotationForDisplay)
  {
    m_RotationForDisplay = RotationForDisplay;
  }

  const std::vector< std::string > & BodyGeometricalData::getURLs() const
  {
    return m_URLs;
  }

  void BodyGeometricalData::resetURL( )
  {
    m_URLs.clear();
  }

  void BodyGeometricalData::addURL(const std::string & URLtoVRML)
  {
    //    std::string * mys= new std::string(URLtoVRML);
    m_URLs.push_back(URLtoVRML);
  }

  void BodyGeometricalData::addShape(Geometry::Shape aShape)
  {
    m_Shapes.push_back(aShape);
  }

  const std::vector< Geometry::Shape > & BodyGeometricalData::getShapes() const
  {
    return m_Shapes;
  }

  BodyGeometricalData & BodyGeometricalData::operator=(const BodyGeometricalData & r)
  {
    const std::vector<std::string> &lURLs = r.getURLs();
    m_URLs = lURLs;
    m_Shapes = r.getShapes();
    m_RotationForDisplay = r.getRotationForDisplay();
    m_BodyName = r.getBodyName();
    m_RelatedJointName = r.getRelatedJointName();
    return *this;

  }

  void BodyGeometricalData::setBodyName(const std::string &aBodyName)
  {
    m_BodyName = aBodyName;
  }

  std::string & BodyGeometricalData::getBodyName()
  {
    return m_BodyName;
  }

  const std::string & BodyGeometricalData::getBodyName()const
  {
    return m_BodyName;
  }

  void BodyGeometricalData::setRelatedJointName(const std::string &aRelatedJointName)
  {
    m_RelatedJointName = aRelatedJointName;
  }

  std::string & BodyGeometricalData::getRelatedJointName()
  {
    return m_RelatedJointName;
  }

  const std::string & BodyGeometricalData::getRelatedJointName()const
  {
    return m_RelatedJointName;
  }


}
