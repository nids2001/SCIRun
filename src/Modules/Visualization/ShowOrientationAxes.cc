/*
   For more information, please see: http://software.sci.utah.edu
   The MIT License
   Copyright (c) 2020 Scientific Computing and Imaging Institute,
   University of Utah.
   Permission is hereby granted, free of charge, to any person obtaining a
   copy of this software and associated documentation files (the "Software"),
   to deal in the Software without restriction, including without limitation
   the rights to use, copy, modify, merge, publish, distribute, sublicense,
   and/or sell copies of the Software, and to permit persons to whom the
   Software is furnished to do so, subject to the following conditions:
   The above copyright notice and this permission notice shall be included
   in all copies or substantial portions of the Software.
   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
   OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
   THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
   FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
   DEALINGS IN THE SOFTWARE.
*/


#include <Core/Algorithms/Visualization/RenderFieldState.h>
#include <Modules/Visualization/ShowOrientationAxes.h>
#include <Core/GeometryPrimitives/BBox.h>
#include <Core/GeometryPrimitives/Point.h>
#include <Core/GeometryPrimitives/Vector.h>
#include <Graphics/Datatypes/GeometryImpl.h>
#include <Graphics/Glyphs/GlyphGeom.h>

using namespace SCIRun;
using namespace Modules::Visualization;
using namespace Dataflow::Networks;
using namespace Core;
using namespace Core::Algorithms;
using namespace Core::Datatypes;
using namespace Core::Geometry;
using namespace Graphics;
using namespace Graphics::Datatypes;

MODULE_INFO_DEF(ShowOrientationAxes, Visualization, SCIRun);

namespace SCIRun {
namespace Modules {
namespace Visualization {
  class ShowOrientationAxesImpl
  {
  public:
    ShowOrientationAxesImpl();
    void setScale(double scale);
    void setPosition(double x, double y, double z);
    GeometryHandle makeGeometry(const GeometryIDGenerator& id) const;
  private:
    RenderState getRenderState() const;
    void addOrientationArrows(GlyphGeom& glyphs, const Point& pos) const;
    BBox getBBox(const Point& pos) const;
    BBox bbox_;
    double x_ = 0.0;
    double y_ = 0.0;
    double z_ = 0.0;
    double scale_ = 1.0;
    const int RESOLUTION_ = 20;
    const double ARROW_RATIO_ = 0.2;
    const bool RENDER_CYLINDER_BASE_ = false;
    const bool RENDER_CONE_BASE_ = true;
    const ColorRGB RED_ = ColorRGB(1,0,0);
    const ColorRGB GREEN_ = ColorRGB(0,1,0);
    const ColorRGB BLUE_ = ColorRGB(0,0,1);
    const ColorRGB RED_NEGATIVE_ = ColorRGB(0.3,0,0);
    const ColorRGB GREEN_NEGATIVE_ = ColorRGB(0,0.3,0);
    const ColorRGB BLUE_NEGATIVE_ = ColorRGB(0,0,0.3);
  };
}}}

ShowOrientationAxesImpl::ShowOrientationAxesImpl()
{}

void ShowOrientationAxesImpl::setPosition(double x, double y, double z)
{
  x_ = x;
  y_ = y;
  z_ = z;
}

void ShowOrientationAxesImpl::setScale(double scale)
{
  scale_ = scale;
}

RenderState ShowOrientationAxesImpl::getRenderState() const
{
  RenderState renState;
  renState.set(RenderState::USE_NORMALS, true);
  renState.set(RenderState::IS_ON, true);
  renState.set(RenderState::USE_TRANSPARENCY, false);
  renState.mGlyphType = RenderState::GlyphType::ARROW_GLYPH;
  renState.set(RenderState::USE_DEFAULT_COLOR, false);
  return renState;
}

void ShowOrientationAxesImpl::addOrientationArrows(GlyphGeom& glyphs, const Point& pos) const
{
  double radius = 0.1 * scale_;

  // Positive
  glyphs.addArrow(pos, pos + Vector(scale_, 0, 0), radius, ARROW_RATIO_, RESOLUTION_,
                  RED_, RED_, RENDER_CYLINDER_BASE_, RENDER_CONE_BASE_);
  glyphs.addArrow(pos, pos + Vector(0, scale_, 0), radius, ARROW_RATIO_, RESOLUTION_,
                  GREEN_, GREEN_, RENDER_CYLINDER_BASE_, RENDER_CONE_BASE_);
  glyphs.addArrow(pos, pos + Vector(0, 0, scale_), radius, ARROW_RATIO_, RESOLUTION_,
                  BLUE_, BLUE_, RENDER_CYLINDER_BASE_, RENDER_CONE_BASE_);

  // Negative
  glyphs.addArrow(pos, pos - Vector(scale_, 0, 0), radius, ARROW_RATIO_, RESOLUTION_,
                  RED_NEGATIVE_, RED_NEGATIVE_, RENDER_CYLINDER_BASE_, RENDER_CONE_BASE_);
  glyphs.addArrow(pos, pos - Vector(0, scale_, 0), radius, ARROW_RATIO_, RESOLUTION_,
                  GREEN_NEGATIVE_, GREEN_NEGATIVE_, RENDER_CYLINDER_BASE_, RENDER_CONE_BASE_);
  glyphs.addArrow(pos, pos - Vector(0, 0, scale_), radius, ARROW_RATIO_, RESOLUTION_,
                  BLUE_NEGATIVE_, BLUE_NEGATIVE_, RENDER_CYLINDER_BASE_, RENDER_CONE_BASE_);
}

BBox ShowOrientationAxesImpl::getBBox(const Point& pos) const
{
  BBox bbox = BBox();
  Vector halfDiag = Vector(scale_, scale_, scale_);
  bbox.extend(pos - halfDiag);
  bbox.extend(pos + halfDiag);
  return bbox;
}

GeometryHandle ShowOrientationAxesImpl::makeGeometry(const GeometryIDGenerator& idGen) const
{
  GlyphGeom glyphs;
  Point pos = Point(x_, y_, z_);

  addOrientationArrows(glyphs, pos);
  BBox bbox = getBBox(pos);

  auto geom(boost::make_shared<GeometryObjectSpire>(idGen, "ShowOrientationAxes", true));
  glyphs.buildObject(*geom, geom->uniqueID(), false, 1.0, ColorScheme::COLOR_IN_SITU,
                     getRenderState(), SpireIBO::PRIMITIVE::TRIANGLES, bbox, true, nullptr);
  return geom;
}

ShowOrientationAxes::ShowOrientationAxes() : GeometryGeneratingModule(staticInfo_),
                                             impl_(new ShowOrientationAxesImpl)
{
  INITIALIZE_PORT(OutputGeom);
}

void ShowOrientationAxes::setStateDefaults()
{
  auto state = get_state();
  state->setValue(Scale, 1.0);
  state->setValue(X, 0.0);
  state->setValue(Y, 0.0);
  state->setValue(Z, 0.0);
}

void ShowOrientationAxes::execute()
{
  if (needToExecute())
  {
    auto state = get_state();
    impl_->setScale(state->getValue(Scale).toDouble());
    impl_->setPosition(state->getValue(X).toDouble(), state->getValue(Y).toDouble(),
                       state->getValue(Z).toDouble());

    sendOutput(OutputGeom, impl_->makeGeometry(*this));
  }
}

const AlgorithmParameterName ShowOrientationAxes::Scale("Scale");
const AlgorithmParameterName ShowOrientationAxes::X("X");
const AlgorithmParameterName ShowOrientationAxes::Y("Y");
const AlgorithmParameterName ShowOrientationAxes::Z("Z");
