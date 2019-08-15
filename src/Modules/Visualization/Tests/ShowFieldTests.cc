/*
   For more information, please see: http://software.sci.utah.edu

   The MIT License

   Copyright (c) 2015 Scientific Computing and Imaging Institute,
   University of Utah.

   License for the specific language governing rights and limitations under
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

#include <Testing/ModuleTestBase/ModuleTestBase.h>
#include <Modules/Visualization/ShowField.h>
#include <Core/Datatypes/Legacy/Field/Field.h>
#include <Core/Algorithms/Base/AlgorithmVariableNames.h>
#include <Core/Utils/Exception.h>
#include <Core/Logging/Log.h>
#include <Core/Datatypes/ColorMap.h>
#include <Core/Datatypes/Legacy/Field/VMesh.h>
#include <Core/Datatypes/Legacy/Field/VField.h>
#include <Testing/Utils/SCIRunUnitTests.h>
#include <Testing/Utils/MatrixTestUtilities.h>

using namespace SCIRun::Testing;
using namespace SCIRun::TestUtils;
using namespace SCIRun::Core::Datatypes;
using namespace SCIRun::Dataflow::Networks;
using namespace SCIRun::Core::Algorithms;
using namespace SCIRun::Core::Geometry;
using namespace SCIRun::Core::Algorithms::Visualization;
using namespace SCIRun::Modules::Visualization;
using namespace SCIRun::Core;
using namespace SCIRun;
using namespace SCIRun::Core::Logging;
using ::testing::Values;
using ::testing::Combine;
using ::testing::Range;
using ::testing::TestWithParam;
using ::testing::Bool;


class ShowFieldScalingTest : public ParameterizedModuleTest<int>
{
protected:
  virtual void SetUp()
  {
    LogSettings::Instance().setVerbose(false);
    showField = makeModule("ShowField");
    showField->setStateDefaults();
    auto size = GetParam();
    latVol = CreateEmptyLatVol(size, size, size);
    stubPortNWithThisData(showField, 0, latVol);
    LOG_DEBUG("Setting up ShowField with size {}^3 latvol", size);
  }

  UseRealModuleStateFactory f;
  ModuleHandle showField;
  FieldHandle latVol;
};

TEST_P(ShowFieldScalingTest, ConstructLatVolGeometry)
{
  LOG_DEBUG("Start ShowField::execute");
  showField->execute();
  LOG_DEBUG("End ShowField::execute");
}

INSTANTIATE_TEST_CASE_P(
  ConstructLatVolGeometry,
  ShowFieldScalingTest,
  Values(20, 40, 60, 80
  //, 100, 120, 150
  //, //200 //to speed up make test
  //, 256 // probably runs out of memory
  )
  );

class ShowFieldStateGeometryNameSynchronizationTest : public ModuleTest
{
protected:
  virtual void SetUp()
  {
    LogSettings::Instance().setVerbose(false);
    showField = makeModule("ShowField");
    showField->setStateDefaults();
    auto size = 2;
    latVol = CreateEmptyLatVol(size, size, size);
    stubPortNWithThisData(showField, 0, latVol);
  }

  UseRealModuleStateFactory f;
  ModuleHandle showField;
  FieldHandle latVol;
};

TEST_F(ShowFieldStateGeometryNameSynchronizationTest, GeometryNameSynchronizesWithShowFieldState)
{
  ModuleLevelUniqueIDGenerator generator(*showField, "EntireField");
  auto hash1 = generator();
  auto hash2NoChange = generator();
  EXPECT_EQ(hash1, hash2NoChange);

  showField->get_state()->setValue(Parameters::CylinderRadius, 2);

  auto stateChangeShouldBeDifferent = generator();
  EXPECT_NE(hash2NoChange, stateChangeShouldBeDifferent);
  EXPECT_EQ(stateChangeShouldBeDifferent, generator());

  auto size = 3;
  latVol = CreateEmptyLatVol(size, size, size);
  stubPortNWithThisData(showField, 0, latVol);

  auto inputChangeShouldBeDifferent = generator();
  EXPECT_NE(stateChangeShouldBeDifferent, inputChangeShouldBeDifferent);

  stubPortNWithThisData(showField, 1, ColorMapHandle());
  auto addInputShouldBeDifferent = generator();
  EXPECT_NE(inputChangeShouldBeDifferent, addInputShouldBeDifferent);

  EXPECT_NE(hash1, addInputShouldBeDifferent);
  EXPECT_NE(inputChangeShouldBeDifferent, hash1);
}

struct FaceNode
{
  float x, y, z;
};

struct FaceNormal
{
  float x, y, z;
};

using FaceIndex = VMesh::Face::iterator;

#define XYZ_TO_FLOAT_TRIPLE(p) { static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z()) }

template <int Dim>
struct FaceData
{
  FaceNode nodes[Dim];
  static constexpr int Dimension = Dim;

  void fill(VField* field, FaceIndex index)
  {
    auto mesh = field->vmesh();
    
    mesh->get_nodes(fieldNodes, *index);

    for (size_t i = 0; i < Dim; ++i)
    {
      Point p;
      mesh->get_point(p, fieldNodes[i]);
      nodes[i] = XYZ_TO_FLOAT_TRIPLE(p);
    }
  }

protected:
  VMesh::Node::array_type fieldNodes;
};

template <typename Data>
struct FaceDataWithNormals : Data
{
  FaceNormal normals[Data::Dimension];

  void fill(VField* field, FaceIndex index)
  {
    Data::fill(field, index);

    //TODO details: generated/face normals, invert normals--again with parameterized mapping to lambda
    auto mesh = field->vmesh();

    for (size_t i = 0; i < Data::Dimension; ++i)
    {
      Vector n;
      mesh->get_normal(n, fieldNodes[i]);
      normals[i] = XYZ_TO_FLOAT_TRIPLE(n);
    }
  }
};

struct ColorTextureCoords
{
  float x, y;
};

template <typename Data>
struct FaceDataWithColor : Data
{
  ColorTextureCoords color[Data::Dimension];

  void fill(VField* field, FaceIndex index)
  {
    Data::fill(field, index);

    //TODO fill color based on field parameters, with no per-face if statements
  }
};

struct FaceDataTri : FaceData<3>
{
};

struct FaceDataTriColor : FaceDataWithColor<FaceDataTri> {};
struct FaceDataTriNormals : FaceDataWithNormals<FaceDataTri> {};
struct FaceDataTriColorNormals : FaceDataWithColor<FaceDataTriNormals> {};  // first normals, then color coords


struct FaceDataQuad : FaceData<4>
{
};

struct FaceDataQuadColor : FaceDataWithColor<FaceDataQuad> {};
struct FaceDataQuadNormals : FaceDataWithNormals<FaceDataQuad> {};
struct FaceDataQuadColorNormals : FaceDataWithColor<FaceDataQuadNormals> {};   // first normals, then color coords

#define PRINT_SIZEOF(type) std::cout << #type << " sizeof: " << sizeof(type) << std::endl

TEST(ShowFieldFaceDataStructTest, PrintSizeof)
{
  PRINT_SIZEOF(FaceNode);
  PRINT_SIZEOF(FaceNormal);
  PRINT_SIZEOF(FaceDataTri);
  PRINT_SIZEOF(FaceDataTriColor);
  PRINT_SIZEOF(FaceDataTriNormals);
  PRINT_SIZEOF(FaceDataTriColorNormals);
  PRINT_SIZEOF(FaceDataQuad);
  PRINT_SIZEOF(FaceDataQuadColor);
  PRINT_SIZEOF(FaceDataQuadNormals);
  PRINT_SIZEOF(FaceDataQuadColorNormals);
}

template <class FaceDataType>
class FaceDataGenerator
{
public:
  explicit FaceDataGenerator(VField* field) : field_(field) {}
  //TODO: initialize color-data generator from mapping based on field properties

  FaceDataType generate(FaceIndex faceIndex) const
  {
    FaceDataType f;
    f.fill(field_, faceIndex);
    //TODO: f.fillColor(field_, faceIndex, toColor_);
    return f;
  }
private:
  VField* field_;
  //ColorGenerator toColor_;
};

template <int N>
std::ostream& operator<<(std::ostream& o, const FaceData<N>& f)
{
  o << (N == 3 ? "Tri:\n" : "Quad:\n");
  for (const auto& p : f.nodes)
    o << "\tNode[" << p.x << ", " << p.y << ", " << p.z << "]\n";
  return o;
}

template <typename T>
std::ostream& operator<<(std::ostream& o, const FaceDataWithNormals<T>& f)
{
  o << (f.Dimension == 3 ? "Tri:\n" : "Quad:\n");
  for (const auto& p : f.nodes)
    o << "\tNode[" << p.x << ", " << p.y << ", " << p.z << "]\n";
  o << "Normals:\n";
  for (const auto& v : f.normals)
    o << "\tVector[" << v.x << ", " << v.y << ", " << v.z << "]\n";
  return o;
}

// stub
class VarBuffer
{
public:
  template <typename T>
  void write(const T& t) 
  {
    std::cout << "VarBuffer writing " << t << std::endl;
  }
};

template <class FaceDataType>
class FaceDataBufferWriter
{
public:
  FaceDataBufferWriter(VarBuffer* vbo, const FaceDataGenerator<FaceDataType>& gen) : vbo_(vbo), gen_(gen) {}

  void writeRange(FaceIndex begin, FaceIndex end) const
  {
    for (FaceIndex i = begin; i != end; ++i)
      vbo_->write(gen_.generate(i));
  }
private:
  mutable VarBuffer* vbo_;
  FaceDataGenerator<FaceDataType> gen_;
};

enum class WriteCase
{
  TRI, TRI_TEXCOORDS, TRI_NORMALS, TRI_NORMALS_TEXCOORDS,
  QUAD, QUAD_TEXCOORDS, QUAD_NORMALS, QUAD_NORMALS_TEXCOORDS
};

WriteCase getWriteCase(bool useQuads, bool useNormals, bool useColorMap)
{
  return WriteCase(((int)useQuads << 2) | ((int)useNormals << 1) | ((int)useColorMap));
}

struct WriteArgs
{
  VField* field;
  VarBuffer* vbo;
};

using WriteFaceRange = std::function<void(FaceIndex, FaceIndex)>;
using WriteFaceRangeMaker = std::function<WriteFaceRange(const WriteArgs&)>;

struct SizedFaceDataWriterMaker
{
  size_t sizeOfData;
  WriteFaceRangeMaker maker;
};

template <typename F>
WriteFaceRange makeWriteFaceRange(const WriteArgs& input)
{
  FaceDataBufferWriter<F> writer(input.vbo, FaceDataGenerator<F>(input.field));
  return [writer](FaceIndex begin, FaceIndex end) { writer.writeRange(begin, end); };
}

#define WRITE_FACE_MAKER_FOR_TYPE(T) [](const WriteArgs& input) { return makeWriteFaceRange<T>(input); }
#define WRITE_SIZED_FACE_MAKER_FOR_TYPE(T) { sizeof(T), [](const WriteArgs& input) { return makeWriteFaceRange<T>(input); } }

std::map<WriteCase, SizedFaceDataWriterMaker> faceGenerationMap =
{
  {WriteCase::TRI, WRITE_SIZED_FACE_MAKER_FOR_TYPE(FaceDataTri) },
  {WriteCase::TRI_TEXCOORDS, WRITE_SIZED_FACE_MAKER_FOR_TYPE(FaceDataTriColor) },
  {WriteCase::TRI_NORMALS, WRITE_SIZED_FACE_MAKER_FOR_TYPE(FaceDataTriNormals) },
  {WriteCase::TRI_NORMALS_TEXCOORDS, WRITE_SIZED_FACE_MAKER_FOR_TYPE(FaceDataTriColorNormals) },
  {WriteCase::QUAD, WRITE_SIZED_FACE_MAKER_FOR_TYPE(FaceDataQuad) },
  {WriteCase::QUAD_TEXCOORDS, WRITE_SIZED_FACE_MAKER_FOR_TYPE(FaceDataQuadColor) },
  {WriteCase::QUAD_NORMALS, WRITE_SIZED_FACE_MAKER_FOR_TYPE(FaceDataQuadNormals) },
  {WriteCase::QUAD_NORMALS_TEXCOORDS, WRITE_SIZED_FACE_MAKER_FOR_TYPE(FaceDataQuadColorNormals) }
};

//TODO: create similar mapping for color->texture functions based on mesh dim and basis


TEST(ShowFieldFaceDataStructTest, EnumerateWriteCases)
{
  for (bool useQuads : {false, true})
  {
    for (bool useNormals : {false, true})
    {
      for (bool useColorMap : {false, true})
      {
        auto writeCase = getWriteCase(useQuads, useNormals, useColorMap);
        auto sizedWriterFunc = faceGenerationMap[writeCase];
        std::cout << std::boolalpha <<
          "useQuads " << useQuads <<
          " useNormals " << useNormals <<
          " useColorMap " << useColorMap <<
          " WriteCaseEnum " << static_cast<int>(writeCase) <<
          " writerFuncLambda " << (sizedWriterFunc.maker ? "not null" : "null")  <<
          " writeSize " << sizedWriterFunc.sizeOfData << std::endl;
        EXPECT_TRUE(sizedWriterFunc.maker != nullptr);
      }
    }
  }
}

class ShowFieldFaceDataWriteTest : public ::testing::TestWithParam<std::tuple<bool, bool, bool>>
{
protected:
  virtual void SetUp()
  {
    LogSettings::Instance().setVerbose(false);
  }

  //TODO: when vbos are constructed for real...
  VarBuffer vbo;
};

TEST_P(ShowFieldFaceDataWriteTest, TestWritingFaces)
{
  for (auto f :
    {
     // CreateEmptyLatVol(2, 2, 2),
      loadFieldFromFile(TestResources::rootDir() / "Fields/extractsimpleisosurface/test_isosimsuf_tri.fld")
      //,
      //loadFieldFromFile(TestResources::rootDir() / "Fields/extractsimpleisosurface/test_isosimsuf_tet.fld"),
    //loadFieldFromFile(TestResources::rootDir() / "Fields/test_image_node.fld"),
    //  loadFieldFromFile(TestResources::rootDir() / "Fields/hexvol.fld"),
     // loadFieldFromFile(TestResources::rootDir() / "Fields/quadsurf.fld") 
    })
  {

    const bool useQuads = std::get<0>(GetParam());
    const bool useNormals = std::get<1>(GetParam());

    auto mesh = f->vmesh();
    auto field = f->vfield();
    mesh->synchronize(Mesh::FACES_E);
    if (useNormals)
    {
      mesh->synchronize(Mesh::NORMALS_E);
    }
    const bool useColorMap = std::get<2>(GetParam());
    auto writeCase = getWriteCase(useQuads, useNormals, useColorMap);
    auto sizedWriterFunc = faceGenerationMap[writeCase];

    std::cout << std::boolalpha <<
      "useQuads " << useQuads <<
      " useNormals " << useNormals <<
      " useColorMap " << useColorMap <<
      " WriteCaseEnum " << static_cast<int>(writeCase) <<
      " writerFuncLambda " << (sizedWriterFunc.maker ? "not null" : "null") <<
      " writeSize " << sizedWriterFunc.sizeOfData << std::endl;

    //TODO: when vbos are constructed for real...they use #faces * sizedWriterFunc.sizeOfData

    if (sizedWriterFunc.maker)
    {
      auto writeRange = sizedWriterFunc.maker({ field, &vbo });

      VMesh::Face::iterator b, e;
      mesh->begin(b);
      mesh->end(e);

      //TODO: easy multithreading with this
      writeRange(b, e);
    }
    else
    {
      FAIL() << "no writer func defined for write case";
    }
  }
}


INSTANTIATE_TEST_CASE_P(
  TestWritingFaces,
  ShowFieldFaceDataWriteTest,
  Combine(Bool(), Bool(), Bool())
);