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

#ifndef MODULES_FIELDS_EDITMESHBOUNDINGBOX_H
#define MODULES_FIELDS_EDITMESHBOUNDINGBOX_H

#include <Core/Datatypes/Geometry.h>
#include <Core/GeometryPrimitives/OrientedBBox.h>
#include <Dataflow/Network/GeometryGeneratingModule.h>
#include <Graphics/Datatypes/GeometryImpl.h>
#include <Graphics/Widgets/Widget.h>
#include <Graphics/Widgets/BoundingBoxWidget.h>
#include <Modules/Fields/share.h>

namespace SCIRun {
  namespace Modules {
    namespace Fields {

      class EditMeshBoundingBoxImpl;

      class SCISHARE EditMeshBoundingBox : public Dataflow::Networks::GeometryGeneratingModule,
        public Has1InputPort<FieldPortTag>,
        public Has3OutputPorts < FieldPortTag, GeometryPortTag, MatrixPortTag >
      {
      public:
        static const int mDIMENSIONS = 3;
        EditMeshBoundingBox();
        void execute() override;
        void setStateDefaults() override;

        static const Core::Algorithms::AlgorithmParameterName FieldTransformMatrix;
        static const Core::Algorithms::AlgorithmParameterName ScaleTransformMatrix;
        static const Core::Algorithms::AlgorithmParameterName RotationTransformMatrix;
        static const Core::Algorithms::AlgorithmParameterName TranslationPoint;
        static const Core::Algorithms::AlgorithmParameterName InverseFieldTransformMatrix;

        static const Core::Algorithms::AlgorithmParameterName ResetToInput;
        static const Core::Algorithms::AlgorithmParameterName ResetSize;
        static const Core::Algorithms::AlgorithmParameterName ResetCenter;
        static const Core::Algorithms::AlgorithmParameterName DataSaved;
        //Input Field Attributes
        static const Core::Algorithms::AlgorithmParameterName InputCenterX;
        static const Core::Algorithms::AlgorithmParameterName InputCenterY;
        static const Core::Algorithms::AlgorithmParameterName InputCenterZ;
        static const Core::Algorithms::AlgorithmParameterName InputSizeX;
        static const Core::Algorithms::AlgorithmParameterName InputSizeY;
        static const Core::Algorithms::AlgorithmParameterName InputSizeZ;
        //Output Field Atributes
        static const Core::Algorithms::AlgorithmParameterName SetOutputCenter;
        static const Core::Algorithms::AlgorithmParameterName SetOutputSize;
        static const Core::Algorithms::AlgorithmParameterName OutputCenterX;
        static const Core::Algorithms::AlgorithmParameterName OutputCenterY;
        static const Core::Algorithms::AlgorithmParameterName OutputCenterZ;
        static const Core::Algorithms::AlgorithmParameterName OutputSizeX;
        static const Core::Algorithms::AlgorithmParameterName OutputSizeY;
        static const Core::Algorithms::AlgorithmParameterName OutputSizeZ;
        //Widget Scale/Mode
        static const Core::Algorithms::AlgorithmParameterName Scale;
        static const Core::Algorithms::AlgorithmParameterName OldScale;
        static const Core::Algorithms::AlgorithmParameterName ScaleChanged;
        static const Core::Algorithms::AlgorithmParameterName NoTranslation;
        static const Core::Algorithms::AlgorithmParameterName XYZTranslation;
        static const Core::Algorithms::AlgorithmParameterName RDITranslation;
        static const Core::Algorithms::AlgorithmParameterName RestrictX;
        static const Core::Algorithms::AlgorithmParameterName RestrictY;
        static const Core::Algorithms::AlgorithmParameterName RestrictZ;
        static const Core::Algorithms::AlgorithmParameterName RestrictR;
        static const Core::Algorithms::AlgorithmParameterName RestrictD;
        static const Core::Algorithms::AlgorithmParameterName RestrictI;

        static const Core::Algorithms::AlgorithmParameterName BoxMode;
        static const Core::Algorithms::AlgorithmParameterName BoxRealScale;

        INPUT_PORT(0, InputField, Field);
        OUTPUT_PORT(0, OutputField, Field);
        OUTPUT_PORT(1, Transformation_Widget, GeometryObject);
        OUTPUT_PORT(2, Transformation_Matrix, Matrix);

        MODULE_TRAITS_AND_INFO(ModuleFlags::ModuleHasUI)

      private:
        const double HALF_SCALE_ = 0.5;

        void clearVals();
        void computeWidgetBox(const Core::Geometry::BBox& box);
        void buildGeometryObject();
        void updateState(FieldHandle field);
        void sendOutputPorts();
        void resetToInputField();
        void changeAxesOrientation(FieldHandle field);
        void setOutputCenter();
        void resetOutputCenter();
        void setOutputSize();
        void resetOutputSize();
        void processWidgetFeedback(const Core::Datatypes::ModuleFeedback& var);
        void adjustGeometryFromTransform(const Core::Geometry::Transform& feedbackTrans,
                                         const Core::Datatypes::WidgetMovement& movementType);
        void generateGeomsList();
        void saveToParameters();
        void loadFromParameters();
        std::string convertForLabel(double coord);
        void updateInputFieldAttributes();
        void setOutputParameters();

        Core::Geometry::BBox bbox_;
        Core::Geometry::Point pos_;
        std::vector<Core::Geometry::Vector> eigvecs_;
        Core::Geometry::Vector size_;
        FieldHandle outputField_;
        int widgetNum_{0};
        int resolution_ = 20;

        std::vector<Graphics::Datatypes::GeometryHandle> geoms_;
        Core::Geometry::Point ogPos_;
        Core::Geometry::Vector ogSize_;
        Core::Geometry::Transform inputFieldInverse_;
        Core::Geometry::Transform widgetScale_;
        Core::Geometry::Transform widgetRotation_;
        Core::Geometry::Point widgetTranslation_;
        bool widgetMoved_;
        bool widgetAxesRotated_;
        bool firstRun_;
      };
    }
  }
}

#endif
