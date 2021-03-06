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


#ifndef MODULES_LEGACY_FIELDS_ConvertQuadSurfToTriSurf_H__
#define MODULES_LEGACY_FIELDS_ConvertQuadSurfToTriSurf_H__

#include <Dataflow/Network/Module.h>
#include <Modules/Legacy/Fields/share.h>

namespace SCIRun {
  namespace Modules {
    namespace Fields {

	/// @class ConvertQuadSurfToTriSurf
	/// @brief Convert a QuadSurfField into a TriSurfField.

      class SCISHARE ConvertQuadSurfToTriSurf : public Dataflow::Networks::Module,
        public Has1InputPort<FieldPortTag>,
        public Has1OutputPort<FieldPortTag>
      {
      public:
        ConvertQuadSurfToTriSurf();

        void execute() override;
        void setStateDefaults() override {}

        INPUT_PORT(0, QuadSurf, Field);
        OUTPUT_PORT(0, TriSurf, Field);

        MODULE_TRAITS_AND_INFO(ModuleFlags::ModuleHasAlgorithm)
      };

    }
  }
}

#endif
