/*
   For more information, please see: http://software.sci.utah.edu

   The MIT License

   Copyright (c) 2012 Scientific Computing and Imaging Institute,
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


#ifndef CORE_DATATYPES_SCALAR_H
#define CORE_DATATYPES_SCALAR_H 

#include <Core/Datatypes/Datatype.h>
#include <Core/Datatypes/Share.h>

namespace SCIRun {
namespace Domain {
namespace Datatypes {

//TODO DAN
  class SCISHARE Scalar : public Datatype
  {

  };

  class SCISHARE Int32 : public Scalar
  {
  public:
    explicit Int32(int val);
    int getValue() const;
  private:
    int val_;
  };

  class SCISHARE UInt32 : public Scalar
  {

  };

  class SCISHARE Int64 : public Scalar
  {

  };

  class SCISHARE UInt64 : public Scalar
  {

  };

  class SCISHARE Float : public Scalar
  {

  };

  class SCISHARE Double : public Scalar
  {

  };


}}}


#endif
