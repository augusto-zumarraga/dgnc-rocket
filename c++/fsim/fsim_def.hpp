/*============================================================================*/
/*                                                                            */
/*============================================================================*/
#pragma once
////////////////////////////////////////////////////////////////////////////////
/// \brief
/// \author   Augusto Zumarraga
/// \date     creación: 20/08/2024
/// \date     revisión: 20/08/2024
//______________________________________________________________________________

/*
    Copyright (c) 2024 Augusto Zumarraga

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to
    deal in the Software without restriction, including without limitation the
    rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
    sell copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
    FROM,OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
    IN THE SOFTWARE.
*/

#include <iostream>
#include <fstream>
#include <dgnc/geom.hpp>
#include "mdl/mdl.hpp"
#include "load/loader.hpp"

namespace dgnc { namespace fsim {

using gnc::fcc_t;
using gnc::ins_data_t;
typedef std::vector<fcc_t::tlmy_t> fcc_record_t;

}}
