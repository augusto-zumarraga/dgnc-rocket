/*============================================================================*/
/*                                                                            */
/*============================================================================*/

////////////////////////////////////////////////////////////////////////////////
/// \brief
/// \author   Augusto Zumarraga
/// \date     creación: 26/06/2024
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
#pragma once

#include <string>
#include <vector>
#include <stdexcept>
#include <cmath>

#include <dgnc/data/data.hpp>
#include <dgnc/navs/navs_def.hpp>
#include <dgnc/numeric/interp.hpp>
#include <dgnc/store/data_file.hpp>
#include <dgnc/store/csv_file.hpp>

namespace dgnc {

namespace data {
class file_t;
}

namespace rocket {

using std::string;
using data::file_t;
using std::runtime_error;
using std::logic_error;
using numeric::interp::spline_2_t;
using numeric::interp::spline_1_t;
using numeric::interp::spline_1;
using numeric::interp::t_splines_1;
using math::t_matrix;

typedef geom::scalar scalar;
typedef std::vector<scalar> values_t;
typedef data::t_range<scalar, values_t> range_t;

typedef data::col  aoa;
typedef data::row  slp;
typedef data::blk  mch;
typedef data::cols aoas;
typedef data::rows slps;
typedef data::blks mchs;

typedef geom::radian     radian;
typedef geom::degree     degree;
typedef geom::vector     vector;
typedef geom::direction  direction;
typedef geom::quaternion quaternion;

typedef navs::distance_t   distance_t;
typedef navs::position_t   position_t;
typedef navs::velocity_t   velocity_t;
typedef navs::angle_rate_t angle_rate_t;
typedef navs::force_t      force_t;
typedef navs::moment_t     moment_t;
typedef t_matrix<3,3>      tensor_t;

struct mass_prop_t
{
	scalar   m;
	vector   cg;
	tensor_t J ;
	tensor_t Ji;
};


struct tvc_state_t
{
	float dy; // valores normalizados[±1]
	float dz;
	tvc_state_t(float y, float z)
	: dy(y), dz(z)
	{}
	tvc_state_t()
	: dy(0), dz(0)
	{}
};


}}
#include "air.hpp"

