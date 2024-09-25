/*============================================================================*/
/*                                                                            */
/*============================================================================*/

////////////////////////////////////////////////////////////////////////////////
/// \brief
/// \author   Augusto Zumarraga
/// \date     creación: 10/06/2024
/// \date     revisión: 05/09/2024
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
#include <dgnc/navs/wgs84.hpp>
#include <dgnc/navs/eci.hpp>
#include <dgnc/patterns/fsm.hpp>

namespace gnc {

namespace enu   = dgnc::navs::enu;
namespace ned   = dgnc::navs::ned;
namespace loc   = dgnc::navs::loc;
namespace eci   = dgnc::navs::eci;
namespace ecef  = dgnc::navs::ecef;
namespace wgs84 = dgnc::navs::wgs84;
namespace math  = dgnc::math;

using dgnc::geom::radian;
using dgnc::geom::degree;
using dgnc::geom::vector;
using dgnc::geom::direction;
using dgnc::geom::quaternion;
using dgnc::geom::euler;
using dgnc::geom::t_matrix;
using dgnc::geom::dcm;
using dgnc::geom::scalar;
using dgnc::geom::polar;
using dgnc::geom::incidence;

using dgnc::second_t;

using dgnc::navs::position_t  ;
using dgnc::navs::velocity_t  ;
using dgnc::navs::angle_rate_t;
using dgnc::navs::force_t     ;
using dgnc::navs::moment_t    ;

typedef ned::att_t att_t;

namespace patterns = dgnc::patterns;

constexpr auto min_ts = 0.01;        // tiempo de muestreo
constexpr auto     t1 = 150 ;        // tiempo de vuelo etapa 1
constexpr auto     t2 = 400 ;        // tiempo de vuelo etapa 2
constexpr unsigned N1 = t1/min_ts;   // muestreo etapa 1
constexpr unsigned N2 = t2/min_ts;   // muestreo etapa 2
constexpr unsigned NM = t2/min_ts;   // muestreo etapa 2

//------------------------------------------------------------------------------
class alarm_t
{
public:

	alarm_t() : m_tm(0)
	{}
    void set(second_t tm)
    {
    	m_tm = tm;
    }
    bool expired(second_t tm) const
    {
    	return tm >= m_tm;
    }
    bool operator()(second_t tm) const
    {
    	return expired(tm);
    }
    second_t remaining(second_t tm) const
    {
    	return m_tm - tm;
    }

protected:

    second_t m_tm;
};

}

#include "sns.hpp"
#include "cmd.hpp"
#include "sched.hpp"


