/*============================================================================*/
/*                                                                            */
/*============================================================================*/
#pragma once
////////////////////////////////////////////////////////////////////////////////
/// \brief
/// \author   Augusto Zumarraga
/// \date     creación: 10/06/2024
/// \date     revisión: 04/09/2024
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
#include <dgnc/rocket/data/aero.hpp>
#include <dgnc/rocket/data/mass.hpp>
#include <dgnc/rocket/data/prop.hpp>
#include <dgnc/rocket/data/rcs.hpp>
#include <dgnc/rocket/dyn/aero_elastic.hpp>
#include <dgnc/rocket/dyn/rigid_body.hpp>

namespace dgnc { namespace fsim {

namespace enu   = navs::enu;
namespace ned   = navs::ned;
namespace eci   = navs::eci;
namespace ecef  = navs::ecef;
namespace wgs84 = navs::wgs84;

using geom::radian;
using geom::degree;
using geom::scalar;
using geom::vector;
using geom::direction;
using geom::quaternion;

using rocket::position_t  ;
using rocket::velocity_t  ;
using rocket::angle_rate_t;
using rocket::force_t     ;
using rocket::moment_t    ;
using rocket::tensor_t    ;
using rocket::mass_prop_t ;
using rocket::air_t       ;
//using rocket::wind_t      ;

typedef rocket::aero::model aero_t;
typedef rocket::inertia_t   mass_t;
typedef rocket::engine_t    prop_t;
typedef rocket::rcs_t       rcsa_t;
typedef rocket::struct_t    flex_t;

//------------------------------------------------------------------------------
struct enviroment_t
{
	force_t        fb ;  // fuerza específica en terna b
	moment_t       Mb ;  // momento resultante en terna b
	ecef::lla_t    lla;
	angle_rate_t   wbi;  // velocidad angular en terna b
	air_t          air;  // parámetros del aire
	rocket::wind_t wnd;  // parámetros del flujo de aire
	mass_prop_t    mass; // propiedades másicas
	force_t        T;    // empuje
	scalar         mdot; // flujo de combustible
	scalar         rcs ;
};
struct sim_info_t
{
	enviroment_t   env;
	ecef::state_t  ins;
	flex_t::sens_t flx;  // estado elástico
	scalar         rcs;
};
typedef std::vector<sim_info_t> sim_record_t;

}}



