/*============================================================================*/
/*                                                                            */
/*============================================================================*/

////////////////////////////////////////////////////////////////////////////////
/// \brief
/// \author   Augusto Zumarraga
/// \date     creación: 04/09/2024
/// \date     revisión: 27/09/2024
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
#include "mdl.hpp"

using namespace dgnc;
using namespace fsim;

actuators_t::x_rsc_t::x_rsc_t()
: t_cycle (0)
, t_fire_p(0)
, t_fire_n(0)
, cmnd(0)
{}
void actuators_t::x_rsc_t::setup(second_t t_cyc)
{
	t_cycle = t_cyc;
	fire_times.resize(4);
}
void actuators_t::x_rsc_t::update(second_t tm)
{
	if(!t_cycle)
		return;
	if(t_fire_p)
	{
		if(t_fire_p + t_cycle < tm)
		{
			t_fire_p = 0;
			fire_times[0] = 0;
			fire_times[1] = 0;
		}
	}
	if(t_fire_n)
	{
		assert(!t_fire_p);
		if(t_fire_n + t_cycle < tm)
		{
			t_fire_n = 0;
			fire_times[2] = 0;
			fire_times[3] = 0;
		}
	}
	if(!t_fire_p && !t_fire_n)
	{
		if(cmnd > 0)
		{
			t_fire_p = tm;
			fire_times[0] = tm;
			fire_times[1] = tm;
		}
		else
		if(cmnd < 0)
		{
			t_fire_n = tm;
			fire_times[2] = tm;
			fire_times[3] = tm;
		}
	}
}

//------------------------------------------------------------------------------
void rocket_sfc_t::add_roll_fin()
{
	rocket::aero::roll_finset_t::params_t finset =
	{ (0.17+0.1)/2*0.14    // superficie de la aleta
	, 0.14                 // envergadura de la aleta
	, (0.14+0.35)/2        // radio hasta la CAM
	, 2                    // cantidad de aletas
	, radian(degree(20))   // δ máx
	};
	m_aero.setup(finset);
}
void rocket_sfc_t::add_roll_rcs()
{
	double z = 0.35/2;

	vector    pos(1,0,0);
	direction axs(0,1,0);
	scalar    thrust(0.2);
	second_t  t_burn(0.13);
	second_t  t_decy(0.02);
	second_t  t_cycl(0.2);

	m_rcsa.axes.resize(4);

	// +p
	pos.z() = -z;
	axs.y() =  1;
	m_rcsa.axes[0] = rcsa_t::axis_t(pos, axs, thrust, t_burn, t_decy, t_cycl);
	pos.z() =  z;
	axs.y() = -1;
	m_rcsa.axes[1] = rcsa_t::axis_t(pos, axs, thrust, t_burn, t_decy, t_cycl);

	// -p
	pos.z() = -z;
	axs.y() = -1;
	m_rcsa.axes[2] = rcsa_t::axis_t(pos, axs, thrust, t_burn, t_decy, t_cycl);
	pos.z() =  z;
	axs.y() =  1;
	m_rcsa.axes[3] = rcsa_t::axis_t(pos, axs, thrust, t_burn, t_decy, t_cycl);

	m_acts.rcs().setup(t_cycl);
}

