/*============================================================================*/
/*                                               Guiado, Navegación y Control */
/*============================================================================*/

////////////////////////////////////////////////////////////////////////////////
/// \brief    CONFIGURATION GUIDANCE FOR PICO LAUNCHERS (API-PGUID)
///           Algoritmos
/// \author   Alberto Fraguío
/// \author   Augusto Zumarraga
/// \date     creación: 13/08/2024
/// \date     revisión: 13/08/2024
//______________________________________________________________________________

/*
    Copyright (c) 2024 Alberto Fraguío

	Pguid is free software: you can redistribute it and/or modify it under the
	terms of the GNU General Public License as published by the Free Software
	Foundation, either version 3 of the License, or (at your option) any later
	version.

	Pguid is distributed in the hope that it will be useful, but WITHOUT ANY
	WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
	A PARTICULAR PURPOSE. See the GNU General Public License for more details.

	You should have received a copy of the GNU General Public License along with
    Pguid. If not, see <https://www.gnu.org/licenses/>.
*/
#include "pguid_term.hpp"
#include "pguid_ltg.hpp"

using namespace gnc;
using namespace guid;

//==============================================================================
void term_t::params_t::operator=(const guid::params_t& p)
{
	Tg       = p.ts;
	VD       = p.VD;
	Vepsilon = p.Veps;
}
void term_t::fill(tlmy_t& s)
{

}
void term_t::init(const nav_t& nv, const ltg_t& ltg)
{
	ti    = ltg.m_res.t0;
	tf    = nv.elapsed + ltg.m_state.ttgo;
	Tg    = params.Tg;
	Vmiss = std::abs(nv.vel.norm() - params.VD);
	ulambda_I = ltg.m_res.ulambda_I;
	m_out     = ltg.m_res.out;
}
bool term_t::update(const nav_t& nv)
{
	if(nv.elapsed >= tf)
		return true;

	Tg = nv.elapsed + params.Tg > tf ? tf - nv.elapsed : params.Tg;

	vector vtildeBI_I = nv.vel.as_any() + nv.sfc.as_any() * Tg;
	scalar VtildeBI   = vtildeBI_I.norm();

	m_out.dir = ulambda_I + scalar(nv.elapsed - ti) * m_out.rate;
	Vmiss = params.VD - VtildeBI;

	return Vmiss <= params.Vepsilon;
}


