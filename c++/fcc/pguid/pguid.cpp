/*============================================================================*/
/*                                               Guiado, Navegación y Control */
/*============================================================================*/

////////////////////////////////////////////////////////////////////////////////
/// \brief    CONFIGURATION GUIDANCE FOR PICO LAUNCHERS (API-PGUID)
///           Máquina de Estados
/// \author   Alberto Fraguío
/// \author   Augusto Zumarraga
/// \date     creación: 13/08/2024
/// \date     revisión: 14/08/2024
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
#include "pguid.hpp"

using namespace gnc;

//==============================================================================
pguid_t::pguid_t()
: fsm_t(state_init)
{}
void pguid_t::setup(const params_t& p)
{
	m_ltg.params = p;
	m_trm.params = p;
}
void pguid_t::set_gravity_turn(const nav_t& nv)
{
	m_out.dir  = nv.vel;
	m_out.rate = 0;
}

pguid_t::state_t pguid_t::state_init       (st_init       , &pguid_t::       init_on_timer);
pguid_t::state_t pguid_t::state_pre_thrust (st_pre_thrust , &pguid_t:: pre_thrust_on_timer, &pguid_t:: pre_thrust_on_entry);
pguid_t::state_t pguid_t::state_pre_cycling(st_pre_cycling, &pguid_t::pre_cycling_on_timer, &pguid_t::pre_cycling_on_entry);
pguid_t::state_t pguid_t::state_ltg        (st_ltg        , &pguid_t::        ltg_on_timer);
pguid_t::state_t pguid_t::state_terminal   (st_terminal   , &pguid_t::   terminal_on_timer, &pguid_t::terminal_on_entry);
pguid_t::state_t pguid_t::state_orbit      (st_orbit      , &pguid_t::      orbit_on_timer);

//==============================================================================
void pguid_t::init_on_timer(const nav_t& nv)
{
	set_gravity_turn(nv);
	set_state(state_pre_thrust, nv);
}

//------------------------------------------------------------------------------
void pguid_t::pre_thrust_on_entry(const nav_t& nv)
{

}
void pguid_t::pre_thrust_on_timer(const nav_t& nv)
{
	set_gravity_turn(nv);
	if(m_ltg.go(nv))
	{
		double f = (nv.sfc * m_out.dir)/nv.Fs;
		if(std::abs(1-f) < math::d2r_d)
			set_state(state_pre_cycling, nv);
	}
}

//------------------------------------------------------------------------------
void pguid_t::pre_cycling_on_entry(const nav_t& nv)
{
	m_ltg.init(nv);
	m_alarm.set(nv.elapsed + m_ltg.params.cycle_time);
}
void pguid_t::pre_cycling_on_timer(const nav_t& nv)
{
	set_gravity_turn(nv);
	m_ltg.update(nv);
    if(m_alarm(nv.elapsed))
		set_state(state_ltg, nv);
}

//------------------------------------------------------------------------------
void pguid_t::ltg_on_timer(const nav_t& nv)
{
	if(m_ltg.update(nv))
		set_state(state_terminal, nv);
	m_out = m_ltg.out();
}

//------------------------------------------------------------------------------
void pguid_t::terminal_on_entry(const nav_t& nv)
{
	m_trm.init(nv, m_ltg);
}
void pguid_t::terminal_on_timer(const nav_t& nv)
{
	if(m_trm.update(nv))
    	set_state(state_orbit, nv);
	m_out = m_trm.out();
}

//------------------------------------------------------------------------------
void pguid_t::orbit_on_timer(const nav_t& s)
{

}



