/*============================================================================*/
/*                                               Guiado, Navegación y Control */
/*============================================================================*/

////////////////////////////////////////////////////////////////////////////////
/// \brief    CONFIGURATION GUIDANCE FOR PICO LAUNCHERS (API-PGUID)
/// \author   Alberto Fraguío
/// \author   Augusto Zumarraga (C++ coding)
/// \date     creación: 13/08/2024
/// \date     revisión: 05/09/2024
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
#pragma once

#include "pguid_term.hpp"
#include "pguid_ltg.hpp"

namespace gnc {

//------------------------------------------------------------------------------
/// Máquina de Estados Finitos de la navegación activa
//------------------------------------------------------------------------------
class pguid_t : protected patterns::t_finite_state_machine<pguid_t, const guid::nav_t>
{
public:

	typedef guid::nav_t    nav_t;
	typedef guid::out_t    out_t;
	typedef guid::tlmy_t   tlmy_t;
	typedef guid::params_t params_t;

	enum e_states
	{ st_init = 0
    , st_pre_thrust
	, st_pre_cycling
	, st_ltg
	, st_terminal
	, st_orbit
	};

	pguid_t();
	pguid_t& on_time(const nav_t& nv)
	{
		on_event(nv);
		return *this;
	}
	void setup(const params_t&);
	void fill (tlmy_t& t)
	{
		t.reset(current_state_id());
		t.pref  = m_out.dir;
		switch(t.state)
		{
		case st_ltg     : m_ltg.fill(t); break;
		case st_terminal: m_trm.fill(t); break;
		}
	}

	const out_t& out() const { return m_out; }
	bool        seco() const { return state() >= st_orbit; }
	e_states   state() const { return e_states(current_state_id()); }
	void       setup(scalar Ve, scalar rD, second_t ts, second_t pc, second_t tburn, second_t tgomin, scalar Fsmin = 0.1);
	bool     suspend(scalar Fs) const { return Fs <= m_ltg.params.Fsmin; }
	bool      resume(scalar Fs) const { return Fs >  m_ltg.params.Fsmin * 1.1; }

private:

	out_t        m_out;
	alarm_t      m_alarm;
	guid:: ltg_t m_ltg;
	guid::term_t m_trm;

	void set_gravity_turn(const nav_t&);

	typedef patterns::t_finite_state_machine<pguid_t, const nav_t> fsm_t;
	typedef fsm_t::pstate_t pstate_t;
	typedef fsm_t:: state_t  state_t;
	friend  fsm_t;

	static state_t state_init       ;
	static state_t state_pre_thrust ;
	static state_t state_pre_cycling;
	static state_t state_ltg        ;
	static state_t state_terminal   ;
	static state_t state_orbit      ;

	void        init_on_timer(const nav_t&);
	void  pre_thrust_on_entry(const nav_t&);
	void  pre_thrust_on_timer(const nav_t&);
	void pre_cycling_on_entry(const nav_t&);
	void pre_cycling_on_timer(const nav_t&);
	void         ltg_on_timer(const nav_t&);
	void    terminal_on_entry(const nav_t&);
	void    terminal_on_timer(const nav_t&);
	void       orbit_on_timer(const nav_t&);
};

}


