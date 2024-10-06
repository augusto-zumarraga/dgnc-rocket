/*============================================================================*/
/*                                               Guiado, Navegación y Control */
/*============================================================================*/

////////////////////////////////////////////////////////////////////////////////
/// \brief
/// \author   Augusto Zumarraga
/// \date     creación: 27/06/2024
/// \date     revisión: 17/07/2024
/// \date     revisión: 14/08/2024 PGUID
/// \date     revisión: 25/09/2024
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

#include "plan.hpp"
#include "cntrl/cntrl.hpp"
#include "pguid/pguid.hpp"

namespace gnc {

class fcc_loader;

//------------------------------------------------------------------------------
class fcc_t : protected patterns::t_finite_state_machine<fcc_t, const ins_data_t>
{
public:

	typedef pguid_t     guide_t;
	typedef ctrl::atm_t c_atm_t;
	typedef ctrl::exo_t c_exo_t;

	enum e_states
	// primera etapa
	{ st_init = 0
    , st_armed
	, st_ascent
	, st_load_relief
	, st_meco
	, st_coasting
	// segunda etapa
	, st_atm = st_coasting
	, st_separation
	, st_fire_s2
	, st_gravity_turn
	, st_steering
	, st_low_thrust
	, st_engine_off
	, st_orbit
	};
	static constexpr auto st_last = st_orbit + 1;

	struct tlmy_t
	{
		int             state;
		quaternion      q_ref;
		direction       e_dir;
		ctrl::tlmy_t    ctl;
		guide_t::tlmy_t gdn;

		void reset(int st = 0)
		{
			state = st;
			q_ref.reset();
			e_dir = 0;
			ctl.reset();
			gdn.reset();
		}
	};

	fcc_t();

	//--------------------------------------------------------------------------
	fcc_t& arm(const ins_data_t& ins)
	{
		if(current_state_id() == st_init)
			set_state(state_armed, ins);
		return *this;
	}
	fcc_t& abort(const ins_data_t& ins)
	{
		if(current_state_id() == st_armed)
			set_state(state_init, ins);
		return *this;
	}
	fcc_t& on_time(const ins_data_t& ins)
	{
		on_event(ins);
		return *this;
	}
	fcc_t& update_tlmy();
	fcc_t& reset(const ins_data_t&, e_states st, second_t t_launch, second_t t_sep);

	//--------------------------------------------------------------------------
	int state_trace() const
	{
		return current_state_id() == st_steering
			 ? int(m_pguid.state()) + st_last :
			   current_state_id();
	}
	bool exo_phase() const
	{
		return current_state_id() > st_atm;
	}
	second_t time_to_launch(second_t t) const
	{
		return current_state_id() == st_armed ? m_alarm.remaining(t) : second_t(0);
	}
	const cmnds::cont_t& AOs () const { return exo_phase() ? m_cntrl_s2.cmnd() : m_cntrl_s1.cmnd(); }
    const cmnds::bool_t& DOs () const { return m_cmnd; }
	const        tlmy_t& tlmy() const { return m_tlmy; }
	const      flight_t& plan() const { return m_plan; }

protected:

	friend class fcc_loader_t;
	c_atm_t& ctrl_s1() { return m_cntrl_s1; }
	c_exo_t& ctrl_s2() { return m_cntrl_s2; }
    flight_t& plan  () { return m_plan; }

private:

	flight_t m_plan;
	alarm_t  m_alarm;
	guide_t  m_pguid;
	c_atm_t  m_cntrl_s1;
	c_exo_t  m_cntrl_s2;
	tlmy_t   m_tlmy;

	cmnds::bool_t m_cmnd;

	typedef patterns::t_finite_state_machine<fcc_t, const ins_data_t> fsm_t;
	typedef fsm_t::pstate_t pstate_t;
	typedef fsm_t:: state_t  state_t;
	friend  fsm_t;

	static state_t state_init;
	static state_t state_armed;
	static state_t state_ascent;
	static state_t state_load_relief;
	static state_t state_meco;
	static state_t state_separation;
	static state_t state_fire_s2;
	static state_t state_gravity_turn;
	static state_t state_steering;
	static state_t state_low_thrust;
	static state_t state_coasting;
	static state_t state_engine_off;
	static state_t state_orbit;

	void         init_on_timer(const ins_data_t&);
	void        armed_on_entry(const ins_data_t&);
	void        armed_on_timer(const ins_data_t&);
	void       ascent_on_entry(const ins_data_t&);
	void       ascent_on_timer(const ins_data_t&);
	void  load_relief_on_timer(const ins_data_t&);
	void         meco_on_entry(const ins_data_t&);
	void         meco_on_timer(const ins_data_t&);
	void         meco_on_exit (const ins_data_t&);
	void   separation_on_entry(const ins_data_t&);
	void   separation_on_timer(const ins_data_t&);
	void      fire_s2_on_entry(const ins_data_t&);
	void      fire_s2_on_timer(const ins_data_t&);
	void      fire_s2_on_exit (const ins_data_t&);
	void gravity_turn_on_timer(const ins_data_t&);
	void     steering_on_entry(const ins_data_t&);
	void     steering_on_timer(const ins_data_t&);
	void   low_thrust_on_entry(const ins_data_t&);
	void   low_thrust_on_timer(const ins_data_t&);
	void     coasting_on_entry(const ins_data_t&);
	void     coasting_on_timer(const ins_data_t&);
	void   engine_off_on_entry(const ins_data_t&);
	void   engine_off_on_timer(const ins_data_t&);
	void        orbit_on_entry(const ins_data_t&);
	void        orbit_on_timer(const ins_data_t&);
};

}


