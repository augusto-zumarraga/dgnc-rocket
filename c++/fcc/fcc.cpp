/*============================================================================*/
/*                                               Guiado, Navegación y Control */
/*============================================================================*/

////////////////////////////////////////////////////////////////////////////////
/// \brief
/// \author   Augusto Zumarraga
/// \date     creación: 27/06/2024
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
#include "fcc.hpp"

using namespace gnc;

//==============================================================================
flight_t::flight_t() : R_orbit(6570987), m_tm_eci(0)
{}
void flight_t::setup(pguid_t& p)
{
	p.setup(guid::params_t
		   ( params.exit_speed
		   , R_orbit
		   , times.sampling
		   , times.s2_burn
		   , params.pre_cycle
		   , params.tgo_min
		   , params.v_eps
		   , params.f_min));
}

//==============================================================================
fcc_t::fcc_t()
: fsm_t(state_init)
{
	m_tlmy.reset();
}
fcc_t& fcc_t::update_tlmy()
{
	m_tlmy.state = current_state_id();
	if(exo_phase())
		m_cntrl_s2.update(m_tlmy.ctl);
	else
		m_cntrl_s1.update(m_tlmy.ctl);
	return *this;
}
fcc_t& fcc_t::reset(const ins_data_t& s, e_states st, second_t t_launch, second_t t_sep)
{
	m_tlmy.reset();
	m_plan.start(t_launch);
	m_cntrl_s1.start(t_launch);
	m_cntrl_s2.start(t_sep   );
	switch(st)
	{
	case st_init         : set_state(state_init        , s); break;
	case st_armed        : set_state(state_armed       , s); break;
	case st_ascent       : set_state(state_ascent      , s); break;
	case st_load_relief  : set_state(state_load_relief , s); break;
	case st_meco         : set_state(state_meco        , s); break;
	case st_coasting     : set_state(state_coasting    , s); break;
	case st_separation   : set_state(state_separation  , s); break;
	case st_fire_s2      : set_state(state_fire_s2     , s); break;
	case st_gravity_turn : set_state(state_gravity_turn, s); break;
	case st_steering     : set_state(state_steering    , s); break;
	case st_low_thrust   : set_state(state_low_thrust  , s); break;
	case st_engine_off   : set_state(state_engine_off  , s); break;
	case st_orbit        : set_state(state_orbit       , s); break;
	}
	return *this;
}
// vuelo atmosférico (1ra etapa)
fcc_t::state_t fcc_t::state_init        (st_init        , &fcc_t::        init_on_timer);
fcc_t::state_t fcc_t::state_armed       (st_armed       , &fcc_t::       armed_on_timer, &fcc_t:: armed_on_entry);
fcc_t::state_t fcc_t::state_ascent      (st_ascent      , &fcc_t::      ascent_on_timer, &fcc_t::ascent_on_entry);
fcc_t::state_t fcc_t::state_load_relief (st_load_relief , &fcc_t:: load_relief_on_timer);
fcc_t::state_t fcc_t::state_meco        (st_meco        , &fcc_t::        meco_on_timer, &fcc_t::meco_on_entry, &fcc_t::meco_on_exit);
fcc_t::state_t fcc_t::state_coasting    (st_coasting    , &fcc_t::    coasting_on_timer, &fcc_t::coasting_on_entry);

// vuelo exo-atmosférico (2da etapa)
fcc_t::state_t fcc_t::state_separation  (st_separation  , &fcc_t::  separation_on_timer, &fcc_t::separation_on_entry);
fcc_t::state_t fcc_t::state_fire_s2     (st_fire_s2     , &fcc_t::     fire_s2_on_timer, &fcc_t::fire_s2_on_entry, &fcc_t::fire_s2_on_exit);
fcc_t::state_t fcc_t::state_gravity_turn(st_gravity_turn, &fcc_t::gravity_turn_on_timer);
fcc_t::state_t fcc_t::state_steering    (st_steering    , &fcc_t::    steering_on_timer, &fcc_t::  steering_on_entry);
fcc_t::state_t fcc_t::state_low_thrust  (st_low_thrust  , &fcc_t::  low_thrust_on_timer, &fcc_t::low_thrust_on_entry);
fcc_t::state_t fcc_t::state_engine_off  (st_engine_off  , &fcc_t::  engine_off_on_timer, &fcc_t::engine_off_on_entry);
fcc_t::state_t fcc_t::state_orbit       (st_orbit       , &fcc_t::       orbit_on_timer, &fcc_t::     orbit_on_entry);

//==============================================================================
void fcc_t::init_on_timer(const ins_data_t& s)
{

}

//------------------------------------------------------------------------------
void fcc_t::armed_on_entry(const ins_data_t& s)
{
	m_cmnd.reset();
	m_alarm.set(s.elapsed + second_t(10));
}
void fcc_t::armed_on_timer(const ins_data_t& s)
{
	m_cmnd.eng = m_alarm(s.elapsed);
	if(m_plan.s1_fired(s.sfc.x()))
		set_state(state_ascent, s);
}

//------------------------------------------------------------------------------
void fcc_t::ascent_on_entry(const ins_data_t& s)
{
	m_cntrl_s1.start(s.elapsed);
	m_plan.start(s.elapsed);
}
void fcc_t::ascent_on_timer(const ins_data_t& s)
{
	m_tlmy.q_ref = m_plan.wire(s.elapsed);
	att_t qwr = ecef_to_ned(s.pos, s.att);
	m_cntrl_s1.att_loop(m_tlmy.q_ref, qwr, s);
	if(m_plan.do_relief(s.lla.alt))
		set_state(state_load_relief, s);
}

//------------------------------------------------------------------------------
void fcc_t::load_relief_on_timer(const ins_data_t& s)
{
	m_tlmy.q_ref = m_plan.wire(s.elapsed);
	m_cntrl_s1.load_relief(m_tlmy.q_ref, s);
    if(m_plan.start_meco_maneuver(s.elapsed))
		set_state(state_meco, s);
}

//------------------------------------------------------------------------------
void fcc_t::meco_on_entry(const ins_data_t& s)
{
	m_cntrl_s1.stop_rotation();
	m_alarm.set(s.elapsed + m_plan.meco_advance());
	m_tlmy.reset(current_state_id());
}
void fcc_t::meco_on_timer(const ins_data_t& s)
{
	m_cntrl_s1.pqr_loop(s);
    if(m_alarm(s.elapsed))
		set_state(state_coasting, s);
}
void fcc_t::meco_on_exit(const ins_data_t& s)
{
	m_cmnd.eng = false;
}

//------------------------------------------------------------------------------
void fcc_t::coasting_on_entry(const ins_data_t& s)
{
	m_alarm.set(s.elapsed + m_plan.s1_coast_time());
	//m_cntrl_s1.stop_rotation();
}
void fcc_t::coasting_on_timer(const ins_data_t& s)
{
	m_cntrl_s1.roll_loop(s);
//	m_cntrl_s2.start(s.elapsed);
//	m_cntrl_s2.roll_loop(s);
//	m_cntrl_s1.cmnd().rc = m_cntrl_s2.cmnd().rc;
	if(m_alarm(s.elapsed) || m_plan.do_separation(s.lla.alt))
		set_state(state_separation, s);
}

//==============================================================================
void fcc_t::separation_on_entry(const ins_data_t& s)
{
	m_cntrl_s1.reset();
	m_cntrl_s2.reset();
	m_cmnd.sep = true;
	m_plan.start_separation(s.elapsed);
}
void fcc_t::separation_on_timer(const ins_data_t& s)
{
    if(m_plan.separation_completed(s.elapsed))
		set_state(state_fire_s2, s);
}
//------------------------------------------------------------------------------
void fcc_t::fire_s2_on_entry(const ins_data_t& s)
{
	m_tlmy.q_ref.reset();
	m_cntrl_s2.start(s.elapsed);
	m_alarm.set(s.elapsed + m_plan.s2_fire_delay());
}
void fcc_t::fire_s2_on_timer(const ins_data_t& s)
{
	m_cntrl_s2.roll_loop(s);
    if(m_alarm(s.elapsed))
    	m_cmnd.eng = true;
    if(m_plan.s2_fired(s.sfc.x()))
		set_state(m_plan.do_steering(s.lla.alt) ? state_steering : state_gravity_turn, s);
}
void fcc_t::fire_s2_on_exit(const ins_data_t& s)
{
	m_cmnd.eng = true;
	m_cntrl_s2.start(s.elapsed);
}

//------------------------------------------------------------------------------
void fcc_t::gravity_turn_on_timer(const ins_data_t& s)
{
	m_tlmy.gdn.pref = s.vel;
	m_tlmy.e_dir = s.att >> s.vel.as_any();

	m_cntrl_s2.pointing_loop(m_tlmy.e_dir, s);

//	if(m_release && m_plan.do_release(s.lla.alt))
//		m_cmnd.rel = true;
	if(m_plan.do_steering(s.lla.alt))
		set_state(state_steering, s);
}

//------------------------------------------------------------------------------
void fcc_t::steering_on_entry(const ins_data_t& s)
{
	m_plan.setup(m_pguid);
}
void fcc_t::steering_on_timer(const ins_data_t& s)
{
	guide_t::nav_t nv(s.nav());
	nv.vxp = nv.vel.as_any() ^ nv.pos.as_any();
	nv.sfc = nv.att << s.sfc;
	nv.Fs  = nv.sfc.norm();
	if(m_pguid.suspend(nv.Fs))
		set_state(state_low_thrust, s);
	else
	{
		m_pguid.on_time(nv);
		m_pguid.fill(m_tlmy.gdn);

		m_tlmy.e_dir = nv.att >> m_pguid.out().dir;
		m_cntrl_s2.pointing_loop(m_tlmy.e_dir, s);

		m_cmnd.rel = m_plan.do_release(s.lla.alt);
		if(m_pguid.seco())
			set_state(state_engine_off, s);
	}
}

//------------------------------------------------------------------------------
void fcc_t::engine_off_on_entry(const ins_data_t& s)
{
	m_cntrl_s2.stop_rotation();
	m_cmnd.eng = false;
	m_tlmy.e_dir = 0;
}
void fcc_t::engine_off_on_timer(const ins_data_t& s)
{
	m_cntrl_s2.roll_loop(s);
	if(s.sfc.norm() < 1)
		set_state(state_orbit, s);
}

//------------------------------------------------------------------------------
void fcc_t::low_thrust_on_entry(const ins_data_t& s)
{
	m_cntrl_s2.low_thrust();
}
void fcc_t::low_thrust_on_timer(const ins_data_t& s)
{
	m_cntrl_s2.roll_loop(s);
	if(m_pguid.resume(s.sfc.norm()))
		set_state(state_steering, s);
}

//==============================================================================
void fcc_t::orbit_on_entry(const ins_data_t& s)
{
	m_cntrl_s2.reset();
	m_cmnd.eng = false;
}
void fcc_t::orbit_on_timer(const ins_data_t& s)
{
	m_cntrl_s2.roll_loop(s);
}




