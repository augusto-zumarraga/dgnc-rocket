/*============================================================================*/
/*                                                                            */
/*============================================================================*/
#pragma once
////////////////////////////////////////////////////////////////////////////////
/// \brief
/// \author   Augusto Zumarraga
/// \date     creación: 10/06/2024
/// \date     revisión: 21/08/2024 w x Jw
/// \date     revisión: 14/09/2024 flex
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

namespace dgnc { namespace fsim {

//------------------------------------------------------------------------------
inline rocket_sfc_t::state_t
rocket_sfc_t::state_t::operator*(double h) const
{
	state_t l;
	l.mass = mass * h;
	l.act  = act  * h;
	return l;
}
inline rocket_sfc_t::state_t
rocket_sfc_t::state_t::operator+(const state_t& r) const
{
	state_t l;
	l.mass = mass + r.mass;
	l.act  = act  + r.act;
	l.flx  = flx;
	return l;
}

//------------------------------------------------------------------------------
inline rocket_sfc_t::rocket_sfc_t(const char* mdl)
{
	char str[256];
	sprintf(str, "%s_aero.dat", mdl);
	m_aero.load(str);
	sprintf(str, "%s_mass.csv", mdl);
	m_mass.import(str);
	sprintf(str, "%s_prop.csv", mdl);
	m_prop.import(str);
	sprintf(str, "%s_flex.csv", mdl);
	m_flex.import(str);

	m_acts.set_propelents_mass(m_mass.propelents());
}

template <class S>
void rocket_sfc_t::init(S& xo, const ecef::state_t& st)
{
	static_cast<typename S::nav_t&>(xo)	= st;
	xo.wbi  = xo.att >> vector(0,0,wgs84::We);
	xo.mass = m_mass.initial();
	xo.act.init();
	xo.flx.clear();
}
template <class S>
void rocket_sfc_t::init(S& xo)
{
	xo.mass = m_mass.initial();
	xo.act.init();
	xo.flx.clear();
}

inline void set_wind(enviroment_t& s, const ecef::state_t& x, const wind_t& wind, const position_t& ref)
{
	s.lla = x.pos;
	s.air = rocket::air_t(s.lla.alt);
	velocity_t vb = x.vel.as_any();
	if(wind)
		vb += navs::ned_to_ecef(s.lla, wind(s.lla.alt)).as_any();
	vb = x.att >> vb;
	s.wnd = rocket::wind_t(s.air, vb, s.wbi, ref);
}
inline force_t ramp_reaction(const ecef::state_t& st)
{
	return -(st.att >> wgs84::gravity(st.pos));
}
inline force_t ramp_reaction(const eci::state_t& st)
{
	return -(st.att >> eci::gravity(st));
}

template <class S>
enviroment_t rocket_sfc_t::enviroment(double t, const S& x) const
{
	enviroment_t s;
	s.mass = m_mass(x.mass);
	s.wbi = x.wbi;
	position_t ref = s.mass.cg - vector(m_aero.xref,0,0);
	set_wind(s, x, m_wind, ref);

	aero_t::result_t aero = m_aero(s.wnd, ref, x.act.ail);
	prop_t::result_t prop = m_prop(x.act.flw, s.air.po, x.act, s.mass.cg);

	s.T    =  prop.force;
	s.mdot =  prop.mdot;
	s.fb   = (aero.force  + prop.force )/x.mass;
	s.Mb   =  aero.moment + prop.moment;
	if(m_rcsa)
	{
		rcsa_t::result_t rcs = m_rcsa(t, m_acts.rcs(), s.mass.cg);
		s.fb += rcs.force/x.mass;
		s.Mb += rcs.moment;
		s.rcs = m_acts.rcs().status();
	}
	else
		s.rcs = 0;
	if(s.lla.alt < 20 && s.T.x() < wgs84::go) // rampa de lanzamiento
		s.fb = ramp_reaction(x);
	return s;
}
template <class S>
inline void rocket_sfc_t::sample_begin(double t, const S& x, double ts)
{
	m_acts.update(t);
	m_sim.ins = x;
	m_sim.env = enviroment(t,x);
	m_acts.propelents_consumed(m_sim.env.mdot * ts);
	if(m_flex)
		m_sim.flx = m_flex.get_sens(x.flx);
}
template <class S>
inline void rocket_sfc_t::sample_end(double t, const S& x, double ts)
{

}
template <class S>
rocket_sfc_t::rates_t rocket_sfc_t::compute(double t, const S& x) const
{
	using namespace rocket;
	enviroment_t env = enviroment(t, x);
	rates_t r;
	r.J  = env.mass.J;
	r.Ji = env.mass.Ji;
	r.fb = env.fb.as_any();
	r.Mb = env.Mb.as_any();
	r.sr.mass = -env.mdot;
	r.sr.act  = m_acts.rates(x.act);
	return r;
}


//------------------------------------------------------------------------------
/// Se ejecuta al comenzar un paso de integración
inline void dsicrete_states_t::step_begin(double ti, const rocket_mdl_t& mdl, rocket_mdl_t::vect_t& x, double h)
{
	if(mdl.m_flex)
		x.flx = mdl.m_flex.integrate(x.flx, x.mass, mdl.m_sim.env.wnd.vel, mdl.m_sim.env.wnd.wb, mdl.m_sim.env.T, h);
}
inline void dsicrete_states_t::step_end(double tf, const rocket_mdl_t& mdl, rocket_mdl_t::vect_t& x, double)
{
	math::sat(x.act.dy , 1);
	math::sat(x.act.dz , 1);
	math::sat(x.act.ail, 1);
	math::sat(x.act.flw, 1, 0);
	x.act.rcs = mdl.m_sim.env.rcs;
}

}}


