/*============================================================================*/
/*                                                                            */
/*============================================================================*/
#pragma once
////////////////////////////////////////////////////////////////////////////////
/// \brief
/// \author   Augusto Zumarraga
/// \date     creación: 10/06/2024
/// \date     revisión: 18/08/2024 flex
/// \date     revisión: 03/09/2024 RCS
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
#include "mdl_def.hpp"
#include <dgnc/rocket/dyn/rigid_body.hpp>
#include <dgnc/numeric/ode.hpp>
#include "wind.hpp"

namespace dgnc { namespace fsim {

//------------------------------------------------------------------------------
class actuators_t
{
public:

	struct state_t : public rocket::tvc_state_t
	{
		float ail; // valores normalizados [±1]
		float flw; // valores normalizados [±1]
		float rcs;
		state_t operator*(double h) const;
		state_t operator+(const state_t& r) const;
		void init();
	};
	state_t rates(state_t) const;

	//--------------------------------------------------------------------------
	//                                                              Actuadores
	//--------------------------------------------------------------------------
	struct servo_t
	{
		float bnd; // bandwidth
		float res; // resolution
		float cmd; // command
		float off; // command
		servo_t(float w = 100, float r = 1e-3)
		: bnd(w)
		, res(r)
		, cmd(0)
		, off(0)
		{}
		void  set (float c);
		float rate(float pos) const;
	};
	struct flow_t : public servo_t
	{
		double fuel, fmin;
		flow_t() : servo_t(20), fuel(100), fmin(0.5)
		{}
		void set(float c)
		{
			servo_t::set(fuel > fmin ? c : 0);
		}
		void operator-=(scalar dm)
		{
			if(fuel > dm)
				fuel -= dm;
			else
			{
				fuel  = 0;
				servo_t::set(0);
			}
		}
	};
	struct x_rsc_t
	{
		second_t t_cycle, t_fire_p, t_fire_n;
		int cmnd;
		rcsa_t::fires_t fire_times;

		x_rsc_t();
		void setup(second_t t_cyc);
		void update(second_t tm);
		void set(float c, unsigned k = 0)
		{
			cmnd = c;
		}
		operator const rcsa_t::fires_t&() const
		{
			return fire_times;
		}
		int status() const
		{
			return t_fire_p ? 1 : (t_fire_n ? -1 : 0);
		}
	};

	//--------------------------------------------------------------------------
	// Comandos
	//--------------------------------------------------------------------------
	actuators_t& tvc(float dy, float dz)
	{
		m_tvc_y.set(dy);
		m_tvc_z.set(dz);
		return *this;
	}
	actuators_t& ail(float da)
	{
		m_ail  .set(da);
		return *this;
	}
	actuators_t& rcs(float da)
	{
		m_rcs_x.set(da);
		return *this;
	}
	actuators_t& eng(bool b)
	{
		m_flow.set(b ? 1 : 0);
		return *this;
	}
	const x_rsc_t& rcs() const
	{
		return m_rcs_x;
	}
	x_rsc_t& rcs()
	{
		return m_rcs_x;
	}

	actuators_t();

	void update(second_t t)
	{
		m_rcs_x.update(t);
	}

protected:

	friend class rocket_sfc_t;

	void set_propelents_mass(scalar m)
	{
		m_flow.fuel = m;
	}
	void propelents_consumed(scalar dm)
	{
		m_flow -= dm;
	}
	servo_t m_tvc_y;
	servo_t m_tvc_z;
	servo_t m_ail;
	 flow_t m_flow;
	x_rsc_t m_rcs_x;
};

//------------------------------------------------------------------------------
inline void actuators_t::servo_t::set(float c)
{
	if(std::abs(c) > 1)
		cmd = math::sgn(c);
	else
		cmd = floor(c/res + 0.5)*res;
	cmd += off;
}
inline float actuators_t::servo_t::rate(float pos) const
{
	return (cmd - pos)*bnd;
}

inline actuators_t::actuators_t()
: m_tvc_y(50)
, m_tvc_z(50)
, m_ail  (65)
{}
inline actuators_t::state_t
actuators_t::rates(state_t act) const
{
	act.dy  = m_tvc_y.rate(act.dy );
	act.dz  = m_tvc_z.rate(act.dz );
	act.ail = m_ail  .rate(act.ail);
	act.flw = m_flow .rate(act.flw);
	return act;
};


//------------------------------------------------------------------------------
inline void actuators_t::state_t::init()
{
	dy  = 0;
	dz  = 0;
	ail = 0;
	flw = 0;
	rcs = 0;
}
inline actuators_t::state_t actuators_t::state_t::operator*(double h) const
{
	state_t l = *this;
	l.dy  *= h;
	l.dz  *= h;
	l.ail *= h;
	l.flw *= h;
	return l;
}
inline actuators_t::state_t actuators_t::state_t::operator+(const state_t& r) const
{
	state_t l = *this;
	l.dy  += r.dy ;
	l.dz  += r.dz ;
	l.ail += r.ail;
	l.flw += r.flw;
	return l;
}

}}

inline std::ostream& operator<<(std::ostream& s, const dgnc::fsim::actuators_t::state_t& x)
{
	s << x.dy << ',' << x.dz << ',' << x.ail << ',' << x.rcs << ',' << x.flw;
	return s;
}




