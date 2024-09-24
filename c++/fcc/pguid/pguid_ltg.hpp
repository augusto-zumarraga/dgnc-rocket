/*============================================================================*/
/*                                               Guiado, Navegación y Control */
/*============================================================================*/

////////////////////////////////////////////////////////////////////////////////
/// \brief    CONFIGURATION GUIDANCE FOR PICO LAUNCHERS (API-PGUID)
/// \author   Alberto Fraguío
/// \author   Augusto Zumarraga (C++ coding)
/// \date     creación: 13/08/2024
/// \date     revisión: 31/08/2024
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

#include "pguid_def.hpp"

namespace gnc { namespace guid {

//------------------------------------------------------------------------------
/// Navegación LTG
//------------------------------------------------------------------------------
struct ltg_t
{
	struct params_t
	{
		second_t Tg        ;  // [s]       LTG sampling time (please, don't change before consulting)
		second_t tgomin    ;  // [s]       LTG stop criteria (must be >= 4s)
		second_t tburn     ;  // [s]       Maximum available burn time
		second_t cycle_time;  // [-]       Required converging intervals (please, don't change before consulting)

		scalar   Ve        ;  // [m/s]     exhaust exit velocity
		scalar   rD        ;  // [m]
		scalar   VD        ;  // [m/s]     Rapidez objetivo
		scalar   Fsmin     ;  // [m/s^2]   Specific force magnituds less thant Fsmin freeze LTG program.

		void operator=(const guid::params_t&);

	} params;

	bool go(const nav_t& nv) const
	{
		return nv.Fs > params.Fsmin;
	}
	bool done() const
	{
		return m_state.ttgo < params.tgomin;
	}
	const out_t& out() const
	{
		return m_res.out;
	}
	void fill  (tlmy_t&);
	void init  (const nav_t&);
	///  \return true if is done
	bool update(const nav_t&);


protected:

	// state
	struct state_t
	{
		vector    rbias_I;
		vector    rgrav_I;
		vector    vgo_I  ;
		vector    sDI_I  ;
		direction uZ_I   ;
		second_t  ttgo   ;

	} m_state;

	// results
	struct result_t
	{
		direction ulambda_I;
		second_t  t0  ;
		scalar    rtgo;
		out_t     out ;

	} m_res;

	friend class term_t;

	struct tgo_t { scalar tgo, tau, L; };
	struct itg_t { scalar S, J, Q, P, H; };
	struct kpl_t { vector pos, vel; };
	struct prd_t { vector pos, vel, rth, rgv; };

	tgo_t  time_to_go (scalar Fs) const;
	itg_t  integrals  (const tgo_t&) const;
	kpl_t  keppler    (const vector&, const vector&, double tgo) const;
	vector range_to_go(const tgo_t& t, const itg_t& i, const nav_t&) const;
	prd_t  predictor  (const tgo_t& t, const itg_t& i, const nav_t&) const;
	void   corrector  (const prd_t& p, const nav_t&);
};

}}


