/*============================================================================*/
/*                                                                            */
/*============================================================================*/

////////////////////////////////////////////////////////////////////////////////
/// \brief
/// \author   Augusto Zumarraga
/// \date     creación: 27/06/2024
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
#pragma once

#include "../../fcc/fcc.hpp"
#include <dgnc/navs/orb.hpp>
#include <dgnc/numeric/poly.hpp>

namespace gnc {

//------------------------------------------------------------------------------
class fcc_loader_t
{
public:

	struct pd_gains_t
	{
		double f;
		double g;
	};
	struct gains_t
	{
		pd_gains_t roll;
		pd_gains_t ptch;
	};
	struct gain_schedule_t
	{
		struct { std::vector<double> f, g; } roll;
		struct { std::vector<double> f, g; } ptch;
		void resize(size_t s)
		{
			roll.f.resize(s);
			roll.g.resize(s);
			ptch.f.resize(s);
			ptch.g.resize(s);
		}
		void reserve(size_t s)
		{
			roll.f.reserve(s);
			roll.g.reserve(s);
			ptch.f.reserve(s);
			ptch.g.reserve(s);
		}
		size_t size() const
		{
			return roll.f.size();
		}
		void push_back(const gains_t& g)
		{
			roll.f.push_back(g.roll.f);
			roll.g.push_back(g.roll.g);
			ptch.f.push_back(g.ptch.f);
			ptch.g.push_back(g.ptch.g);
		}
	};

	typedef flight_t::times_t  times_t;
	typedef flight_t::params_t params_t;
	typedef ctrl::st_params_t  cparams_t;
	typedef dgnc::navs::orb::circular_t orbit_t;
	//typedef std::vector<gnc::att_t> wire_t;
	struct wire_t
	{
		std::vector<float> range;
		std::vector<dgnc::geom::vector> data;
	};

	// En <path>
	//
	//    /params.ini
	//          orbit
	//               .inclination [⁰]
	//               .height [km]
	//          times [en segundos]
	//               .S1 burn
	//               .S2 burn
	//               .sample
	//               .separation
	//               .ignition
	//          heights [en km]
	//                .relief
	//                .steer
	//                .vaccum
	//          space port [⁰]
	//                .latitud
	//                .longitud
	//          steering
	//                .pre_cycle
	//                .tgo_min
	//                .v_exit
	//
	//    /wire.csv       quaternion ECEF para cada instante de muestreo
	//    /gains_S1.scv   ganancias de primera etapa para cada instante de muestreo
	//    /gains_S2.scv   ganancias de segunda etapa para cada instante de muestreo
	//
	fcc_loader_t(std::string path);

	void print(std::ostream& s) const;

	const ecef::state_t& launch_state() const { return m_launch_state; }
	const orbit_t&       orbit       () const { return m_orbit; }
	const times_t&       times       () const { return m_times; }
	second_t             sample_time () const { return m_times.sampling; }
	second_t             meco        () const { return m_times.s1_burn; }
	second_t             seco        () const { return m_times.s1_burn
			                                         + m_times.s1_coast_time
			                                         + m_times.separation_time
													 + m_times.s2_fire_delay
													 + m_times.s2_burn; }
	void setup(gnc::fcc_t&) const;

protected:

	void import_gains (unsigned stage, const char* fname);
	void import_wire  (const char* fname);
	void import_params(const char* fname);

	wire_t          m_wire;
	times_t         m_times;
	params_t        m_params;
	orbit_t         m_orbit;
	ecef::state_t   m_launch_state;

	gain_schedule_t m_gains[2];
	gains_t         m_f_gains[2];
	cparams_t       m_s1_roll;
	cparams_t       m_s1_ptch;
	cparams_t       m_s2_roll;
	cparams_t       m_s2_ptch;
};

}

inline std::ostream& operator<<(std::ostream& s, const gnc::fcc_loader_t& f)
{
	f.print(s);
	return s;
}


