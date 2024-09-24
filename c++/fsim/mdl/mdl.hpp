/*============================================================================*/
/*                                                                            */
/*============================================================================*/
#pragma once
////////////////////////////////////////////////////////////////////////////////
/// \brief
/// \author   Augusto Zumarraga
/// \date     creación: 10/06/2024
/// \date     revisión: 18/07/2024
/// \date     revisión: 18/08/2024 flex
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
#include "mdl_acts.hpp"
#include "wind.hpp"
#include <dgnc/numeric/ode.hpp>

namespace dgnc { namespace fsim {

//------------------------------------------------------------------------------
class rocket_sfc_t
{
public:

	//--------------------------------------------------------------------------
	//  Estados
	//--------------------------------------------------------------------------
	struct state_t
	{
		scalar               mass;
		actuators_t::state_t act ;
		flex_t::state_t      flx ;

		state_t(float m = 1000) : mass(m)
		{}
		state_t operator*(double h) const;
		state_t operator+(const state_t& r) const;
	};
	struct rates_t
	{
		state_t sr; // state rate
        operator const state_t&() const { return sr; }

        vector  fb;
		vector  Mb;
		tensor_t J, Ji;

		const vector  & specifc_force     () const { return fb; }
		const vector  & moment            () const { return Mb; }
		const tensor_t& inertia_tensor    () const { return J ; }
		const tensor_t& inertia_tensor_inv() const { return Ji; }
	};

	//--------------------------------------------------------------------------
	//       Métodos públicos para cómputo e integración numérica
	//--------------------------------------------------------------------------
	template <class S>
	void         init(S& xo, const ecef::state_t&);
	template <class S>
	void         init(S& xo);
	template <class S>
	enviroment_t enviroment(double t, const S& x) const;
	template <class S>
	rates_t      compute(double t, const S& x) const;
	const
	sim_info_t&  sim_info() const { return m_sim; }
	template <class S>
	void         sample_begin(double t, const S& x, double ts);
	template <class S>
	void         sample_end  (double t, const S& x, double ts);

	//--------------------------------------------------------------------------
	// Acceso a componentes
	//--------------------------------------------------------------------------
	const aero_t& aero() const { return m_aero; }
	const mass_t& mass() const { return m_mass; }
	      mass_t& mass()       { return m_mass; }
	const prop_t& prop() const { return m_prop; }
	const flex_t& flex() const { return m_flex; }
	 actuators_t& acts()       { return m_acts; }
          wind_t& wind()       { return m_wind; }

    //--------------------------------------------------------------------------
    // Inicialización
    //--------------------------------------------------------------------------
	rocket_sfc_t(const char* mdl);
	void add_roll_fin();
	void add_roll_rcs();

private:

	friend class dsicrete_states_t;

	aero_t m_aero;
	mass_t m_mass;
	prop_t m_prop;
	rcsa_t m_rcsa;
	flex_t m_flex;
	wind_t m_wind;
	actuators_t m_acts;

	sim_info_t m_sim;
};

typedef t_rigid_body<rocket_sfc_t, nav_ecef_t> rocket_mdl_t;

/// Clase dummy para el control de los pasos de integración
struct dsicrete_states_t
{
	/// Se ejecuta al comenzar un paso de integración
	static void step_begin(double ti, const rocket_mdl_t& mdl, rocket_mdl_t::vect_t& x, double h);
	/// Se ejecuta al finalizar un paso de integración
	static void step_end(double tf, const rocket_mdl_t& mdl, rocket_mdl_t::vect_t&, double);
};

typedef dgnc::ode::t_solver<dgnc::ode::t_RK4<rocket_mdl_t>, dsicrete_states_t> solver_t;
}}

inline std::ostream& operator<<(std::ostream& s, const dgnc::fsim::rocket_sfc_t::state_t& x)
{
	s << x.mass << ',' << x.act << ',' << x.flx;
	return s;
}

#include "mdl_impl.hpp"


