/*============================================================================*/
/*                                                                            */
/*============================================================================*/

////////////////////////////////////////////////////////////////////////////////
/// \brief
/// \author   Augusto Zumarraga
/// \date     creación: 10/06/2024
/// \date     revisión: 05/07/2024
/// \date     revisión: 21/07/2024 refactoring
/// \date     revisión: 13/09/2024 ostream
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

#include <dgnc/numeric/one_dof.hpp>
#include "../data/data.hpp"

namespace dgnc { namespace rocket {

//------------------------------------------------------------------------------
class struct_t
{
public:

	typedef ode::one_dof_lin_mdl_t model_t;

	struct sens_t
	{
		bool         good;
		force_t      sfc;
		quaternion   att;
		angle_rate_t wbi;

		sens_t(bool b = false) : good(b)
		{
			if(b)
			{
				sfc = 0;
				wbi = 0;
			}
		}
		operator bool() const { return good; }
		void invalidate()     { good = false; }
	};
	struct state_t
	{
		model_t::state_t y_state, z_state;
		double wn, xi, y, yy, uy, uz;
		void clear()
		{
			wn = xi = y = yy = uy = uz = 0;
			y_state.clear();
			z_state.clear();
		}

	};

	operator bool() const
	{
		return !m_params.range().empty();
	}
	struct_t& import(const char* csv_name);
	state_t   integrate(state_t, scalar mass, const velocity_t& v, const angle_rate_t& w, const force_t& f, scalar h) const;
	sens_t    get_sens(state_t x) const;

protected:

	// m -> { wn, xi, ηw, ηq, ηf, y(x_isa), y'(x_isa) }
	typedef t_splines_1<7> data_t;
	data_t m_params;
};

}}

inline std::ostream& operator<<(std::ostream& s, const dgnc::rocket::struct_t::model_t::state_t& x)
{
	s << x.pos << ',' << x.vel;
	return s;
}
inline std::ostream& operator<<(std::ostream& s, const dgnc::rocket::struct_t::state_t& x)
{
	s << x.y_state << ',' << x.z_state;
	return s;
}


