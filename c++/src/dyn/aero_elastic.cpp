/*============================================================================*/
/*                                                                            */
/*============================================================================*/

////////////////////////////////////////////////////////////////////////////////
/// \brief
/// \author   Augusto Zumarraga
/// \date     creación: 20/08/2024
/// \date     revisión: 20/08/2024
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
#include <dgnc/rocket/dyn/aero_elastic.hpp>

using namespace dgnc;
using namespace rocket;

struct_t& struct_t::import(const char* csv_name)
{
	csv::table_t csv = csv::read(csv_name, ' ');
 	if(csv.size() > 0)
 	{
		assert(csv.size() >= 8);
		// m, wn, xi, ηw, ηq, ηf, y(x_isa), y'(x_isa)
		m_params.setup(csv[0], csv, 1);
 	}
	return *this;
}

struct_t::state_t struct_t::integrate(state_t x, scalar m, const velocity_t& v, const angle_rate_t& w, const force_t& f, scalar h) const
{
	data_t::result_t prm = m_params(m);
	x.wn = prm[0];
	x.xi = prm[1];
    double bv = prm[2], bw = prm[3], bf = prm[4];
	x.y  = prm[5];
	x.yy = prm[6];
	x.uy = bv*v.y() + bw*w.z() + bf*f.y();
	x.uz = bv*v.z() - bw*w.y() + bf*f.z();
//	x.uy = bf*f.y();
//	x.uz = bf*f.z();

	model_t mdl(x.wn, x.xi, h);
	x.y_state = mdl.integral(x.y_state, x.uy);
	x.z_state = mdl.integral(x.z_state, x.uz);
	return x;
}

struct_t::sens_t struct_t::get_sens(state_t x) const
{
	sens_t sns;
	double k_ = x.wn*x.wn, b_ = 2*x.xi*x.wn;

	sns.sfc.y() = 0;
	sns.sfc.y() = (x.uy - k_*x.y_state.pos - b_ *x.y_state.vel)*x.y;
	sns.sfc.z() = (x.uz - k_*x.z_state.pos - b_ *x.z_state.vel)*x.y;
	sns.wbi.z() = x.yy*x.y_state.vel;
	sns.wbi.y() = x.yy*x.z_state.vel;
	sns.wbi.x() = 0;
	sns.att = quaternion(0, -x.yy*x.z_state.pos, x.yy*x.y_state.pos);

	return sns;
}




