/*============================================================================*/
/*                                                                            */
/*============================================================================*/

////////////////////////////////////////////////////////////////////////////////
/// \brief
/// \author   Augusto Zumarraga
/// \date     creación: 26/06/2024
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
#include <dgnc/rocket/data/prop.hpp>

using namespace dgnc;
using namespace rocket;

const engine_t& engine_t::save(const char* fname) const
{
	return *this;
}
engine_t& engine_t::load(const char* fname)
{
	return *this;
}
engine_t& engine_t::import(const char* fname)
{
	csv::table_t data = csv::read(fname);
	const csv::column_t& params = data[0];

    mdot     = params[0];
    pe       = params[1];
    Ae       = params[2];
    x_gimbal = params[3];
    d_max    = params[4];

    constexpr auto N = 5;

    unsigned n = params.size() - N;
    assert(n%2 == 0);
    n /= 2;
    csv::column_t::const_iterator i = params.begin() + N;
  	ve.setup(i, i+n, i+n);
	return *this;
}
engine_t::result_t engine_t::operator()(scalar f, scalar po, tvc_state_t tvc, position_t cg) const
{
	result_t r;
	if(f > 1e-2)
	{
		r.mdot     = mdot * f;
		double _ve = ve(po);
		r.thrust   = r.mdot * _ve + (pe*f - po)*Ae;
		r.force    = vector(r.thrust, -tvc.dz * d_max * r.thrust, tvc.dy * d_max * r.thrust);
		cg.x()    -= x_gimbal;
		r.moment   = cg ^ r.force;
	}
	return r;
}
