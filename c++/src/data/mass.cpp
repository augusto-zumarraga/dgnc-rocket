/*============================================================================*/
/*                                                                            */
/*============================================================================*/

////////////////////////////////////////////////////////////////////////////////
/// \brief
/// \author   Augusto Zumarraga
/// \date     creación: 26/06/2024
/// \date     revisión: 26/06/2024
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
#include <dgnc/rocket/data/mass.hpp>

using namespace dgnc;
using namespace rocket;

const inertia_t& inertia_t::save(const char* fname) const
{
	return *this;
}
inertia_t& inertia_t::load(const char* fname)
{
	return *this;
}
// tbl = [m cg:{x y z} I:{x y z}];
inertia_t& inertia_t::import(const char* csv_name)
{
	csv::table_t csv = csv::read(csv_name, ' ');
 	assert(csv.size() == 7);
 	data.setup(csv[0], csv, 1);
 	m_prop = data.range().back() - data.range().front() - extra.m;
	return *this;
}

mass_prop_t inertia_t::operator()(scalar m) const
{
	data_t::result_t x = data(m);

	mass_prop_t r;
	if(extra)
	{
		x[0]  = ((m - extra.m) * x[0] + extra.m * extra.xcg) / m;
		x[3] += extra.Ix;
		double d = x[0] - extra.xcg, d2 = d*d;
		x[4] += extra.Iy + extra.m * d2;
		x[5] += extra.Iy + extra.m * d2;
	}
	r.m  = m;
	r.cg = vector(x[0], x[1], x[2]);
	r.J  = 0;         r.Ji = 0; // J.inv();
	r.J (0,0) = x[3]; r.Ji(0,0) = 1.0f/x[3];
	r.J (1,1) = x[4]; r.Ji(1,1) = 1.0f/x[4];
	r.J (2,2) = x[5]; r.Ji(2,2) = 1.0f/x[5];

	return r;
}



