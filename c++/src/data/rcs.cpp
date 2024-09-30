/*============================================================================*/
/*                                                                            */
/*============================================================================*/

////////////////////////////////////////////////////////////////////////////////
/// \brief
/// \author   Augusto Zumarraga
/// \date     creación: 26/06/2024
/// \date     revisión: 30/06/2024
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
#include <dgnc/rocket/data/rcs.hpp>

using namespace dgnc;
using namespace rocket;

rcs_t::axis_t::axis_t(vector pos, direction axis, scalar thrust, second_t t_brn, second_t t_decay, second_t t_cyc)
: thrust(axis * thrust)
, pos   (pos)
, t_burn(t_brn)
, t_cycl(t_cyc)
, slope (1.0/t_decay)
{}

rcs_t::result_t rcs_t::operator()(second_t tm, const fires_t& fs, vector cg) const
{
	assert(fs.size() == axes.size());
	result_t r;
	fires_t::const_iterator j = fs.begin();
	for(axes_t::const_iterator i = axes.begin(), e = axes.end(); i < e; ++i, ++j)
	{
		if(*j)
		{
			double f = i->slope/(*j + i->t_burn - tm);
			math::sat(f, 1, 0);
			if(f)
			{
				vector frc = i->thrust * f;
				r.force  += frc;
				r.moment += (cg - i->pos) ^ frc;
			}
		}
	}
	return r;
}



