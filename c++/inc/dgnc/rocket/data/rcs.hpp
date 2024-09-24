/*============================================================================*/
/*                                                                            */
/*============================================================================*/

////////////////////////////////////////////////////////////////////////////////
/// \brief
/// \author   Augusto Zumarraga
/// \date     creación: 29/08/2024
/// \date     revisión: 29/08/2024
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
#include "data.hpp"

namespace dgnc { namespace rocket {

//------------------------------------------------------------------------------
struct rcs_t
{
	class axis_t
	{
	public:

		axis_t(vector pos, direction axis, scalar thrust, second_t t_brn, second_t t_decay, second_t t_cyc);
		axis_t() // @suppress("Class members should be properly initialized")
		{}

	protected:

		friend class rcs_t;
		vector   thrust; // empuje
		vector   pos   ; // posición
		second_t t_burn; // tiempo de
		second_t t_cycl; // frecuencia de disparo
		double   slope;
	};
	typedef std::vector<axis_t> axes_t;
	axes_t axes;
	typedef std::vector<second_t> fires_t;

	struct result_t
	{
		force_t  force;
		moment_t moment;
		result_t() : force(0), moment(0)
		{}
	};
	result_t operator()(second_t tm, const fires_t&, vector cg) const;
	operator bool() const { return !axes.empty(); }
};

}}
