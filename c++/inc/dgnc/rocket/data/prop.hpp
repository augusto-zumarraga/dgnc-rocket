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
#pragma once
#include "data.hpp"

namespace dgnc { namespace rocket {

//------------------------------------------------------------------------------
struct engine_t
{
    scalar mdot     ;
    scalar pe       ;
    scalar Ae       ;
    scalar x_gimbal ;
    scalar d_max    ;
	spline_1 ve;

	struct result_t
	{
		scalar mdot;
		scalar thrust;
		force_t  force;
		moment_t moment;
		result_t() : mdot(0), thrust(0), force(0), moment(0)
		{}
	};
	/// \param f   nivel de flujo [normalizado]
	/// \param po  presión atmosférica [Pa]
	/// \param tvc deflexión del eje de empuje
	/// \param cg  posición del CG
	result_t operator()(scalar f, scalar po, tvc_state_t tvc, position_t cg) const;

    const
	engine_t& save(const char* fname) const;
    engine_t& load(const char* fname);
    engine_t& import(const char* csv_name);
};

}}
