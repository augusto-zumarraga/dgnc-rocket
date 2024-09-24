/*============================================================================*/
/*                                                                            */
/*============================================================================*/

////////////////////////////////////////////////////////////////////////////////
/// \brief
/// \author   Augusto Zumarraga
/// \date     creación: 26/06/2024
/// \date     revisión: 21/08/2024
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
struct inertia_t
{
	typedef t_splines_1<6> data_t;
	data_t data;

	struct item_t
	{
		item_t() : m(0), xcg(0), Ix(0), Iy(0), Iz(0)
		{}
		operator bool() const { return m > 0; }
		// Evento de liberación
		double eject()
		{
			double p = m;
			m = 0;
			return p;
		}

		double m;
		double xcg;
		double Ix;
		double Iy;
		double Iz;
	    item_t& import(const char* csv_name);

	} extra;

	double initial() const
	{
		return data.range().back() + extra.m;
	}
	double propelents() const
	{
		return m_prop;
	}

	mass_prop_t operator()(scalar m) const;

    const
	inertia_t& save(const char* fname) const;
    inertia_t& load(const char* fname);
    inertia_t& import(const char* csv_name);

private:

    double m_prop;
};

}}

