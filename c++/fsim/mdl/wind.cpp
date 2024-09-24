/*============================================================================*/
/*                                                                            */
/*============================================================================*/

////////////////////////////////////////////////////////////////////////////////
/// \brief
/// \author   Augusto Zumarraga
/// \date     creación: 20/08/2024
/// \date     revisión: 04/09/2024
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
#include "wind.hpp"
#include <dgnc/store/csv_file.hpp>

using namespace dgnc;
using namespace fsim;

//------------------------------------------------------------------------------
wind_t::wind_t(const char* csv_name)
{
	if(csv_name)
		import(csv_name);
}
wind_t& wind_t::import(const char* csv_name)
{
	csv::table_t csv = csv::read(csv_name, ' ');
 	assert(csv.size() == 4);
 	m_data.setup(csv[0], csv, 1);
	return *this;
}
ned::vel_t wind_t::operator()(double alt) const
{
	data_t::result_t x = m_data(alt);
	return ned::vel_t(x[0], x[1], x[2]);
}



