/*============================================================================*/
/*                                               Guiado, Navegación y Control */
/*============================================================================*/

////////////////////////////////////////////////////////////////////////////////
/// \brief
/// \author   Augusto Zumarraga
/// \date     creación: 27/06/2024
/// \date     revisión: 25/08/2024
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

namespace gnc { namespace cmnds {

/// Acciones contínuas
struct cont_t
{
	float dy;
	float dz;
	float da;
	float rc;

	cont_t() : dy(0), dz(0), da(0), rc(0)
	{}
	void reset() { dy = dz = da = rc = 0; }
};

/// Acciones discretas
struct bool_t
{
	bool eng; // estado del motor
	bool sep; // separación de etapas
	bool rel; // liberar item expulsable

	bool_t() : eng(false), sep(false), rel(false)
	{}
	void reset() { eng = sep = rel = false; }
};

}}

