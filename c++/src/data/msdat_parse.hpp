/*============================================================================*/
/*                                                                            */
/*============================================================================*/

////////////////////////////////////////////////////////////////////////////////
/// \brief
/// \author   Augusto Zumarraga
/// \date     creación: 10/06/2024
/// \date     revisión: 23/06/2024
/// \date     revisión: 26/06/2024 refactoring
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

#include "msdat_imp.hpp"
#include "text.hpp"
#include <dgnc/store/text_file.hpp>

namespace dgnc { namespace rocket { namespace msdat {

scalar   parse_val  (const char* txt, const char* name);
values_t parse_val  (const char* txt, const char* name, unsigned count);
values_t parse_array(const char* txt, unsigned count);
string   parse_text (const char* txt, const char* name);


inline values_t parse_val(const string& txt, const char* name, unsigned count)
{
	return parse_val(txt.c_str(), name, count);
}
inline scalar parse_val(const string& txt, const char* name)
{
	return parse_val(txt.c_str(), name);
}
inline values_t parse_array(const string& txt, unsigned count)
{
	return parse_array(txt.c_str(), count);
}
inline string parse_text(const string& txt, const char* name)
{
	return parse_text(txt.c_str(), name);
}


string find_card(text_file::iterator& line, const text_file::iterator end, const char* nombre);
string find_control(text_file::iterator& line, const text_file::iterator end, const char* nombre);

struct FLTCON_t
{
	range_t mach;
	range_t alfa;
	scalar  beta;
	scalar  alt;

	FLTCON_t(const char* txt);
};

struct fcond_t
{
	scalar MACH;
	scalar REYNOLDS;
	scalar ALT;
	scalar Q;
	scalar BETA;
	scalar ROLL;

	scalar REF_AREA;
	scalar MOMENT_CENTER;
	scalar REF_LENGTH;
	scalar LAT_REF_LENGTH;

	fcond_t() : MACH(0) // @suppress("Class members should be properly initialized")
	{}
	fcond_t& search(text_file::iterator& line, const text_file::iterator end);

	operator bool() const { return MACH > 0; }
};
struct case_t
{                       //  CASE.          1     2    3    4    5    6    7    8    9   10   11
	table2_t COEF;      //  COEF [nalpha]: ALPHA CN   CM   CA   CY   CLN  CLL  CL   CD  L/D  XCP.
	table2_t DERIV;     //  DERIV[nalpha]: ALPHA CNA  CMA  CYB  CLNB CLLB
	table2_t DYN_N;     //  DYN_N[nalpha]: ALPHA CNQ  CMQ  CAQ  CNAD CMAD
	table2_t DYN_L;     //  DYN_L[nalpha]: ALPHA CYR  CLNR CLLR CYP  CLNP CLLP

	case_t(text_file::iterator& line, const text_file::iterator end, unsigned nalpha, bool deriv_rad);
};

}}}



