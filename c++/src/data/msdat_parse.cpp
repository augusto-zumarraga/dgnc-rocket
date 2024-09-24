/*============================================================================*/
/*                                                                            */
/*============================================================================*/

////////////////////////////////////////////////////////////////////////////////
/// \brief
/// \author   Augusto Zumarraga
/// \date     creación: 10/06/2024
/// \date     revisión: 17/06/2024
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
#include "msdat_parse.hpp"
#include <dgnc/store/text_file.hpp>

using namespace dgnc;
using namespace rocket;
using namespace msdat;

namespace {

template <typename P1>
void log(const char* fmt, P1 p1)
{
	printf(fmt, p1);
}
template <typename P1, typename P2>
void log(const char* fmt, P1 p1, P2 p2)
{
	printf(fmt, p1, p2);
}
template <typename P1, typename P2, typename P3>
void log(const char* fmt, P1 p1, P2 p2, P3 p3)
{
	printf(fmt, p1, p2, p3);
}
template <typename P1, typename P2, typename P3, typename P4>
void log(const char* fmt, P1 p1, P2 p2, P3 p3, P4 p4)
{
	printf(fmt, p1, p2, p3, p4);
}
template <typename P1, typename P2, typename P3, typename P4, typename P5>
void log(const char* fmt, P1 p1, P2 p2, P3 p3, P4 p4, P5 p5)
{
	printf(fmt, p1, p2, p3, p4, p5);
}

//------------------------------------------------------------------------------
inline const char* discard(const char* s, unsigned n)
{
	for( ; n && *s; --n, ++s)
	{}
	return s;
}
const char* find_val(const char* txt, const char* tag)
{
    assert(txt);
	const char* h = findtag(txt, tag);
    if(h)
        h = find_char(h, '=');
    return h;
}
void scan_row(const char* txt, unsigned len, scalar* dst, unsigned n_skip = 1)
{
	while(n_skip)
	{
		txt = find_number(txt);
		if(!txt)
			throw runtime_error("scan_row: digits not found");
        txt = find_char(txt, ' ');
        --n_skip;
	}
    for(scalar* end = dst + len; dst < end; ++dst)
    {
		if(!txt)
			throw runtime_error("scan_row: no row data");
    	txt = find_number(txt);
    	if(!txt)
    		throw runtime_error("scan_row: no column data");
        int n = sscanf(txt,"%lf", dst);
        if(n != 1)
        {
            *dst = NAN;
            txt = find_char(txt, '*');
        }
        txt = find_char(txt, ' ');
    }
}

}

//-------------------------------------------------------------------------
scalar msdat::parse_val(const char* txt, const char* name)
{
	assert(txt);
    txt = find_val(txt,name);
	if(!txt)
    	throw runtime_error("parse_val not found");
    float val;
    if(sscanf(txt,"%f", &val) == EOF)
    	throw runtime_error("parse_val");
    return val;
}
values_t msdat::parse_val(const char* txt, const char* name, unsigned count)
{
    txt = find_val(txt,name);
    assert(txt);
    std::vector<scalar> vals;
	vals.reserve(count);
	float val;
    for(unsigned k=0; k<count && *txt; ++k)
    {
        if(sscanf(txt,"%f", &val) == EOF)
        {
            txt = find_char(txt, 0);
        	continue;
        }
        vals.push_back(val);
        txt = find_char(txt, ',');
    }
    return vals;
}
values_t msdat::parse_array(const char* txt, unsigned count)
{
	values_t A;
	A.reserve(count);
	float val;
    for(unsigned k=0; k<count; ++k)
    {
        if(sscanf(txt,"%f", &val) == EOF)
        {
            txt = find_char(txt, 0);
        	continue;
        }
        A.push_back(val);
        txt = find_char(txt, ',');
    }
    return A;
}
string msdat::parse_text(const char* txt, const char* name)
{
    txt = find_val(txt, name);
    assert(txt);
    const char* b = find_char(txt, ',');
    return string(txt, b ? b-txt-1 : strlen(txt));
}


//-------------------------------------------------------------------------
string msdat::find_card(text_file::iterator& line, const text_file::iterator end, const char* nombre)
{
	string card;
    while(line < end)
    {
    	string ss_line = line('*'); ++line;
    	const char* txt = ss_line.empty() ? 0 : findtag(ss_line.c_str(), nombre);
        if(txt)
        {
            const char* e = 0;
            log("CARD %s at line %d\n", nombre, line.pos()+1);
            while(!e)
            {
            	card += txt;
                e   = find_char(txt,'$');
                txt = discard(*line, 4);
                if(++line >= end)
                	throw std::runtime_error("EOF while searching for CARD terminator");
            }
            break;
        }
    }
	return card;
}
string msdat::find_control(text_file::iterator& line, const text_file::iterator end, const char* nombre)
{
    string ctrl;
    unsigned len = strlen(nombre);
    for( ; line < end; ++line)
    {
    	ctrl = line('*');
    	const char* txt = ctrl.empty() ? 0 : findtag(ctrl.c_str(), nombre);
        if(txt)
        {
            assert(*(txt-len-1) == ' ' && *(txt) == ' ');
            {
                txt++;
                char buff[256]; buff[0] = 0;
                sscanf(txt, "%255s", buff);
                ctrl = buff;
                log("CONTROL CARD %s at line %d\n", nombre, line.pos()+1);
                break;
    		}
    	}
    }
    return ctrl;
}

//------------------------------------------------------------------------------
FLTCON_t::FLTCON_t(const char* txt)
{
	scalar qty;
    string   val;

    qty  =   parse_val(txt, "NALPHA");
    val  =    find_val(txt, "ALPHA");
    alfa = parse_array(val.c_str(), qty);

    qty  =   parse_val(txt, "NMACH");
	val  =    find_val(txt, "MACH");
    mach = parse_array(val.c_str(), qty);

    beta = parse_val(txt, "BETA");
    alt  = parse_val(txt, "ALT");
}

fcond_t& fcond_t::search(text_file::iterator& line, const text_file::iterator end)
{
    while(line < end)
    {
        const char* txt = *(line++);
        txt = strfind(txt, "******* FLIGHT CONDITIONS AND REFERENCE QUANTITIES *******");
        if(txt)
        {
        	unsigned nline = line.pos();
            txt = discard(*line, 4);
            MACH     = parse_val(txt, "MACH NO");
            REYNOLDS = parse_val(txt, "REYNOLDS NO");
            if(++line >= end)
            	throw std::runtime_error("EOF while searching for FLIGHT CONDITIONS data");
            
            txt = discard(*line, 4);
            ALT = parse_val(txt, "ALTITUDE");
            Q   = parse_val(txt, "DYNAMIC PRESSURE");
            if(++line >= end)
            	throw std::runtime_error("EOF while searching for FLIGHT CONDITIONS data");

            txt = discard(*line, 4);
            BETA = parse_val(txt, "SIDESLIP");
            ROLL = parse_val(txt, "ROLL");

            if((line += 3) >= end)
            	throw std::runtime_error("EOF while searching for FLIGHT CONDITIONS data");
            // = parse_val(tline, "REF AREA =");
            //. = parse_val(tline(k:end), "MOMENT CENTER =");

            //. = parse_val(tline, "REF LENGTH =");
            //. ] = parse_val(tline(k:end), "LAT REF LENGTH =");
            log("FCOND DATA at line %d: MACH = %f, BETA = %f, ROLL = %f\n", nline, MACH, BETA, ROLL);
            break;
        }
    }
    return *this;
}

case_t::case_t(text_file::iterator& line, const text_file::iterator end, unsigned nalpha, bool deriv_rad)
{
	while(line < end)
    {
        const char* txt = *line;
        text_file::iterator lbak = line++;
        const char* h = strfind(txt, "******* FLIGHT CONDITIONS AND REFERENCE QUANTITIES *******");
        if(h)
        {
        	line = lbak;
            break;
        }
        h = strfind(txt, "----- LONGITUDINAL -----     -- LATERAL DIRECTIONAL --");
        if(h)
        {
            log("COEFS at line %d\n", line.pos());
            // "ALPHA       CN        CM        CA        CY       CLN       CLL");
            if((line += 2) >= end)
            	throw std::runtime_error("EOF while searching for CN... COEFs");

            COEF.resize(nalpha, 10);
            for(unsigned a=0; a<nalpha; ++a)
            {
                txt = *line;
                scan_row(txt, 6, COEF.row(a));
                if(++line >= end)
                	throw std::runtime_error("EOF while loading CN COEFs");
            }
            // "ALPHA       CL        CD      CL/CD     X-C.P.");
            if((line += 3) >= end)
            	throw std::runtime_error("EOF while searching for CL... COEFs");
            for(unsigned a=0; a<nalpha; ++a)
            {
                txt = *line;
                scan_row(txt, 4, COEF.row(a)+6);
                ++line;
            }
            continue;
        }
        if(deriv_rad)
            h = strfind(txt, "---------- DERIVATIVES (PER RADIAN) ----------");
        else
            h = strfind(txt, "---------- DERIVATIVES (PER DEGREE) ----------");

        if(h)
        {
            log("DERIVS at line %d\n", line.pos()+1); // inp.line());
            //"ALPHA       CNA         CMA         CYB         CLNB        CLLB");
            if(++line >= end)
            	throw std::runtime_error("EOF while searching for DERIVs");
            DERIV.resize(nalpha, 5);
            for(unsigned a=0; a<nalpha; ++a)
            {
                txt = *line;
                scan_row(txt, 5, DERIV.row(a));
                ++line;
            }
            continue;
        }
        if(deriv_rad)
            h = strfind(txt, "------------ DYNAMIC DERIVATIVES (PER RADIAN) -----------");
        else
            h = strfind(txt, "------------ DYNAMIC DERIVATIVES (PER DEGREE) -----------");

        if(h)
        {
            txt = *line;
            if(++line >= end)
            	throw std::runtime_error("EOF while loading CNQ... coefs");
            h = strfind(txt, "ALPHA       CNQ        CMQ        CAQ       CNAD       CMAD");
            if(h)
            {
                log("DYNAMIC N DATA at line %d\n", line.pos()+1);
                DYN_N.resize(nalpha, 5);
                for(unsigned a=0; a<nalpha; ++a)
                {
                    txt = *line;
                    scan_row(txt, 5, DYN_N.row(a));
                    ++line;
                }
                continue;
            }
            else
            {
                h = strfind(txt, "ALPHA       CYR       CLNR       CLLR        CYP       CLNP       CLLP");
				if(h)
				{
                    log("DYNAMIC L DATA at line %d\n", line.pos()+1);
	                DYN_L.resize(nalpha, 6);
	                for(unsigned a=0; a<nalpha; ++a)
	                {
	                    txt = *line;
	                    scan_row(txt, 6, DYN_L.row(a));
	                    ++line;
	                }
                    continue;
				}
        	}
        }
    }
}

