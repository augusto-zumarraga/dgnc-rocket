/*============================================================================*/
/*                                                                            */
/*============================================================================*/

////////////////////////////////////////////////////////////////////////////////
/// \brief
/// \author   Augusto Zumarraga
/// \date     creación: 10/06/2024
/// \date     revisión: 05/10/2024
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
#include "fsim_impl.hpp"
#include "plot/plot.hpp"
#include <dgnc/store/ini_file.hpp>
#include <dgnc/rocket/dyn/rigid_body.hpp>

using namespace dgnc;
using namespace fsim;

rocket_t::rocket_t(const std::string& mdl_s1, const std::string& mdl_s2, const std::string& wind, double _ts, logger_t& l)
: S1(mdl_s1.c_str())
, S2(mdl_s2.c_str())
, p_stage(&S1)
, log (l)
, ts  (_ts)
, mark(0)
, eng(false)
, T(-1)
{
	S1.add_roll_fin();
	S2.add_roll_rcs();
	if(!wind.empty())
		S1.wind().import(wind.c_str());
	S1.acts().sep().t_time   = second_t(2);
	S1.acts().sep().on_event = std::bind(&rocket_t::on_separation, this, _1);
	S2.acts().rel().t_time   = second_t(0.2);
	S2.acts().rel().on_event = std::bind(&rocket_t::on_release   , this, _1);
}
void rocket_t::init(second_t toff, const navs::ecef::state_t& po, navs::angle_rate_t sr)
{
	xo.elapsed = toff;
	S1.init(xo, po);
	sep_rot = sr;
}
void rocket_t::cmnds(second_t t, const gnc::cmnds::cont_t& AOs, const gnc::cmnds::bool_t& DOs)
{
	p_stage->acts().tvc(AOs.dy , AOs.dz);
	p_stage->acts().ail(AOs.da );
	p_stage->acts().rcs(AOs.rc );
	p_stage->acts().eng(DOs.eng);
	if(DOs.sep)
		p_stage->acts().sep().loose(t);
	if(DOs.rel)
		p_stage->acts().rel().loose(t);
	// -------------------------------------------------    EVENTS HANDLING
	if(eng != DOs.eng)
	{
		eng = DOs.eng;
		if(eng && T.get() < 0)
		{
			T = t;
			log << "\nT = " << T;
		}
		else
			log << "\nT+" << (t - T);
		log << (eng ? " : IGNITION" : " : CUT OFF") << '\n';
	}
}
void rocket_t::on_separation(bool b_trigger)
{
	if(T < second_t(0))
		log << '\n' << xo.elapsed;
	else
		log << "\nT+" << xo.elapsed - T;
	if(b_trigger)
		log << " : SEPARATION START" << '\n';
	else
	{
		p_stage = &S2;
		S2.init(xo);
		xo.wbi += sep_rot;
		mark = (xo.elapsed / ts);
		log << " : SEPARATION COMPLETED" << '\n';
	}
}
void rocket_t::on_release(bool b_trigger)
{
	if(T < second_t(0))
		log << '\n' << xo.elapsed;
	else
		log << "\nT+" << xo.elapsed - T;
	if(!b_trigger)
		log << " : MASS unlock\n";
	else
	{
		double me = p_stage->mass().extra.eject();
		xo.mass -= me;
		log << " : MASS RELEASE (" << me << "kg)" << '\n';
	}
}


