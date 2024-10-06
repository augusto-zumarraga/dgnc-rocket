/*============================================================================*/
/*                                                                            */
/*============================================================================*/
#pragma once
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

#include "fsim_def.hpp"
#include "fsim.hpp"

namespace dgnc { namespace fsim {

//------------------------------------------------------------------------------
class logger_t
{
public:

	logger_t(std::string fname);
	template <class T>
	logger_t& operator<<(const T& s);
	logger_t& operator<<(char s);

private:

	std::ofstream m_out;
};

//------------------------------------------------------------------------------
/// Ejecutor de la simulación
struct rocket_t
{
	rocket_mdl_t S1, S2, *p_stage;
	logger_t& log;
	double   ts;
	unsigned mark;
	bool     eng;
	rocket_mdl_t::vect_t xo;
	navs::angle_rate_t sep_rot;
	second_t T;

	rocket_t(const std::string& mdl_s1, const std::string& mdl_s2, const std::string& wind, double _ts, logger_t& l);
	rocket_mdl_t* operator->() { return  p_stage; }
	rocket_mdl_t& operator* () { return *p_stage; }
	operator rocket_mdl_t&  () { return *p_stage; }

	void init (second_t toff, const navs::ecef::state_t& po, navs::angle_rate_t sr);
	void cmnds(second_t t, const gnc::cmnds::cont_t& AOs, const gnc::cmnds::bool_t& DOs);
	void on_separation(bool b_trigger);
	void on_release   (bool b_trigger);
};

//------------------------------------------------------------------------------
/// Ejecutor de la simulación
class exec_t
{
public:

	exec_t(std::string);
	bool operator()(bool do_plot);

protected:

	bool fts     (const sim_info_t& s, logger_t&);
	void on_state(second_t T, int st, const sim_info_t& p, logger_t&);
	void update  (const sim_info_t& sim, ins_data_t& ins);
	void dump    (const std::string&, const solver_t::result_t& r);

	double tsim, toff, tlaunch, tint, w_max;
	std::string  mdl_s1, mdl_s2, pln, exp, wind;
	navs::angle_rate_t sep_rot;
};

//------------------------------------------------------------------------------
constexpr auto min_mass = 22.13;
extern navs::orb::circular_t orbit;
}}

//------------------------------------------------------------------------------
std::ostream& operator<<(std::ostream&, const dgnc::fsim::sim_info_t&);

template <class T>
dgnc::fsim::logger_t& dgnc::fsim::logger_t::operator<<(const T& s)
{
	using ::operator<<;
	std::cout << s;
	if(m_out.good())
		m_out << s;
	return *this;
}


