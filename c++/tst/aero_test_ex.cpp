/*============================================================================*/
/*                                                                            */
/*============================================================================*/

////////////////////////////////////////////////////////////////////////////////
/// \brief
/// \author   Augusto Zumarraga
/// \date     creación: 10/06/2024
/// \date     revisión: 22/06/2024
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
#include <dgnc/rocket/data/aero.hpp>
#include <dgnc/store/csv_file.hpp>
#include <dgnc/utest.hpp>

using namespace dgnc;

typedef utest::handler_t test_handler_t;

namespace {
double check(double prev, double f1, double f2, double a, double b, double m, const geom::vector& w, const char* name, unsigned k)
{
	using math::r2d;
	double d;
	d = std::abs(f1 - f2);
	if(d > prev)
	{
		prev = d;
		std::cout << name << '(' << k << ", α:" << r2d(a) << ", β:" << r2d(b) << ", M:" << m
				  << ", w = {" << w.x() << ',' << w.y() << ',' << w.z()
				  << "}) diff = " << d << '/' << f2 << std::endl;
	}
	return prev;
}
}

bool aero_test_ex()
{
	test_handler_t tst;
	using std::abs;

	tst = "import csv";
 	csv::table_t data = csv::read("check_aero.csv", ' ', 0);
 	TestAssert(data.size() == 16);

	// R(k,:) = [V W aoa slp M Q Fa Ma];

	const csv::column_t& u   = data[ 0];
	const csv::column_t& v   = data[ 1];
	const csv::column_t& w   = data[ 2];
	const csv::column_t& p   = data[ 3];
	const csv::column_t& q   = data[ 4];
	const csv::column_t& r   = data[ 5];
    const csv::column_t& aoa = data[ 6];
    const csv::column_t& slp = data[ 7];
    const csv::column_t& mch = data[ 8];
    const csv::column_t& Q   = data[ 9];
    const csv::column_t& Xa  = data[10];
    const csv::column_t& Ya  = data[11];
    const csv::column_t& Za  = data[12];
    const csv::column_t& La  = data[13];
    const csv::column_t& Ma  = data[14];
    const csv::column_t& Na  = data[15];

	tst = "load data";

	using namespace rocket;
	using namespace aero;

	model mdl;
    mdl.load("S1_aero.dat");

	csv::column_t::const_iterator p_u   = u  .begin();
	csv::column_t::const_iterator p_v   = v  .begin();
	csv::column_t::const_iterator p_w   = w  .begin();
	csv::column_t::const_iterator p_p   = p  .begin();
	csv::column_t::const_iterator p_q   = q  .begin();
	csv::column_t::const_iterator p_r   = r  .begin();
    csv::column_t::const_iterator p_aoa = aoa.begin();
    csv::column_t::const_iterator p_slp = slp.begin();
    csv::column_t::const_iterator p_mch = mch.begin();
    csv::column_t::const_iterator p_Q   = Q  .begin();
    csv::column_t::const_iterator p_Xa  = Xa .begin();
    csv::column_t::const_iterator p_Ya  = Ya .begin();
    csv::column_t::const_iterator p_Za  = Za .begin();
    csv::column_t::const_iterator p_La  = La .begin();
    csv::column_t::const_iterator p_Ma  = Ma .begin();
    csv::column_t::const_iterator p_Na  = Na .begin();

    double vs  = 2.994422949417801e+02;
    double po  = 2.643110457234041e+04;
    double rho = 0.412683460380927;
    position_t p_ref(-2.0, 0, 0);

	static char trace[64];
	tst = (const char*) trace;
    double a_tol = 1e-4;
    double q_tol = 1;
    double f_diff[3] = { 0, 0, 0};
    double m_diff[3] = { 0, 0, 0};

    for(unsigned k=0; k<u.size(); ++k
    							, ++p_u  , ++p_v  ,	++p_w  , ++p_p  , ++p_q, ++p_r
								, ++p_aoa, ++p_slp, ++p_mch, ++p_Q
								, ++p_Xa , ++p_Ya , ++p_Za , ++p_La , ++p_Ma, ++p_Na)
    {
    	sprintf(trace, "compute %d", k);

    	vector   V(*p_u, *p_v, *p_w);
    	vector   W(*p_p, *p_q, *p_r);
    	air_t  air(rho, po, vs, 293.15);
    	wind_t wnd(air, V, W, p_ref);

    	TestAssert(abs(wnd.aoa - *p_aoa) < a_tol);
    	TestAssert(abs(wnd.slp - *p_slp) < a_tol);
    	TestAssert(abs(wnd.mch - *p_mch) < 1e-3 );
    	TestAssert(abs(wnd.Q   - *p_Q  ) < q_tol);

    	model::result_t res = mdl(wnd, p_ref);
		{
    		int p = std::cout.precision(5);
    		f_diff[0] = check(f_diff[0], res.force .x(), *p_Xa, *p_aoa, *p_slp, *p_mch, W, "fx", k);
    		f_diff[1] = check(f_diff[1], res.force .y(), *p_Ya, *p_aoa, *p_slp, *p_mch, W, "fy", k);
    		f_diff[2] = check(f_diff[2], res.force .z(), *p_Za, *p_aoa, *p_slp, *p_mch, W, "fz", k);
    		m_diff[0] = check(m_diff[0], res.moment.x(), *p_La, *p_aoa, *p_slp, *p_mch, W, "mx", k);
    		m_diff[1] = check(m_diff[1], res.moment.y(), *p_Ma, *p_aoa, *p_slp, *p_mch, W, "my", k);
    		m_diff[2] = check(m_diff[2], res.moment.z(), *p_Na, *p_aoa, *p_slp, *p_mch, W, "mz", k);
    		std::cout.precision(p);
		}
    }
    return true;
}


