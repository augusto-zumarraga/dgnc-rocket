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
#include <stdio.h>
#include <stdlib.h>
#include <matplot/matplot.h>
#include <dgnc/rocket/data/aero.hpp>
#include <dgnc/rocket/data/msdat.hpp>
#include <dgnc/numeric/interp.hpp>
#include <dgnc/utest.hpp>

using namespace dgnc;
using namespace rocket;

typedef utest::handler_t test_handler_t;

namespace {

inline double r2d(double r)
{
	constexpr auto f = 180.0/3.1415926535898;
	return r * f;
}

void show_coef(const msdat::data_t& dt, const aero::ranges_t& R, const aero::coefs_t& cf, const msdat::coef_t& CF, const char* sz_lbl)
{
    using namespace matplot;
    using namespace rocket;
    using namespace aero;

    mch M(0);
    slp b(3);
    values_t mach{R.mach[M]}, alfa(100);

    float a = 0, da = R.alfa.back()/alfa.size();
	for(values_t::iterator i = alfa.begin(), e = alfa.end(); i < e; ++i, a += da)
		*i = a;

	auto f = figure(false);
	auto ax1 = nexttile();
	{
	    values_t beta{R.beta[b]};
	    coefs_t::set_t CL = cf(R, mach, beta, alfa);

	    std::vector<double> x(alfa.size());
		std::vector<double> y(beta.size());
	    std::transform(alfa.begin(), alfa.end(), x.begin(), r2d);
	    std::transform(beta.begin(), beta.end(), y.begin(), r2d);
		auto [X, Y] = meshgrid(x, y);
		auto p = plot3(X, Y, CL);
		p[0]->line_width(2);
	}
	hold(on);
	{
	    values_t beta{(R.beta[b]+R.beta[b+1])/2};
	    coefs_t::set_t CL = cf(R, mach, beta, alfa);

	    std::vector<double> x(alfa.size());
		std::vector<double> y(beta.size());
	    std::transform(alfa.begin(), alfa.end(), x.begin(), r2d);
	    std::transform(beta.begin(), beta.end(), y.begin(), r2d);
		auto [X, Y] = meshgrid(x, y);
		auto p = plot3(X, Y, CL);
		p[0]->line_width(2);
	}
	{
	    values_t beta{(R.beta[b+4]+R.beta[b+5])/2};
	    coefs_t::set_t CL = cf(R, mach, beta, alfa);

	    std::vector<double> x(alfa.size());
		std::vector<double> y(beta.size());
	    std::transform(alfa.begin(), alfa.end(), x.begin(), r2d);
	    std::transform(beta.begin(), beta.end(), y.begin(), r2d);
		auto [X, Y] = meshgrid(x, y);
		auto p = plot3(X, Y, CL);
		p[0]->line_width(2);
	}
	{
	    values_t beta{0};
	    coefs_t::set_t CL = cf(R, mach, beta, alfa);

	    std::vector<double> x(alfa.size());
		std::vector<double> y(beta.size());
	    std::transform(alfa.begin(), alfa.end(), x.begin(), r2d);
	    std::transform(beta.begin(), beta.end(), y.begin(), r2d);
		auto [X, Y] = meshgrid(x, y);
		auto p = plot3(X, Y, CL);
		p[0]->line_width(2);
	}
    {
		std::vector<double> x(dt.alfa.size());
		std::vector<double> y(dt.beta.size());
		vector_2d Z;

	    std::transform(dt.alfa.begin(), dt.alfa.end(), x.begin(), r2d);
	    std::transform(dt.beta.begin(), dt.beta.end(), y.begin(), r2d);
//	    std::copy(dt.cL[M].begin(), dt.cL[M].end(), Z.begin());
		{
	    	Z.resize(y.size());
	    	msdat::coef_t::block_t blk = CF[M];
			for(data::row r = 0, re = y.size(); r < re; ++r)
			{
				Z[r].resize(x.size());
				for(data::col c = 0, ce = x.size(); c < ce; ++c)
					Z[r][c] = blk(r,c);
			}
		}
		auto [X, Y] = meshgrid(x, y);
		plot3(X, Y, Z, "-ob");
    }
	//marker(line_spec::marker_style::asterisk);
	title(ax1, sz_lbl); //);
	xlabel("α");
	ylabel("β");
	f->draw();
}

void import_and_save(const char* src,const char* dst)
{
    using namespace matplot;

	msdat::data_t dt;
	dt.load(src);

    aero::model mdl;
    mdl.setup(dt);
	mdl.save(dst);
}
void read_back(const char* src,const char* dst)
{
	rocket::msdat::data_t dt;
	dt.load(src);

	rocket::aero::model mdl;
    mdl.load(dst);
	show_coef(dt, mdl.R, mdl.F.cL, dt.cL, "c_L vs {α,β}");
	show_coef(dt, mdl.R, mdl.F.cD, dt.cD, "c_D vs {α,β}");
	show_coef(dt, mdl.R, mdl.F.cy, dt.cy, "c_y vs {α,β}");
	show_coef(dt, mdl.R, mdl.M.cl, dt.cl, "c_l vs {α,β}");
	show_coef(dt, mdl.R, mdl.M.cm, dt.cm, "c_m vs {α,β}");
	show_coef(dt, mdl.R, mdl.M.cn, dt.cn, "c_n vs {α,β}");
}

}

bool aero_test()
{
	{
		const char src[] = "S2.dat";
		const char dst[] = "S2_aero.dat";
		import_and_save(src, dst);
	}
	{
		const char src[] = "S1.dat";
		const char dst[] = "S1_aero.dat";
		import_and_save(src, dst);
		read_back(src, dst);
	}
    return true;
}

bool atm_test()
{
	char ctx[64]; *ctx = 0;
	using std::abs;
	test_handler_t tst = ctx;

	float h[5] = {   1000,  12400,   26000,      67000,     240000 };
	float t[5] = { 281.66, 216.66,  222.66,     225.86,     186.96 };
	float a[5] = { 336.4089594526281, 295.0491281125908, 299.1066498759263, 301.2483161778668, 274.0812434297538 };
	float p[5] = {  89872.95590765662, 18144.29645752371, 2151.92282479313, 7.34892808956, 0.37272 };
	float r[5] = { 1.111787928110332, 0.291796109794915, 0.033674611230403, 0.000113371208793, 1.03454e-10 };

	float t_tol = 1e-2;
	float a_tol = 1e-2;
	float p_tol = 1e-2;
	float r_tol = 1e-6;
	for(unsigned k=0; k<5; ++k)
	{
		sprintf(ctx, "sample %d", k);
		air_t air(h[k]);
		TestAssert(abs(air.T   - t[k]) < t_tol);
		TestAssert(abs(air.snd - a[k]) < a_tol);
		TestAssert(abs(air.rho - r[k]) < r_tol);
		if(k < 4)
		{
			TestAssert(abs(air.po  - p[k]) < p_tol);
		}
		else
		{
			std::cout << "H = " << h[k]
				      << " m, air.po  = " << air.po
					  << ", p = " << p[k] << std::endl;
		}
	}
	return true;
}



