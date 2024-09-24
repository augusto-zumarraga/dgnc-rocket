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
#include <matplot/matplot.h>
#include <dgnc/utest.hpp>

using namespace dgnc;
using namespace matplot;

typedef utest::handler_t test_handler_t;

namespace {
constexpr auto go = 9.80667;

inline double r2d(double r)
{
	constexpr auto f = 180.0/3.1415926535898;
	return r * f;
}
inline double m2km(double r)
{
	return r * 0.001;
}
inline double N2kgf(double r)
{
	return r / go;
}

void plot_column(const csv::column_t& t, const csv::column_t& v, const char* name)
{
    auto ax = nexttile();
    auto p  = plot(ax, t, v);
    p->line_width(2);
//    legend({"c_X", "c_Y"}); //, "c_Z"});
//    title(ax1, "c_f vs α");
    xlabel(ax, "t");
    ylabel(ax, name);
    grid(on);
}
void plot_columns( const csv::column_t& t
		         , const csv::column_t& v1, const char* n1
				 , const csv::column_t& v2, const char* n2
				 , const char* ylbl)
{
    auto ax = nexttile();
    auto p  = plot(ax, t, v1, t, v2);
    p[0]->line_width(2);
    p[1]->line_width(2);
    legend({n1, n2});
    xlabel(ax, "t");
    ylabel(ax, ylbl);
    grid(on);
}
void plot_columns( const csv::column_t& t
		         , const csv::column_t& v1, const char* n1
				 , const csv::column_t& v2, const char* n2
		         , const csv::column_t& v3, const char* n3
				 , const csv::column_t& v4, const char* n4
				 , const char* ylbl)
{
    auto ax = nexttile();
    auto p  = plot(ax, t, v1, t, v2, t, v3, t, v4);
    p[0]->line_width(2);
    p[1]->line_width(2);
//    p[2]->line_width(2);
//    p[3]->line_width(2);
    legend({n1, n2, n3, n4});
    xlabel(ax, "t");
    ylabel(ax, ylbl);
    grid(on);
}

}

bool fsim_test()
{
	test_handler_t tst;

	tst = "import csv";
 	csv::table_t data = csv::read("ascent_3Dd_v1.csv", ' ', 0);
 	TestAssert(data.size() == 39);

    csv::column_t& t = data[0];
    csv::column_t& h = data[16];
    csv::column_t& r = data[17];
    csv::column_t& y = data[21];
    csv::column_t& v = data[25];

    csv::column_t& Xa = data[26];
    csv::column_t& Ya = data[27];
    csv::column_t& Za = data[28];

    const csv::column_t& rho = data[30];
    const csv::column_t& vel = data[31];
    const csv::column_t& aoa = data[32];
    const csv::column_t& slp = data[33];
    const csv::column_t& mch = data[35];

	rocket::aero::model mdl;
    mdl.load("S1_aero.dat");

    csv::column_t xa; xa.reserve(Xa.size());
    csv::column_t ya; ya.reserve(Ya.size());
    csv::column_t za; za.reserve(Za.size());
    for(csv::column_t::const_iterator a = aoa.begin(), e = aoa.end()
       , b = slp.begin(), m = mch.begin(), r = rho.begin(), u = vel.begin()
       ; a < e; ++a, ++b, ++m, ++r, ++u)
    {
    	double Q = 0.5*(*r)*(*u)*(*u);
    	rocket::wind_t wnd(*a, *b, *m, *u, Q);
    	rocket::force_t F = mdl.force(wnd);
    	xa.push_back(F.x()/go);
    	ya.push_back(F.y()/go);
    	za.push_back(F.z()/go);
    }
	tst = "plot data";
 	auto f = figure(true);
    f->name("sim data");
    f->number_title(false);
    //f->color("gray");
    f->position({0, 0, 1400, 900});
 	f->size(1400, 900);


    tiledlayout(2, 3);

    std::transform(h.begin(), h.end(), h.begin(), m2km);
    std::transform(r.begin(), r.end(), r.begin(), m2km);
    std::transform(y.begin(), y.end(), y.begin(), r2d );

    std::transform(Xa.begin(), Xa.end(), Xa.begin(), N2kgf);
    std::transform(Ya.begin(), Ya.end(), Ya.begin(), N2kgf);
    std::transform(Za.begin(), Za.end(), Za.begin(), N2kgf);

    plot_column(t, h, "height [km]");
    plot_column(t, r, "range [km]");
    plot_columns(t, Xa, "X_a", xa, "x_a", "[kgf]");
    plot_column(t, v, "V [m/s]");
    plot_column(t, y, "γ [⁰]");
    plot_columns(t, Ya, "Y_a", Za, "Z_a", ya, "y_a", za, "z_a", "[kgf]");

    f->draw();
    f->font("Arial");
    f->font_size(24);
    f->title("Ascent data");
    return true;
}


