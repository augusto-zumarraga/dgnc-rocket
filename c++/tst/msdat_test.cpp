/*============================================================================*/
/*                                                                            */
/*============================================================================*/

////////////////////////////////////////////////////////////////////////////////
/// \brief
/// \author   Augusto Zumarraga
/// \date     creación: 09/06/2024
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
#include <dgnc/rocket/data/msdat.hpp>

using namespace dgnc;

void show_coef(const rocket::msdat::data_t dt)
{
	using namespace rocket;
	using namespace msdat;

    mch M(0);
    slp b(0);
    aoa a(1);

    values_t a_deg = dt.alfa * (180.0/3.1415926535898);

    using namespace matplot;

    auto f = figure(true);

    tiledlayout(2, 2);
    auto ax1 = nexttile();
    auto p1  = plot(ax1, dt.alfa, dt.cx.get(M,b), dt.alfa, dt.cy.get(M,b)); //, dt.alfa, dt.cz.get(M,b));
//    p1[0]->line_width(2);
    p1[0]->marker(line_spec::marker_style::asterisk);
    p1[1]->marker(line_spec::marker_style::asterisk);
    //p1[2]->marker(line_spec::marker_style::asterisk);
    legend({"c_X", "c_Y"}); //, "c_Z"});
    title(ax1, "c_f vs α");
    ylabel(ax1, "c_f");
    grid(on);

    auto ax2 = nexttile();
    auto p2  = plot(ax2, dt.alfa, dt.cl.get(M,b), dt.alfa, dt.cm.get(M,b), dt.alfa, dt.cn.get(M,b));
    p2[0]->marker(line_spec::marker_style::asterisk);
    p2[1]->marker(line_spec::marker_style::asterisk);
    p2[2]->marker(line_spec::marker_style::asterisk);
    legend({"c_L", "c_M", "c_N"});
    title(ax2, "c_m vs α");
    ylabel(ax2, "c_m");
    grid(on);

    //

    auto ax3 = nexttile();
    auto p3  = plot(ax3, dt.mach, dt.cx.get(a,b), dt.mach, dt.cy.get(a,b), dt.mach, dt.cz.get(a,b));
    p3[0]->marker(line_spec::marker_style::asterisk);
    p3[1]->marker(line_spec::marker_style::asterisk);
    p3[2]->marker(line_spec::marker_style::asterisk);
    title(ax3, "c_f vs M");
    ylabel(ax3, "c_f");
    grid(on);

    auto ax4 = nexttile();
    auto p4  = plot(ax4, dt.mach, dt.cl.get(a,b), dt.mach, dt.cm.get(a,b), dt.mach, dt.cn.get(a,b));
    p4[0]->marker(line_spec::marker_style::asterisk);
    p4[1]->marker(line_spec::marker_style::asterisk);
    p4[2]->marker(line_spec::marker_style::asterisk);
    title(ax4, "c_m vs M");
    ylabel(ax4, "c_m");
    grid(on);

    f->draw();
}
void import_and_save(const char* src, const char* dst)
{
	using namespace rocket::msdat;
	data_t dt;
	dt.import(src);
	dt.save(dst);
	show_coef(dt);
}
void read_back(const char* src)
{
	using namespace rocket::msdat;

	data_t dt;
	dt.load(src);
	show_coef(dt);
}

bool msdat_test()
{
	{
	const char src[] = "/home/augusto/trabajos/DGNC/dev/rocket/mdat/N3/S1";
	const char dst[] = "S1.dat";
	import_and_save(src, dst);
	read_back(dst);
	}
	{
	const char src[] = "/home/augusto/trabajos/DGNC/dev/rocket/mdat/N3/S2";
	const char dst[] = "S2.dat";
	import_and_save(src, dst);
	}
    return true;
}

