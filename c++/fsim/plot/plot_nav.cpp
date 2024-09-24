/*============================================================================*/
/*                                                                            */
/*============================================================================*/

////////////////////////////////////////////////////////////////////////////////
/// \brief
/// \author   Augusto Zumarraga
/// \date     creación: 10/06/2024
/// \date     revisión: 20/08/2024
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
#include "plot_read.hpp"
#include <dgnc/store/csv_file.hpp>
#include <fstream>
#include <functional>

#ifndef FSIM_NO_PLOT
	#include <matplot/matplot.h>
#endif

using namespace dgnc;
using namespace fsim;

//==============================================================================
void fsim::plot_nav( const solver_t::result_t& sim
		           , unsigned mark
				   , const std::string& exp_file, bool do_plot)
{
    using namespace std::placeholders;

    const solver_t::result_t::times_t& t = sim.time();
    size_t N = t.size();
    dbl_vect H(N), r(N);
    dbl_vect V(N), A(N), B(N);
    dbl_vect La(N), Ln(N);
    dbl_vect Xe(N), Ye(N), Ze(N);


    std::vector<nav_data_t> nav(N);
    std::transform(sim.state().begin(), sim.state().end(), nav.begin(), get_nav);
    std::transform(sim.state().begin(), sim.state().end(),  Xe.begin(), get_xe );
    std::transform(sim.state().begin(), sim.state().end(),  Ye.begin(), get_ye );
    std::transform(sim.state().begin(), sim.state().end(),  Ze.begin(), get_ze );
    std::transform(sim.state().begin(), sim.state().end(),   H.begin(), get_hgt);

    navs::ecef::lla_t space_port = nav[0].lla;

    std::transform(nav.begin(), nav.end(),  r.begin(), std::bind(get_rng, _1, space_port));
    std::transform(nav.begin(), nav.end(),  A.begin(), get_gamma);
    std::transform(nav.begin(), nav.end(),  B.begin(), get_beta );
    std::transform(nav.begin(), nav.end(), La.begin(), get_lat);
    std::transform(nav.begin(), nav.end(), Ln.begin(), get_lng);
    std::transform(nav.begin(), nav.end(),  V.begin(), get_vel);

#ifndef FSIM_NO_PLOT
	if(do_plot)
	{
    using namespace matplot;
    std::vector<double> tk, vk;
    tk.push_back(t[mark]);
    vk.push_back(0);

    auto f = figure(true);
    tiledlayout(3, 3);
    {
    	vk.front() = H[mark];
		auto ax = nexttile();
		auto p  = plot(ax, t, H, tk, vk, "o");
		p[0]->line_width(2);
		ylabel(ax, "altura orbital [km]");
		grid(on);
    }
    {
    	vk.front() = r[mark];
		auto ax = nexttile();
		auto p  = plot(ax, t, r, tk, vk, "o");
		p[0]->line_width(2);
		ylabel(ax, "rango [km]");
		grid(on);
    }
    {
    	vk.front() = V[mark];
		auto ax = nexttile();
		auto p  = plot(ax, t, V, tk, vk, "o");
		p[0]->line_width(2);
		ylabel(ax, "velocidad [m/s]");
		xlabel(ax, "t [s]");
		grid(on);
    }
    {
    	vk.front() = A[mark];
		auto ax = nexttile();
		auto p  = plot(ax, t, A, t, B, tk, vk, "o");
		p[0]->line_width(2);
		p[1]->line_width(2);
		ylabel(ax, "[⁰]");
		xlabel(ax, "t [s]");
		auto l = legend({"γ", "β"});
		l->location(legend::general_alignment::bottomleft);
		grid(on);
    }
    {
		auto ax = nexttile();
	    auto g = geoplot(La, Ln, ".");
	    g->line_width(1);
	    double lng[2] = { Ln.front(), Ln.front()}
	         , lat[2] = { La.front(), La.front()};
	    for(dbl_vect::iterator i = Ln.begin(), e = Ln.end(); i < e; ++i)
	    {
	    	if( lng[0] > *i)
	    		lng[0] = *i;
	    	else
			if( lng[1] < *i)
				lng[1] = *i;
	    }
	    for(dbl_vect::iterator i = La.begin(), e = La.end(); i < e; ++i)
	    {
	    	if( lat[0] > *i)
	    		lat[0] = *i;
	    	else
			if( lat[1] < *i)
				lat[1] = *i;
	    }
	    lat[0] -= 5; lat[1] += 5;
	    lng[0] -= 5; lng[1] += 5;
	    double r_lat = (lat[1] - lat[0])/0.4;
	    double r_lng =  lng[1] - lng[0];
	    if(r_lat > r_lng)
	    {
	    	double d = (r_lat - r_lng)/2;
	    	lng[0] -= d;
	    	lng[1] += d;
	    }
	    else
	    {
	    	double d = (r_lng - r_lat)/2;
	    	lat[0] -= d;
	    	lat[1] += d;
	    }
	    geolimits({lat[0], lat[1]}, {lng[0], lng[1]});
	    grid(on);
    }
	{
		auto ax = nexttile();
		auto p = plot3(Xe, Ye, Ze);
		p->line_width(2);
		double R = wgs84::a*1e-3 + 500;
		xlim({-R, R});
		ylim({-R, R});
		zlim({-R, R});
		xlabel(ax, "x_e [km]");
		ylabel(ax, "y_e [km]");
		zlabel(ax, "z_e [km]");
	}
    {
		auto ax = nexttile();
		auto p  = plot(ax, Ye, Xe, Ye, Ze);
		p[0]->line_width(2);
		p[1]->line_width(2);
		xlabel(ax, "y_e [km]");
		auto l = legend({"x_e", "z_e"});
		l->location(legend::general_alignment::bottomleft);
		grid(on);
    }
    f->position({10, 10, 1800, 1000});
 	//f->size(1800, 1000);
    f->draw();
	}
#endif

    if(!exp_file.empty())
    {
    	itr_vect its;
    	its.push_back(H .begin());
    	its.push_back(r .begin());
    	its.push_back(V .begin());
    	its.push_back(A .begin());
    	its.push_back(B .begin());
    	its.push_back(La.begin());
    	its.push_back(Ln.begin());
    	its.push_back(Xe.begin());
    	its.push_back(Ye.begin());
    	its.push_back(Ze.begin());
    	std::ofstream s(exp_file);
    	csv::write(s, t, its);

    	std::cout << "NAV data exported to <" << exp_file << ">\n";
    }
}




