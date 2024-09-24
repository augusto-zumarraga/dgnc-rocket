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
void fsim::plot_gnc( const solver_t::result_t& sim
		           , const sim_record_t& inf
		           , const fcc_record_t& tmy
				   , unsigned mark
				   , const std::string& exp_file, bool do_plot)
{
    using namespace std::placeholders;

    const solver_t::result_t::times_t& t = sim.time();
    size_t N = t.size();

    std::vector<gnc_t> gnc_data(N);
    std::transform(inf.begin(), inf.end(), gnc_data.begin(), get_sim);
    std::transform(tmy.begin(), tmy.end(), gnc_data.begin(), get_tmy);
    std::for_each(gnc_data.begin(), gnc_data.end(), std::mem_fn(&gnc_t::compute));

    dbl_vect ep(N), eq(N), er(N);
    std::transform(gnc_data.begin(), gnc_data.end(), ep.begin(), get_ep);
    std::transform(gnc_data.begin(), gnc_data.end(), eq.begin(), get_eq);
    std::transform(gnc_data.begin(), gnc_data.end(), er.begin(), get_er);


    dbl_vect Rx(N), Ry(N), Rz(N);
    dbl_vect Dx(N), Dy(N), Dz(N);
    dbl_vect Ex(N), Ey(N), Ez(N);
    dbl_vect eD(N);

    std::transform(gnc_data.begin(), gnc_data.end(), Rx.begin(), get_Rx);
    std::transform(gnc_data.begin(), gnc_data.end(), Ry.begin(), get_Ry);
    std::transform(gnc_data.begin(), gnc_data.end(), Rz.begin(), get_Rz);
    std::transform(gnc_data.begin(), gnc_data.end(), Dx.begin(), get_Dx);
    std::transform(gnc_data.begin(), gnc_data.end(), Dy.begin(), get_Dy);
    std::transform(gnc_data.begin(), gnc_data.end(), Dz.begin(), get_Dz);
    std::transform(gnc_data.begin(), gnc_data.end(), Ex.begin(), get_Ex);
    std::transform(gnc_data.begin(), gnc_data.end(), Ey.begin(), get_Ey);
    std::transform(gnc_data.begin(), gnc_data.end(), Ez.begin(), get_Ez);

    std::transform(gnc_data.begin(), gnc_data.end(), eD.begin(), get_eD);

#ifndef FSIM_NO_PLOT
	if(do_plot)
	{
    using namespace matplot;
    std::vector<double> tk, vk;

    auto f = figure(true);
    tiledlayout(3, 3);

    {
        dbl_vect t1, e_roll(mark), e_pitch(mark), e_yaw(mark);

        t1.insert(t1.begin(), t.begin(), t.begin()+mark);
        std::transform(gnc_data.begin(), gnc_data.begin()+mark, e_roll .begin(), get_e_roll );
        std::transform(gnc_data.begin(), gnc_data.begin()+mark, e_pitch.begin(), get_e_pitch);
        std::transform(gnc_data.begin(), gnc_data.begin()+mark, e_yaw  .begin(), get_e_yaw  );

		auto ax = nexttile();
		auto p  = plot(ax, t1, e_roll, t1, e_pitch, t1, e_yaw);
		p[0]->line_width(2);
		p[1]->line_width(2);
		p[2]->line_width(2);
		legend({"ϕ", "θ", "ψ"});
		ylabel(ax, "[⁰]");
		xlabel(ax, "t [s]");
		grid(on);

    }
    {
        tk.assign(3, t[mark]);
        vk.assign(3, 0); vk[0] = ep[mark]; vk[1] = eq[mark]; vk[2] = er[mark];
		auto ax = nexttile();
		auto p  = plot(ax, t, ep, t, eq, t, er, tk, vk, "o");
		p[0]->line_width(2);
		p[1]->line_width(2);
		p[2]->line_width(2);
		auto l = legend({"e_p", "e_q", "e_r"});
		ylabel(ax, "[⁰/s]");
		xlabel(ax, "t [s]");
		l->location(legend::general_alignment::bottomleft);
		grid(on);
    }
    {
        tk.assign(1, t[mark]);
        vk.assign(1, eD[mark]);
        auto ax = nexttile();
		auto p  = plot(ax, t, eD);
		p->line_width(2);
		ylabel(ax, "e_dir [⁰]");
		xlabel(ax, "t [s]");
		grid(on);
    }
    {
        tk.assign(2, t[mark]);
        vk.assign(2, 0); vk[0] = Rx[mark]; vk[1] = Dx[mark];
		auto ax = nexttile();
		auto p  = plot(ax, t, Rx, t, Dx, tk, vk, "o");
		p[0]->line_width(2);
		p[1]->line_width(2);
		auto l = legend({"R_x", "D_x"});
		xlabel(ax, "t [s]");
		grid(on);
    }
    {
        tk.assign(2, t[mark]);
        vk.assign(2, 0); vk[0] = Ry[mark]; vk[1] = Dy[mark];
		auto ax = nexttile();
		auto p  = plot(ax, t, Ry, t, Dy, tk, vk, "o");
		p[0]->line_width(2);
		p[1]->line_width(2);
		auto l = legend({"R_y", "D_y"});
		xlabel(ax, "t [s]");
		grid(on);
    }
    {
        tk.assign(2, t[mark]);
        vk.assign(2, 0); vk[0] = Rz[mark]; vk[1] = Dz[mark];
		auto ax = nexttile();
		auto p  = plot(ax, t, Rz, t, Dz, tk, vk, "o");
		p[0]->line_width(2);
		p[1]->line_width(2);
		auto l = legend({"R_z", "D_z"});
		xlabel(ax, "t [s]");
		grid(on);
    }
    {
        tk.assign(2, t[mark]);
        vk.assign(2, 0); vk[0] = Ey[mark]; vk[1] = Ez[mark];
		auto ax = nexttile();
		auto p  = plot(ax, t, Ey, t, Ez, tk, vk, "o");
		p[0]->line_width(2);
		p[1]->line_width(2);
		auto l = legend({"e_y", "e_z"});
		xlabel(ax, "t [s]");
		grid(on);
    }

    f->position({10, 10, 1800, 1000});
    f->draw();
	}
#endif

    if(!exp_file.empty())
    {
    	itr_vect its;

    	// referencia de dirección
    	its.push_back(Rx.begin());
    	its.push_back(Ry.begin());
    	its.push_back(Rz.begin());
    	// dirección
    	its.push_back(Dx.begin());
    	its.push_back(Dy.begin());
    	its.push_back(Dz.begin());
    	// Error de dirección
    	its.push_back(Ex.begin());
    	its.push_back(Ey.begin());
    	its.push_back(Ez.begin());
    	// norma del error de dirección
    	its.push_back(eD.begin());

    	// error del cuaternión de actitud
        dbl_vect en(N), ex(N), ey(N), ez(N);
        std::transform(gnc_data.begin(), gnc_data.end(), en.begin(), get_en);
        std::transform(gnc_data.begin(), gnc_data.end(), ex.begin(), get_ex);
        std::transform(gnc_data.begin(), gnc_data.end(), ey.begin(), get_ey);
        std::transform(gnc_data.begin(), gnc_data.end(), ez.begin(), get_ez);
    	its.push_back(en.begin());
    	its.push_back(ex.begin());
    	its.push_back(ey.begin());
    	its.push_back(ez.begin());

    	// error de velocidad angular
    	its.push_back(ep.begin());
    	its.push_back(eq.begin());
    	its.push_back(er.begin());

        dbl_vect pst(N), tgo(N), rgo(N), rbs(N);
        std::transform(gnc_data.begin(), gnc_data.end(), pst.begin(), get_pst);
        std::transform(gnc_data.begin(), gnc_data.end(), tgo.begin(), get_tgo);
        std::transform(gnc_data.begin(), gnc_data.end(), rgo.begin(), get_rgo);
        std::transform(gnc_data.begin(), gnc_data.end(), rbs.begin(), get_rbs);
    	its.push_back(pst.begin());
    	its.push_back(tgo.begin());
    	its.push_back(rgo.begin());
    	its.push_back(rbs.begin());

    	std::ofstream s(exp_file);
    	csv::write(s, t, its);

    	std::cout << "GNC data exported to <" << exp_file << ">\n";
    }
}


