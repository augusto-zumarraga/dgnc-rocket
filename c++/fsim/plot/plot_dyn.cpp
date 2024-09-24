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
void fsim::plot_dyn( const solver_t::result_t& sim
		           , const sim_record_t& inf
				   , unsigned mark
				   , const std::string& exp_file, bool do_plot)
{
    using namespace std::placeholders;

    const solver_t::result_t::times_t& t = sim.time();
    size_t N = t.size();
    dbl_vect roll (N);
    dbl_vect pitch(N);
    dbl_vect yaw  (N);
    dbl_vect dy   (N);
    dbl_vect dz   (N);
    dbl_vect da   (N);
    dbl_vect rc   (N);
    dbl_vect Q    (N);
    dbl_vect M    (N);
    dbl_vect aoa  (N);
    dbl_vect slp  (N);
    dbl_vect fx(N), fy(N), fz(N);
    dbl_vect wx(N), wy(N), wz(N);
    dbl_vect T    (N);
    dbl_vect mass (N);
    dbl_vect flx_y(N), flx_z(N);
    dbl_vect flx_p(N), flx_r(N);

    std::vector<dyn_t> dyn(N);
    std::transform(sim.state().begin(), sim.state().end(), dyn.begin(), get_dyn);
    std::transform(sim.state().begin(), sim.state().end(), flx_y.begin(), get_flx_y);
    std::transform(sim.state().begin(), sim.state().end(), flx_z.begin(), get_flx_z);
    std::transform(sim.state().begin(), sim.state().end(), flx_p.begin(), get_flx_p);
    std::transform(sim.state().begin(), sim.state().end(), flx_r.begin(), get_flx_r);

    std::transform(dyn.begin(), dyn.end(), roll .begin(), get_roll );
    std::transform(dyn.begin(), dyn.end(), pitch.begin(), get_pitch);
    std::transform(dyn.begin(), dyn.end(), yaw  .begin(), get_yaw  );
    std::transform(dyn.begin(), dyn.end(), dy   .begin(), get_tvc_y);
    std::transform(dyn.begin(), dyn.end(), dz   .begin(), get_tvc_z);
    std::transform(dyn.begin(), dyn.end(), da   .begin(), get_ail  );

    std::transform(inf.begin(), inf.end(), Q    .begin(), get_Q    );
    std::transform(inf.begin(), inf.end(), M    .begin(), get_M    );
    std::transform(inf.begin(), inf.end(), aoa  .begin(), get_aoa  );
    std::transform(inf.begin(), inf.end(), slp  .begin(), get_slp  );
    std::transform(inf.begin(), inf.end(), rc   .begin(), get_rcs  );

    std::transform(inf.begin(), inf.end(), fx   .begin(), get_fx   );
    std::transform(inf.begin(), inf.end(), fy   .begin(), get_fy   );
    std::transform(inf.begin(), inf.end(), fz   .begin(), get_fz   );
    std::transform(inf.begin(), inf.end(), wx   .begin(), get_wx   );
    std::transform(inf.begin(), inf.end(), wy   .begin(), get_wy   );
    std::transform(inf.begin(), inf.end(), wz   .begin(), get_wz   );
    std::transform(inf.begin(), inf.end(), T    .begin(), get_T    );
    std::transform(inf.begin(), inf.end(), mass .begin(), get_mass );

#ifndef FSIM_NO_PLOT
	if(do_plot)
	{
    using namespace matplot;
    std::vector<double> tk, vk;

    auto f = figure(true);
    tiledlayout(3, 3);

    {
        tk.assign(3, t[mark]);
        vk.assign(3, 0); vk[0] = roll[mark]; vk[1] = pitch[mark]; vk[2] = yaw[mark];
		auto ax = nexttile();
		auto p  = plot(ax, t, roll, t, pitch, t, yaw, tk, vk, "o");
		p[0]->line_width(2);
		p[1]->line_width(2);
		p[2]->line_width(2);
//		p[3]->line_width(0);
//		p[3]->marker(line_spec::marker_style::square).marker_size(4);
		auto l = legend({"ϕ", "θ", "ψ"});
		ylabel(ax, "[⁰]");
		xlabel(ax, "t [s]");
		l->location(legend::general_alignment::bottomleft);
		grid(on);
    }
    {
        tk.assign(2, t[mark]);
        vk.assign(2, 0); vk[0] = aoa[mark]; vk[1] = slp[mark];
		auto ax = nexttile();
		auto p  = plot(ax, t, aoa, t, slp, tk, vk, "o");
		p[0]->line_width(2);
		p[1]->line_width(2);
		auto l = legend({"α", "β"});
		ylabel(ax, "[⁰]");
		xlabel(ax, "t [s]");
		l->location(legend::general_alignment::bottomleft);
		grid(on);
    }
    {
        tk.assign(3, t[mark]);
        vk.assign(3, 0); vk[0] = dy[mark]; vk[1] = dz[mark]; vk[2] = da[mark];
		auto ax = nexttile();
		auto p  = plot(ax, t, dy, t, dz, t, da, t, rc, tk, vk, "o");
		p[0]->line_width(2);
		p[1]->line_width(2);
		p[2]->line_width(2);
		p[3]->line_width(2);
		auto l = legend({"δ_y", "δ_z", "δ_a", "rcs"});
		xlabel(ax, "t [s]");
		l->location(legend::general_alignment::bottomleft);
		grid(on);
    }
    {
        tk.assign(1, t[mark]);
        vk.assign(1, M[mark]);
        auto ax = nexttile();
		auto p  = plot(ax, t, M);
		p->line_width(2);
		ylabel(ax, "M");
		xlabel(ax, "t [s]");
		grid(on);
    }
    {
        vk.assign(1, Q[mark]);
		auto ax = nexttile();
		auto p  = plot(ax, t, Q, tk, vk, "o");
		p[0]->line_width(2);
		ylabel(ax, "Q [kgf/cm²]");
		xlabel(ax, "t [s]");
		grid(on);
    }
    {
        tk.assign(3, t[mark]);
        vk.assign(3, 0); vk[0] = fx[mark]; vk[1] = fy[mark]; vk[2] = fz[mark];
        auto ax = nexttile();
		auto p  = plot(ax, t, fx, t, fy, t, fz, tk, vk, "o");
		p[0]->line_width(2);
		p[1]->line_width(2);
		p[2]->line_width(2);
		auto l = legend({"f_x", "f_y", "f_z"});
		ylabel(ax, "[N]");
		xlabel(ax, "t [s]");
		l->location(legend::general_alignment::center);
		grid(on);
    }
    {
        tk.assign(2, t[mark]);
        vk.assign(2, 0); vk[0] = mass[mark]; vk[1] = T[mark];
		auto ax = nexttile();
		auto p  = plot(ax, t, mass, t, T, tk, vk, "o");
		p[0]->line_width(2);
		p[1]->line_width(2);
		auto l = legend({"mass [kg]", "T [kgf]"});
		xlabel(ax, "t [s]");
//		l->location(legend::general_alignment::bottomleft);
		grid(on);
    }
    {
		auto ax = nexttile();
    }
    {
        tk.assign(3, t[mark]);
        vk.assign(3, 0); vk[0] = wx[mark]; vk[1] = wy[mark]; vk[2] = wz[mark];
        auto ax = nexttile();
		auto p  = plot(ax, t, wx, t, wy, t, wz, tk, vk, "o");
		p[0]->line_width(2);
		p[1]->line_width(2);
		p[2]->line_width(2);
		auto l = legend({"w_x", "w_y", "w_z"});
		ylabel(ax, "[1/s]");
		xlabel(ax, "t [s]");
		//l->location(legend::general_alignment::center);
		grid(on);
    }


    f->position({10, 10, 1800, 1000});
// 	f->size(1800, 1000);
    f->draw();
	}
#endif

    if(!exp_file.empty())
    {
    	itr_vect its;
    	its.push_back(roll .begin());
    	its.push_back(pitch.begin());
    	its.push_back(yaw  .begin());
    	its.push_back(dy   .begin());
    	its.push_back(dz   .begin());
    	its.push_back(da   .begin());
    	its.push_back(rc   .begin());
    	its.push_back(Q    .begin());
    	its.push_back(M    .begin());
    	its.push_back(aoa  .begin());
    	its.push_back(slp  .begin());
    	its.push_back(fx   .begin());
    	its.push_back(fy   .begin());
    	its.push_back(fz   .begin());
    	its.push_back(T    .begin());
    	its.push_back(mass .begin());

    	std::ofstream s(exp_file);
    	csv::write(s, t, its);

    	std::cout << "DYN data exported to <" << exp_file << ">\n";
    }
}



