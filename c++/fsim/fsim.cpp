/*============================================================================*/
/*                                                                            */
/*============================================================================*/

////////////////////////////////////////////////////////////////////////////////
/// \brief
/// \author   Augusto Zumarraga
/// \date     creación: 10/06/2024
/// \date     revisión: 18/09/2024
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
#include "fsim.hpp"
#include "fsim_impl.hpp"
#include "plot/plot.hpp"
#include <dgnc/store/ini_file.hpp>
#include <dgnc/rocket/dyn/rigid_body.hpp>

//==============================================================================
dgnc::fsim::exec_t::exec_t(std::string fpath)
: pln(fpath)
{
	if(!pln.empty() && pln.back() != '/')
		pln += '/';
	data::ini_file f((pln + "params.ini").c_str());

	tint    = atof(f.branch("simulation.integration time").get().c_str());
	tsim    = atof(f.branch("simulation.duration"   ).get().c_str());
	toff    = atof(f.branch("simulation.start time" ).get().c_str());
	tlaunch = atof(f.branch("simulation.launch time").get().c_str());

	mdl  = f.branch("simulation.model root"  ).get();
	exp  = f.branch("simulation.export root" ).get();
	wind = f.branch("simulation.wind profile").get();
	std::string ss = f.branch("simulation.separation rot").get();
	if(ss.empty())
		sep_rot = 0;
	else
	{
		double p, q, r;
		std::sscanf(ss.c_str(), "%lf%lf%lf", &p, &q, &r);
		sep_rot.x() = math::d2r (p);
		sep_rot.y() = math::d2r(q);
		sep_rot.z() = math::d2r(r);
	}

	if(mdl.empty())
		mdl = "data/S";
	if(!tsim)
		tsim = 500;
	if(!tint)
		tint = 0.001;
}


//------------------------------------------------------------------------------
bool dgnc::fsim::exec_t::operator()(bool do_plot)
{
	if(exp.empty())
		do_plot = true;
	gnc::fcc_t fcc;
	logger_t log(exp.empty() ? exp : exp + "_log.txt");

	//________________________________________________________ Pre-Flight setup
	rocket::plan_t plan(pln);
	log << plan << '\n';

	plan.setup(fcc);
	orbit = plan.orbit();

	//______________________________________________________ SIM Inicialization
	if(tsim == 0)
		tsim = plan.seco();
	double ts = plan.sample_time();

	solver_t     slv;
	rocket_mdl_t S1((mdl + "1").c_str());
	rocket_mdl_t S2((mdl + "2").c_str());

	if(!wind.empty())
		S1.wind().import(wind.c_str());
	S1.add_roll_fin();
	S2.add_roll_rcs();

	unsigned Nk = 1/ts, N = tsim*Nk;

	solver_t::result_t sim(N);
	sim_record_t       sim_rec;
	fcc_record_t       fcc_rec;
	gnc::ins_data_t    fcc_sns;

	rocket_mdl_t::vect_t xo;
	xo.elapsed = toff;
	S1.init(xo, plan.launch_state());

	sim_rec.reserve(N);
	fcc_rec.reserve(N);
	sim.push(0, xo);
	rocket_mdl_t* p_stage = &S1;

	int st = fcc.state_trace();
	unsigned mark = 0;
	//________________________________________________________________ SIM Loop

	for(unsigned k=0; k<N; ++k)
	{
		double t = k * ts + toff;
		if(!(k%Nk))
		{
			std::cerr << '.' ;
			if(!(k%(10*Nk)))
				std::cerr << '\n' << t << ' ';
		}

		xo.elapsed = t; // para mitigar errores numéricos
		p_stage->sample_begin(t, xo, ts);

		// registro de los resultados de la simulación
		sim_rec.push_back(p_stage->sim_info());
		// terminación temprana de la simulación
		if(!fts(sim_rec.back(), log))
			break;

		// ------------------------------------------------------- SAMPLE STEP

		update(sim_rec.back(), fcc_sns); // simular las mediciones
		if(t >= tlaunch)
			fcc.launch(fcc_sns);
		fcc.on_time(fcc_sns);            // ejecutar el control de vuelo
		fcc.update_tlmy();               // actualizar la telemetría
		fcc_rec.push_back(fcc.tlmy());

		if(st != fcc.state_trace())
		{
			// reportar cambios de estado
			st = fcc.state_trace();
			on_state(st, sim_rec.back(), log);
		}
		// propagar los comandos
		p_stage->acts().tvc(fcc.AOs().dy , fcc.AOs().dz);
		p_stage->acts().ail(fcc.AOs().da );
		p_stage->acts().rcs(fcc.AOs().rc );
		p_stage->acts().eng(fcc.DOs().eng);

		// --------------------------------------------------- EVENTS HANDLING
		if(p_stage == &S2)
		{
			if(fcc.DOs().sep && fcc.plan().separation_completed(t))
			{
				fcc.DOs().sep = false;
				xo.wbi += sep_rot;
				log << '\n' << t << " : SEPARATION COMPLETED" << '\n';
			}
			if(fcc.DOs().rel)
			{
				fcc.on_release();
				double me = p_stage->mass().extra.eject();
				xo.mass -= me;
				log << '\n'  << t << " : MASS RELEASE (" << me
					<< "kg)" << '\n';
			}
		}
		else
		if(fcc.DOs().sep)
		{
			mark = k;
			p_stage = &S2;
			S2.init(xo);
			log << '\n' << t << " : SEPARATION START" << '\n';
		}

		// --------------------------------------------------------- TIME STEP
		solver_t::times_t  tm(t, t+ts, tint);
		solver_t::result_t res = slv(*p_stage, xo, tm);
		t += ts;
		xo = res.state().back();
		xo.att.normalize();
		assert(std::abs(xo.elapsed - second_t(t)) < 1e-3);
		xo.elapsed = t;
		p_stage->sample_end(t, xo, ts);
		sim.push(t, xo);
	}

	//__________________________________________________________________ Epilog

	log << "\n\n************************************************************\n\n"
		<< sim_rec.back()
		<< "\n\n************************************************************\n"
		<< '\n';

	std::string fdyn, fnav, fgnc;
	if(!exp.empty())
	{
		// Export results
		dump  (exp + "_sim.csv", sim);
		fdyn = exp + "_dyn.csv";
		fnav = exp + "_nav.csv";
		fgnc = exp + "_gnc.csv";

    	std::cout << "SIM data exported to <" << exp << "_sim.csv>\n";
	}
	{
		// Plot results
		plot_gnc(sim, sim_rec, fcc_rec, mark, fgnc, do_plot);
		plot_dyn(sim, sim_rec, mark, fdyn, do_plot);
		plot_nav(sim, mark, fnav, do_plot);
	}
	return do_plot;
}



