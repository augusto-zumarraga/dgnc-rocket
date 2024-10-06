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

bool dgnc::fsim::run(const std::string& fpath, bool plot)
{
	exec_t runner(fpath);
	return runner(plot);
}
const char* dgnc::fsim::version()
{
	return "FSIM Rocket v1.0.0 [" __DATE__ " - " __TIME__ "]";
}


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

	std::string
	mdl    = f.branch("simulation.model root"  ).get();
	mdl_s1 = f.branch("simulation.stage_1"     ).get();
	mdl_s2 = f.branch("simulation.stage_2"     ).get();
	exp    = f.branch("simulation.export root" ).get();
	wind   = f.branch("simulation.wind profile").get();
	w_max  = math::d2r(atof(f.branch("simulation.max angular rate").get().c_str()));

	if( mdl.empty())
		mdl = "data/S";
	if( mdl_s1.empty())
		mdl_s1 = mdl + '1';
	else
		mdl_s1 = mdl + mdl_s1;
	if( mdl_s2.empty())
		mdl_s2 = mdl + '2';
	else
		mdl_s2 = mdl + mdl_s2;

	std::string ss = f.branch("simulation.separation rot").get();
	if(ss.empty())
		sep_rot = 0;
	else
	{
		double p, q, r;
		std::sscanf(ss.c_str(), "%lf%lf%lf", &p, &q, &r);
		sep_rot.x() = math::d2r(p);
		sep_rot.y() = math::d2r(q);
		sep_rot.z() = math::d2r(r);
	}

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
	logger_t log(exp.empty() ? exp : exp + "_log.txt");
	log << version() << '\n'
		<< std::string(strlen(version()), '-')
		<< "\n\n";

	log << "STAGE 1: " << mdl_s1 << '\n'
		<< "STAGE 2: " << mdl_s2 << '\n';
	if(!wind.empty())
	log << "WIND   : " << wind << '\n';

	//________________________________________________________ Pre-Flight setup
	gnc::fcc_loader_t loader(pln);
	log << '\n' << loader << '\n';

	gnc::fcc_t FCC;
	loader.setup(FCC);
	orbit = loader.orbit();

	//______________________________________________________ SIM Inicialization
	if  (tsim == 0)
		 tsim = loader.seco();
	double ts = loader.sample_time();

	solver_t slv;
	rocket_t rckt(mdl_s1, mdl_s2, wind, ts, log);
	rckt.init(toff, loader.launch_state(), sep_rot);

	//--------------------------------------------------------------------------
	unsigned Nk = 1/ts, N = tsim*Nk;
	solver_t::result_t sim(N);
	sim_record_t       sim_rec;
	fcc_record_t       fcc_rec;
	gnc::ins_data_t    fcc_sns;

	sim_rec.reserve(N);
	fcc_rec.reserve(N);
	sim.push(0, rckt.xo);

	int st = FCC.state_trace();
	//________________________________________________________________ SIM Loop

	for(unsigned k=0; k<N; ++k)
	{
		double t = k * ts + toff;
		rckt.xo.elapsed = t; // para mitigar errores numéricos
		rckt->sample_begin(t, rckt.xo, ts);

		// registro de los resultados de la simulación
		sim_rec.push_back(rckt->sim_info());
		// terminación temprana de la simulación
		if(!fts(sim_rec.back(), log))
			break;

		// ------------------------------------------------------- SAMPLE STEP
		update(sim_rec.back(), fcc_sns);     // simular las mediciones
		if(t >= tlaunch)
			FCC.arm(fcc_sns);
		FCC.on_time(fcc_sns);                // ejecutar el control de vuelo

		FCC.update_tlmy();                   // actualizar la telemetría
		fcc_rec.push_back(FCC.tlmy());

			if(st != FCC.state_trace())
			{
				// reportar cambios de estado
				st = FCC.state_trace();
				on_state(rckt.T, st, sim_rec.back(), log);
			}

		rckt.cmnds(t, FCC.AOs(), FCC.DOs()); // propagar los comandos

		//-------------------------------------------------------- TIME DISPLAY
		if(FCC.state_trace() == fcc_t::st_armed)
		{
			static int sec_last = 0;
			int sec_now = FCC.time_to_launch(t).value;
			if(sec_last != sec_now)
			{
				sec_last = sec_now;
				std::cerr << sec_now << " | ";
			}
		}
		else
		if(!(k%Nk))
		{
			std::cerr << '.' ;
			if(!(k%(10*Nk)))
				std::cerr << '\n' << t << ' ';
		}

		// --------------------------------------------------------- TIME STEP
		solver_t::times_t  tm(t, t+ts, tint);
		solver_t::result_t res = slv(rckt, rckt.xo, tm);
		t += ts;
		rckt.xo = res.state().back();
		rckt.xo.att.normalize();
		assert(std::abs(rckt.xo.elapsed - second_t(t)) < 1e-3);
		rckt.xo.elapsed = t;
		rckt->sample_end(t, rckt.xo, ts);
		sim.push(t, rckt.xo);
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
		plot_gnc(sim, sim_rec, fcc_rec, rckt.mark, fgnc, do_plot);
		plot_dyn(sim, sim_rec, rckt.mark, fdyn, do_plot);
		plot_nav(sim, rckt.mark, fnav, do_plot);
	}
	return do_plot;
}



