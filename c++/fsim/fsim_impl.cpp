/*============================================================================*/
/*                                                                            */
/*============================================================================*/

////////////////////////////////////////////////////////////////////////////////
/// \brief
/// \author   Augusto Zumarraga
/// \date     creación: 10/06/2024
/// \date     revisión: 25/09/2024
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

using namespace dgnc;

navs::orb::circular_t fsim::orbit;

//------------------------------------------------------------------------------
fsim::logger_t::logger_t(std::string fname)
{
	if(!fname.empty())
		m_out.open(fname);
}
fsim::logger_t& fsim::logger_t::operator<<(char s)
{
	if(s == '\n')
		std::cout << std::endl;
	if(m_out.good())
		m_out << s;
	return *this;
}

//------------------------------------------------------------------------------
void fsim::exec_t::update(const dgnc::fsim::sim_info_t& sim, gnc::ins_data_t& ins)
{
	ins.nav() = sim.ins;
	ins.lla = sim.env.lla;
	ins.wbi = sim.env.wbi; // + sim.flx.sfc
	ins.sfc = sim.env.fb ; // + sim.flx.sfc
	if(sim.flx)
	{
		ins.wbi += sim.flx.wbi;
		ins.sfc += sim.flx.sfc;
		ins.att  = ins.att * sim.flx.att;
	}
}

//------------------------------------------------------------------------------
bool fsim::exec_t::fts(const sim_info_t& s, logger_t& log)
{
	constexpr auto f_max = 10*9.81;
	if(s.env.lla.alt < 1)
	{
		log << s.ins.elapsed << " : GROUND HIT!!" << '\n' ;
		return false;
	}
	if(w_max && s.env.mdot > 1e-2)
	{
		if(std::abs(s.env.wbi.x()) > w_max)
		{
			log << s.ins.elapsed << " : EXCESSIVE ROLL RATE!!" << '\n' ;
			return false;
		}
		if(std::abs(s.env.wbi.y()) > w_max)
		{
			log << s.ins.elapsed << " : EXCESSIVE PITCH RATE!!" << '\n' ;
			return false;
		}
		if(std::abs(s.env.wbi.z()) > w_max)
		{
			log << s.ins.elapsed << " : EXCESSIVE YAW RATE!!" << '\n' ;
			return false;
		}
	}
	if(std::abs(s.env.fb.y()) > f_max || std::abs(s.env.fb.z()) > f_max)
	{
		log << s.ins.elapsed << " : EXCESSIVE FORCE!!" << '\n' ;
		return false;
	}
	return true;
}

//------------------------------------------------------------------------------
void fsim::exec_t::on_state(int st, const sim_info_t& p, logger_t& log)
{
	log << "\n\n============================================================\n"
			  << p.ins.elapsed << "s: STATE ";
	switch(st)
	{
	case fcc_t::st_init        : log << "init        "; break;
	case fcc_t::st_armed       : log << "armed       "; break;
	case fcc_t::st_ascent      : log << "ascent      "; break;
	case fcc_t::st_load_relief : log << "load_relief "; break;
	case fcc_t::st_meco        : log << "MECO        "; break;
	case fcc_t::st_coasting    : log << "coasting    "; break;
	case fcc_t::st_separation  : log << "separation  "; break;
	case fcc_t::st_fire_s2     : log << "S2 fire     "; break;
	case fcc_t::st_gravity_turn: log << "gravity_turn"; break;
	case fcc_t::st_steering    : log << "steering    "; break;
	case fcc_t::st_low_thrust  : log << "low_thrust  "; break;
	case fcc_t::st_engine_off  : log << "engine_off  "; break;
	case fcc_t::st_orbit       : log << "orbit       "; break;
	default:
		log << "steering/";
		switch(st - fcc_t::st_last)
		{
		case fcc_t::guide_t::st_init       : log << "init       "; break;
		case fcc_t::guide_t::st_pre_thrust : log << "pre_thrust "; break;
		case fcc_t::guide_t::st_pre_cycling: log << "pre_cycling"; break;
		case fcc_t::guide_t::st_ltg        : log << "LTG        "; break;
		case fcc_t::guide_t::st_terminal   : log << "terminal   "; break;
		case fcc_t::guide_t::st_orbit      : log << "orbit      "; break;
		}
	}
	{
		using namespace dgnc::navs;
		using dgnc::geom::euler;
		using dgnc::geom::polar;

		eci::state_t si = p.ins;
		polar lt = orbit(eci::lla_t(si.pos)) << si.vel;

		ned::att_t qned = ecef_to_ned(p.env.lla, p.ins.att);
		euler eul(qned);
		eul.normalize(degree(0.1), degree(0.001));

		log << "\n\n"
		    << "latitud "  << degree(p.env.lla.lat) << ", "
		    << "longitud " << degree(p.env.lla.lng) << ", "
		    << "altura "   << p.env.lla.alt * 1e-3 << "km\n"
		    // no se por que en el namespace anónimo no encuentra << para euler
		    << "actitud {ϕ,θ,ψ}: " << eul << "\n"
		    << "velocidad " << lt
		    ;
		if(st > fcc_t::st_steering)
		{
			orb::elements_t o(orb::state_t(si.pos, si.vel));
			log << "\norbit: apogeo: " << o.apogee()*1e-3
					  << " km, e:" << o.eccentricity()
					  << ", inc:" << degree(o.inclination());
		}
	}
	log << "\n============================================================"
	    << '\n';
}

void fsim::exec_t::dump(const std::string& exp, const solver_t::result_t& r)
{
	std::ofstream s(exp);
	for(unsigned k = 0, N = r.num_samples(); k < N; ++k)
		s << r.time(k)        << ','
		  << r.state(k).nav() << ','
		  << r.state(k).rot() << ','
		  << r.state(k).frc()
		  << std::endl;
}


//------------------------------------------------------------------------------
std::ostream& operator<<(std::ostream& s, const fsim::sim_info_t& x)
{
	using namespace dgnc;
	using namespace navs;
	using geom::degree;

	eci::state_t si = x.ins;
	geom::polar  lt = fsim::orbit(eci::lla_t(si.pos)) << si.vel;
	orb::elements_t o(orb::state_t(si.pos, si.vel));

	s << "latitud "        << degree(x.env.lla.lat) << "⁰, "
	  << "longitud "       << degree(x.env.lla.lng) << "⁰, "
	  << "altura  "        << x.env.lla.alt * 1e-3  << " km\n"
	  << "velocidad (ECI)" << lt.mod << " m/s, "
	  << "elevación "      << degree(lt.elv) << "⁰, "
	  << "rumbo "          << degree(lt.hdn) << "⁰\n"
	  << "masa "           << x.env.mass.m << "[" << fsim::min_mass << "] kg"
      <<  "\norbit: apogeo: " << o.apogee()*1e-3
	  << " km, e:" << o.eccentricity()
	  << " (" << o.eccentricity()*o.semimajor_axis()*1e-3
	  << " km), inc:" << degree(o.inclination())
	  ;
	return s;
}




