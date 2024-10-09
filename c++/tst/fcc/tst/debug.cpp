/*============================================================================*/ 
/*                                                                            */
/*============================================================================*/ 

/*!                                                                             
 \file     debug.cpp
 \brief                                                                         
 \author   Augusto Zumarraga                                                    
 \date     creación: 07/08/2024
 \date     revisión: 09/10/2024
 \note     versión : 0.2
______________________________________________________________________________*/
#include "debug.hpp"
#include <vector>
#include <sstream>

using namespace gnc;

namespace {
constexpr auto kp_min = 0.1;
constexpr auto kq_min = 0.1;
constexpr auto kp_max = 100;
constexpr auto kq_max = 100;
constexpr auto kr_min = 0.0;
constexpr auto kl_min = 0.001;
constexpr auto kr_max = 100;
constexpr auto kl_max = 100;
constexpr auto alfa = 1.7;
constexpr auto beta = 1;

typedef std::runtime_error check_failure;
}

//------------------------------------------------------------------------------
void fcc_loader_t::fill_params(const double* p, unsigned len)
{
	m_stage = *p; ++p;
	if(m_stage < 0 || m_stage > 2)
		throw check_failure("Invalid stage index");
	--m_stage;

	fcc.plan().R_orbit                = *p; ++p;
	fcc.plan().times.sampling         = *p; ++p;
	fcc.plan().times.s1_burn          = *p; ++p;
	fcc.plan().times.s2_burn          = *p; ++p;
	fcc.plan().times.meco_advance     = *p; ++p;
	fcc.plan().times.s1_coast_time    = *p; ++p;
	fcc.plan().times.separation_time  = *p; ++p;
	fcc.plan().times.s2_fire_delay    = *p; ++p;

	fcc.plan().params.hgt_relief = *p; ++p;
	fcc.plan().params.hgt_void   = *p; ++p;
	fcc.plan().params.hgt_sep    = *p; ++p;
	fcc.plan().params.hgt_strn   = *p; ++p;

	fcc.plan().params.exit_speed = *p; ++p;
	fcc.plan().params.pre_cycle  = *p; ++p;
	fcc.plan().params.tgo_min    = *p; ++p;
	fcc.plan().params.v_eps      = *p; ++p;
	fcc.plan().params.f_min      = *p; ++p;

	std::stringstream msg;
	msg
	<<   "R_orbit         : " << fcc.plan().R_orbit
	<< "\nsampling        : " << fcc.plan().times.sampling
	<< "\ns1_burn         : " << fcc.plan().times.s1_burn
	<< "\ns2_burn         : " << fcc.plan().times.s2_burn
	<< "\nmeco_advance    : " << fcc.plan().times.meco_advance
	<< "\ns1_coast_time   : " << fcc.plan().times.s1_coast_time
	<< "\nseparation_time : " << fcc.plan().times.separation_time
	<< "\ns2_fire_delay   : " << fcc.plan().times.s1_coast_time
	<< "\n.hgt_relief     : " << fcc.plan().params.hgt_relief
	<< "\n.hgt_void       : " << fcc.plan().params.hgt_void
	<< "\n.hgt_sep        : " << fcc.plan().params.hgt_sep
	<< "\n.hgt_strn       : " << fcc.plan().params.hgt_strn
	<< "\n.exit_speed     : " << fcc.plan().params.exit_speed
	<< "\n.pre_cycle      : " << fcc.plan().params.pre_cycle
	<< "\n.tgo_min        : " << fcc.plan().params.tgo_min
	<< "\n.v_eps          : " << fcc.plan().params.v_eps
	<< "\n.f_min          : " << fcc.plan().params.f_min;
	trace(msg.str().c_str());
}

//------------------------------------------------------------------------------
namespace {

struct pd_gains_t
{
	double f;
	double g;
};
struct gains_t
{
	pd_gains_t roll;
	pd_gains_t ptch;
};
struct gain_schedule_t
{
	struct { std::vector<double> f, g; } roll;
	struct { std::vector<double> f, g; } ptch;
	void resize(size_t s)
	{
		roll.f.resize(s);
		roll.g.resize(s);
		ptch.f.resize(s);
		ptch.g.resize(s);
	}
	void reserve(size_t s)
	{
		roll.f.reserve(s);
		roll.g.reserve(s);
		ptch.f.reserve(s);
		ptch.g.reserve(s);
	}
	size_t size() const
	{
		return roll.f.size();
	}
	void push_back(const gains_t& g)
	{
		roll.f.push_back(g.roll.f);
		roll.g.push_back(g.roll.g);
		ptch.f.push_back(g.ptch.f);
		ptch.g.push_back(g.ptch.g);
	}
};

}
void fcc_loader_t::fill_gains(const double* p, unsigned n)
{
	ctrl::st_params_t prm = { 0, 0, 0, 0.5 };
	gain_schedule_t gains;
	gains.resize(0);
	gains.reserve(n);
	for(const double *q = p+n, *r = q+n, *l = r+n; n; --n, ++p, ++q, ++r, ++l)
	{
		gains_t g;
		g.ptch.f = *p;
		g.ptch.g = *q;
		g.roll.f = *r;
		g.roll.g = *l;


		if(g.ptch.f < kp_min) throw check_failure("invalid pitch gain"    );
		if(g.ptch.g < kq_min) throw check_failure("invalid rate gain"     );
		if(g.ptch.f > kp_max) throw check_failure("invalid pitch gain"    );
		if(g.ptch.g > kq_max) throw check_failure("invalid rate gain"     );
		if(m_stage == 0)
		{
		if(g.roll.f < kr_min) throw check_failure("invalid roll gain"     );
		if(g.roll.g < kl_min) throw check_failure("invalid roll rate gain");
		if(g.roll.f > kr_max) throw check_failure("invalid roll gain"     );
		if(g.roll.g > kl_max) throw check_failure("invalid roll rate gain");
		}

		gains.push_back(g);
	}
	second_t ts = fcc.plan().times.sampling;
	if(m_stage == 0)
	{
		fcc.ctrl_s1().tvc().setup(ts, prm, gains.ptch.g);
		fcc.ctrl_s1().fin().setup(ts, prm, gains.roll.g);
		fcc.ctrl_s1().py_gains.fill(gains.ptch.f.begin(), gains.ptch.f.size(), ts);
		fcc.ctrl_s1(). r_gains.fill(gains.roll.f.begin(), gains.roll.f.size(), ts);
	}
	else
	{
		fcc.ctrl_s2().tvc().setup(ts, prm, gains.ptch.g);
		fcc.ctrl_s2().rcs().setup(ts, gains.roll.g);
		fcc.ctrl_s2().py_gains.fill(gains.ptch.f.begin(), gains.ptch.f.size(), ts);
		fcc.ctrl_s2(). r_gains.fill(gains.roll.f.begin(), gains.roll.f.size(), ts);
	}
}

void fcc_loader_t::fill_wire(const double* p, unsigned n)
{
	std::vector<gnc::att_t> wire;
	wire.resize(0);
	wire.reserve(n);
    const double* h = p+n;
	for(unsigned k=0; k<n; ++k, ++p, ++h)
	{
		if(*p < 0.4 || *p > radian::_90deg + 1e-3)
        {
            char msg[128];
            sprintf(msg, "Invalid pitch reference [%s], index:%d", *p < 0.4 ? "<0.4" : ">90⁰", k);
			throw check_failure(msg);
        }
		radian pch(*p);
		radian hdg(*h);
		gnc::att_t qr(0, pch, hdg);
		wire.push_back(qr);
	}
	fcc.plan().wire.fill(&wire.front(), wire.size(), fcc.plan().times.sampling);
}

//==============================================================================
fcc_dbg_t::fcc_dbg_t()
: m_state       (fcc_t::st_init)
, m_ini_st      (fcc_t::st_init)
, m_t_launch    (0)
, m_t_separation(0)
{}
void fcc_dbg_t::on_time_step()
{
	if(m_ini_st == fcc_t::st_init && ins.elapsed >= m_t_launch)
    {
        fcc.arm(ins);
		trace("launch command");
		m_ini_st = -1;
    }
    else
	if(m_ini_st > fcc_t::st_init)
	{
		fcc.reset(ins, fcc_t::e_states(m_ini_st), m_t_launch, m_t_separation);
		trace("reset");
		m_ini_st = -1;
	}
	else
		fcc.on_time(ins);
	if(m_state != fcc.state_trace())
	{
		// reportar cambios de estado
		m_state = fcc.state_trace();
		on_state();
	}
}
void fcc_dbg_t::on_state() //second_t T, int st, const sim_info_t& p, logger_t& log)
{
    std::stringstream log;
	//log << "\n\n============================================================\n";
	//if(T < second_t(0))
		log << ins.elapsed << "s: STATE ";
	//else
	//	log << "T+" << ins.elapsed - T << "s: STATE ";
	switch(m_state)
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
		switch(m_state - fcc_t::st_last)
		{
		case fcc_t::guide_t::st_init       : log << "init       "; break;
		case fcc_t::guide_t::st_pre_thrust : log << "pre_thrust "; break;
		case fcc_t::guide_t::st_pre_cycling: log << "pre_cycling"; break;
		case fcc_t::guide_t::st_ltg        : log << "LTG        "; break;
		case fcc_t::guide_t::st_terminal   : log << "terminal   "; break;
		case fcc_t::guide_t::st_orbit      : log << "orbit      "; break;
		}
	}
	/*
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
        */
}
void fcc_dbg_t::initial_state(int st, second_t t_launch, second_t t_sep)
{
	if(st >= fcc_t::st_init && st <= fcc_t::st_orbit)
	{
		m_ini_st = st;
	    m_t_launch = t_launch;
	    m_t_separation = t_sep;
	}
}
void fcc_dbg_t::write_tlmy(double* p)
{
    fcc.update_tlmy();
    const fcc_t::tlmy_t& tmy = fcc.tlmy();

    *(  p) = tmy.state;
    *(++p) = tmy.q_ref.r();
    *(++p) = tmy.q_ref.x();
    *(++p) = tmy.q_ref.y();
    *(++p) = tmy.q_ref.z();
    *(++p) = tmy.e_dir.x();
    *(++p) = tmy.e_dir.y();
    *(++p) = tmy.e_dir.z();
    *(++p) = tmy.ctl.w_ref.x();
    *(++p) = tmy.ctl.w_ref.y();
    *(++p) = tmy.ctl.w_ref.z();
    *(++p) = tmy.gdn.state;
    *(++p) = tmy.gdn.ttgo;
    *(++p) = tmy.gdn.rtgo;
    *(++p) = tmy.gdn.rbis;
    *(++p) = tmy.gdn.pref.x();
    *(++p) = tmy.gdn.pref.y();
    *(++p) = tmy.gdn.pref.z();
}



