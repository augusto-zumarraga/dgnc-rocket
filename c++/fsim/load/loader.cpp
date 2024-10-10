/*============================================================================*/
/*                                                                            */
/*============================================================================*/

////////////////////////////////////////////////////////////////////////////////
/// \brief
/// \author   Augusto Zumarraga
/// \date     creación: 27/06/2024
/// \date     revisión: 30/08/2024
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
#include "loader.hpp"
#include <dgnc/store/csv_file.hpp>
#include <dgnc/store/ini_file.hpp>
#include <dgnc/numeric/interp.hpp>
#include <fstream>
#include <sstream>
#include <cstdlib>

using namespace dgnc;
using namespace gnc;

using geom::radian;
using geom::degree;
using geom::dcm;
using geom::vector;

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

struct ctx_t
{
	static const char* sz;
	ctx_t(const char* p = 0)
	{
		sz = p;
	}
	~ctx_t()
	{
		sz = 0;
	}
	void operator=(const char* p)
	{
		sz = p;
	}
};
const char* ctx_t::sz = 0;

std::string check_msg( const char* sz_msg
		             , unsigned    n_line
					 , const char* sz_file)
{
	std::stringstream ss;
	if(ctx_t::sz)
		ss << ctx_t::sz << '\n';
	ss << "check failed:" << sz_msg << std::endl
	   << "file: " << sz_file
	   << ", line: " << n_line
	   << std::endl;
	return ss.str();
}

void read_gains(const data::ini_file& f, gnc::fcc_loader_t::pd_gains_t& fg, gnc::ctrl::st_params_t& cp, int S, bool roll)
{
	std::ostringstream ss;
	ss << "gains_S" << S << (roll ? ".roll_" : ".pitch_");
	std::string tag = ss.str();

	fg.f = atof(f.branch((tag + 'f').c_str()).get().c_str());
	double L = atof(f.branch((tag + 'L').c_str()).get().c_str());
	cp.l1  = alfa * sqrt(L);
	cp.l2  = beta * L;
	cp.rl  = atof(f.branch((tag + "rate_limit").c_str()).get().c_str());
	cp.k2  = atof(f.branch((tag + "k2").c_str()).get().c_str());
	if(cp.l1 > 0)
		fg.g = 1;
	else
		fg.g = atof(f.branch((tag + 'g').c_str()).get().c_str());
}

}
#define check_failure(msg) std::runtime_error(check_msg(#msg,__LINE__,__FILE__))

//------------------------------------------------------------------------------
fcc_loader_t::fcc_loader_t(std::string fpath)
{
	if(!fpath.empty() && fpath.back() != '/')
		fpath += '/';
	import_params(   (fpath + "params.ini"  ).c_str());
	import_wire  (   (fpath + "wire_v.csv"    ).c_str());
	import_gains (0, (fpath + "gains_S1.csv").c_str());
	import_gains (1, (fpath + "gains_S2.csv").c_str());
}

void fcc_loader_t::import_params(const char* fname)
{
	ctx_t ctx = fname;
	data::ini_file f(fname);

	float  hgt=atof(f.branch("orbit.height"        ).get().c_str()) * 1e3;
	degree inc(atof(f.branch("orbit.inclination"   ).get().c_str()));
	degree ran(atof(f.branch("orbit.ascending node").get().c_str()));

	if(inc < 0 || inc > 90)
		throw check_failure("Invalid orbit inclination  (0:90)");
	if(ran < 0 || ran > 360)
		throw check_failure("Invalid orbit ascending node (0:360)");
	if(hgt < 100e3 || hgt > 300e3)
		throw check_failure("Invalid orbit height");

	m_orbit = orbit_t(inc, ran, hgt + navs::wgs84::a);

	m_times.sampling = atof(f.branch("times.sample" ).get().c_str());
	m_times.s1_burn  = atoi(f.branch("times.S1 burn").get().c_str());
	m_times.s2_burn  = atof(f.branch("times.S2 burn").get().c_str());

	m_times.meco_advance     = atof(f.branch("times.pre-meco").get().c_str());
	m_times.s1_coast_time    = atoi(f.branch("times.S1 coast"  ).get().c_str());
	m_times.separation_time  = atoi(f.branch("times.separation").get().c_str());
	m_times.s2_fire_delay    = atoi(f.branch("times.S2 fire"  ).get().c_str());

	if(m_times.sampling < second_t(gnc::min_ts) || m_times.sampling > second_t(0.1))
		throw check_failure("Invalid sample time");
	if(m_times.s1_burn < second_t(100) || m_times.s1_burn > second_t(150))
		throw check_failure("Invalid stage 1 burn time");
	if(m_times.s2_burn < second_t(100) || m_times.s2_burn > second_t(500))
		throw check_failure("Invalid stage 2 burn time");

	if(m_times.s1_coast_time < second_t(0) || m_times.s1_coast_time > second_t(100))
		throw check_failure("Invalid s1_coast_time");
	if(m_times.separation_time < second_t(0) || m_times.separation_time > second_t(2))
		throw check_failure("Invalid separation_time");
	if(m_times.s2_fire_delay < second_t(0) || m_times.s2_fire_delay > second_t(10))
		throw check_failure("Invalid s2_fire_delay");

	m_params.hgt_relief = atof(f.branch("heights.relief"    ).get().c_str()) * 1e3;
	m_params.hgt_strn   = atof(f.branch("heights.steer"     ).get().c_str()) * 1e3;
	m_params.hgt_void   = atof(f.branch("heights.vaccum"    ).get().c_str()) * 1e3;
	m_params.hgt_sep    = atof(f.branch("heights.separation").get().c_str()) * 1e3;
	if(m_params.hgt_sep == 0)
		m_params.hgt_sep = m_params.hgt_strn;
	if(m_params.hgt_relief < 10 || m_params.hgt_relief > 300e3)
		throw check_failure("Invalid relief height");
	if(m_params.hgt_strn < 10e3 || m_params.hgt_strn   > 300e3)
		throw check_failure("Invalid steering height");
	if(m_params.hgt_void < 10e3 || m_params.hgt_void   > 150e3)
		throw check_failure("Invalid vaccum height");

	navs::ecef::lla_t launch_nav(0,0,15);
	launch_nav.lat = degree(atof(f.branch("space port.latitud" ).get().c_str()));
	launch_nav.lng = degree(atof(f.branch("space port.longitud").get().c_str()));
	radian hdg  = degree(atof(f.branch("space port.heading" ).get().c_str()));
	if(launch_nav.lat < degree(-50) || launch_nav.lat > degree(50) )
		throw check_failure("Invalid launch lattitude");
	if(launch_nav.lng < degree(-180) || launch_nav.lng > degree(180) )
		throw check_failure("Invalid launch longitude");

	m_params.exit_speed = atof(f.branch("guidance.v_exit"   ).get().c_str());
	m_params.pre_cycle  = atof(f.branch("guidance.pre_cycle").get().c_str());
	m_params.tgo_min    = atof(f.branch("guidance.tgo_min"  ).get().c_str());
	m_params.v_eps      = atof(f.branch("guidance.v_epsilon").get().c_str());
	m_params.f_min      = atof(f.branch("guidance.min_sfc"  ).get().c_str());
	if(m_params.v_eps == 0)
		m_params.v_eps = 0.1;
	if(m_params.f_min == 0)
		m_params.f_min = 0.4;
//	if(m_params.exit_speed < 2500 || m_params.exit_speed > 3500)
//		throw check_failure("Invalid exit speed");
//	if(m_params.pre_cycle < 0.1 || m_params.pre_cycle > 10)
//		throw check_failure("Invalid pre_cycle time");
//	if(m_params.tgo_min < 1 || m_params.tgo_min > 20)
//		throw check_failure("Invalid TGO minimum time");

	m_launch_state.elapsed = 0;
	m_launch_state.pos = launch_nav;
	m_launch_state.vel = vector(0,0,0);
	m_launch_state.att = ned_to_ecef(m_launch_state.pos, ned::att_t(0,radian::_90deg,hdg));

	// Control
	read_gains(f, m_f_gains[0].roll, m_s1_roll, 1, true );
	read_gains(f, m_f_gains[0].ptch, m_s1_ptch, 1, false);
	read_gains(f, m_f_gains[1].roll, m_s2_roll, 2, true );
	read_gains(f, m_f_gains[1].ptch, m_s2_ptch, 2, false);
}
/*
void fcc_loader_t::import_wire(const char* fname)
{
	ctx_t ctx = fname;
	csv::head_table_t f = csv::read(fname, 1, ' ');
	const csv::head_t & hdr = std::get<0>(f);
	const csv::table_t& tbl = std::get<1>(f);
	second_t ts(atof(hdr.front().c_str()));

	typedef std::vector<double> component_t;

	unsigned N = tbl.at(0).size();
	component_t r, x, y, z;
	r.reserve(N);
	x.reserve(N);
	y.reserve(N);
	z.reserve(N);
	switch(tbl.size())
	{
	case 4: // full quaternions components
		{
			const csv::column_t& _r = tbl.at(0);
			const csv::column_t& _x = tbl.at(1);
			const csv::column_t& _y = tbl.at(2);
			const csv::column_t& _z = tbl.at(3);

			using navs::ecef::att_t;
			for( csv::column_t::const_iterator ir = _r.begin(), er = _r.end()
			   , ix = _x.begin(), iy = _y.begin(), iz = _z.begin()
			   ; ir < er; ++ir, ++ix, ++iy, ++iz)
			{
				gnc::att_t qr(*ir, *ix, *iy, *iz);
				if(qr.norm() > 1.01)
					throw check_failure("Invalid reference quaternion");
				qr.normalize();
				r.push_back(qr.r());
				x.push_back(qr.x());
				y.push_back(qr.y());
				z.push_back(qr.z());
			}
		}
		break;
	case 2: // pitch and heading
		{
			const csv::column_t& p = tbl.at(0);
			const csv::column_t& h = tbl.at(1);
			m_wire.reserve(p.size());
			for(csv::column_t::const_iterator ip = p.begin(), ih = h.begin(), e = p.end(); ip < e; ++ip, ++ih)
			{
				if(*ip < 0.4)
					throw check_failure("Invalid pitch reference (< 0.4 rad)");
				if(*ip > radian::_90deg + 0.005)
					throw check_failure("Invalid pitch reference (> 90⁰)");
				gnc::att_t qr(0, *ip > radian::_90deg ? radian::_90deg : radian(*ip), radian(*ih));
				r.push_back(qr.r());
				x.push_back(qr.x());
				y.push_back(qr.y());
				z.push_back(qr.z());
			}
		}
		break;
	case 1: // pitch (heading East)
		{
			const csv::column_t& p = tbl.at(0);
			radian hdg = degree(90);
			m_wire.reserve(p.size());
			for(csv::column_t::const_iterator i = p.begin(), e = p.end(); i < e; ++i)
			{
				if(*i < 0.4)
					throw check_failure("Invalid pitch reference (< 0.4 rad)");
				if(*i > radian::_90deg + 0.005)
					throw check_failure("Invalid pitch reference (> 90⁰)");
				gnc::att_t qr(0, *i > radian::_90deg ? radian::_90deg : radian(*i), hdg);
				r.push_back(qr.r());
				x.push_back(qr.x());
				y.push_back(qr.y());
				z.push_back(qr.z());
			}
		}
		break;
	default:
		throw check_failure("invalid wire file");
	}
	N = ceil(N*ts/m_times.sampling);
	m_wire.resize(0);
	m_wire.reserve(N);
	if(ts != m_times.sampling) // Re-sample
	{
		// data times
		std::vector<double> tm(r.size());
		double t = 0;
		for(std::vector<double>::iterator i = tm.begin(), e = tm.end(); i != e; ++i, t += ts)
			*i = t;

		typedef numeric::interp::t_splines_1<4> data_t;
		data_t data;
		std::vector<component_t> src = { r, x, y, z};
		data.setup(tm, src, 0);

		t = 0;
		for(unsigned k=0; k < N; ++k, t += m_times.sampling)
		{
			data_t::result_t x = data(t);
			m_wire.push_back(gnc::att_t(x[0], x[1], x[2], x[3]));
		}
	}
	else
	{
		for(component_t::const_iterator ir = r.begin(), er = r.end()
		   , ix = x.begin(), iy = y.begin(), iz = z.begin()
		   ; ir < er; ++ir, ++ix, ++iy, ++iz)
			m_wire.push_back(gnc::att_t(*ir, *ix, *iy, *iz));
	}
	m_launch_state.att = ned_to_ecef(m_launch_state.pos, m_wire.front());
}
*/
void fcc_loader_t::import_wire(const char* fname)
{
	ctx_t ctx = fname;
	csv::table_t tbl = csv::read(fname, ' ');
	assert(tbl.size() >= 5);

	unsigned N = tbl.at(0).size();

	const csv::column_t& U = tbl.at(1);
	const csv::column_t& u = tbl.at(2);
	const csv::column_t& v = tbl.at(3);
	const csv::column_t& w = tbl.at(4);

	m_wire.range.resize(0);
	m_wire.range.resize(0);
	m_wire.data.reserve(N);
	m_wire.data.reserve(N);

	for( csv::column_t::const_iterator ir = U.begin(), er = U.end()
	   , iu = u.begin(), iv = v.begin(), iw = w.begin()
	   ; ir < er; ++ir, ++iu, ++iv, ++iw)
	{
		m_wire.range.push_back(*ir);
		m_wire.data .push_back(vector(*iu, *iv, *iw));
	}
}

void fcc_loader_t::import_gains(unsigned stage, const char* fname)
{
	ctx_t ctx = fname;
	csv::head_table_t f = csv::read(fname, 1, ' ');
	const csv::head_t & hdr = std::get<0>(f);
	const csv::table_t& tbl = std::get<1>(f);
	if(tbl.size() != 4)
		throw check_failure("invalid gains file");
	const csv::column_t& _p = tbl[0];
	const csv::column_t& _q = tbl[1];
	const csv::column_t& _r = tbl[2];
	const csv::column_t& _l = tbl[3];
	second_t ts(atof(hdr.front().c_str()));

	m_gains[stage].resize(0);
	if(ts != m_times.sampling) // resample
	{
		// data times
		std::vector<double> tm(_p.size());
		double t = 0;
		for(std::vector<double>::iterator i = tm.begin(), e = tm.end(); i != e; ++i, t += ts)
			*i = t;

		typedef numeric::interp::t_splines_1<4> data_t;
		data_t data;
		data.setup(tm, tbl, 0);

		unsigned N = ceil(_p.size() * ts / m_times.sampling);
		m_gains[stage].reserve(N);
		t = 0;
		for(unsigned k=0; k < N; ++k, t += m_times.sampling)
		{
			data_t::result_t x = data(t);

			if(x[0] < kp_min) throw check_failure("invalid pitch gain"    );
			if(x[1] < kq_min) throw check_failure("invalid rate gain"     );
			if(x[0] > kp_max) throw check_failure("invalid pitch gain"    );
			if(x[1] > kq_max) throw check_failure("invalid rate gain"     );
			if(stage == 0)
			{
			if(x[2] < kr_min) throw check_failure("invalid roll gain"     );
			if(x[3] < kl_min) throw check_failure("invalid roll rate gain");
			if(x[2] > kr_max) throw check_failure("invalid roll gain"     );
			if(x[3] > kl_max) throw check_failure("invalid roll rate gain");
			}
			gains_t g;
			g.ptch.f = x[0] * m_f_gains[stage].ptch.f;
			g.ptch.g = x[1] * m_f_gains[stage].ptch.g;
			g.roll.f = x[2] * m_f_gains[stage].roll.f;
			g.roll.g = x[3] * m_f_gains[stage].roll.g;
			m_gains[stage].push_back(g);
		}
	}
	else
	{
		m_gains[stage].reserve(_p.size());
		for( csv::column_t::const_iterator ip = _p.begin(), ep = _p.end(), iq = _q.begin(), ir = _r.begin(), il = _l.begin()
		   ; ip < ep; ++ip, ++iq, ++ir, ++il)
		{
			gains_t g;
			g.ptch.f = (*ip) * m_f_gains[stage].ptch.f;
			g.ptch.g = (*iq) * m_f_gains[stage].ptch.g;
			g.roll.f = (*ir) * m_f_gains[stage].roll.f;
			g.roll.g = (*il) * m_f_gains[stage].roll.g;

			if(g.ptch.f < kp_min) throw check_failure("invalid pitch gain"    );
			if(g.ptch.g < kq_min) throw check_failure("invalid rate gain"     );
			if(g.ptch.f > kp_max) throw check_failure("invalid pitch gain"    );
			if(g.ptch.g > kq_max) throw check_failure("invalid rate gain"     );
			if(stage == 0)
			{
			if(g.roll.f < kr_min) throw check_failure("invalid roll gain"     );
			if(g.roll.g < kl_min) throw check_failure("invalid roll rate gain");
			if(g.roll.f > kr_max) throw check_failure("invalid roll gain"     );
			if(g.roll.g > kl_max) throw check_failure("invalid roll rate gain");
			}
			m_gains[stage].push_back(g);
		}
	}
}

void fcc_loader_t::setup(gnc::fcc_t& f) const
{
	f.plan().R_orbit = m_orbit.radius();
	f.plan().times   = m_times;
	f.plan().params  = m_params;
	//f.plan().wire    .fill(m_wire.begin(), m_wire.size(), m_times.sampling);
	f.plan().wire.fill(m_wire.range.begin(), m_wire.data.begin(), m_wire.range.size());

	f.ctrl_s1().tvc().setup(m_times.sampling, m_s1_ptch, m_gains[0].ptch.g);
	f.ctrl_s1().fin().setup(m_times.sampling, m_s1_roll, m_gains[0].roll.g);
	f.ctrl_s1().py_gains.fill(m_gains[0].ptch.f.begin(), m_gains[0].ptch.f.size(), m_times.sampling);
	f.ctrl_s1(). r_gains.fill(m_gains[0].roll.f.begin(), m_gains[0].roll.f.size(), m_times.sampling);

	f.ctrl_s2().tvc().setup(m_times.sampling, m_s2_ptch, m_gains[1].ptch.g);
	f.ctrl_s2().rcs().setup(m_times.sampling, m_gains[1].roll.g);
	f.ctrl_s2().py_gains.fill(m_gains[1].ptch.f.begin(), m_gains[1].ptch.f.size(), m_times.sampling);
	f.ctrl_s2(). r_gains.fill(m_gains[1].roll.f.begin(), m_gains[1].roll.f.size(), m_times.sampling);
}

std::ostream& operator<<(std::ostream& s, const gnc::ctrl::st_params_t& p)
{
	s <<   "k2: " << p.k2
	  << ", l1: " << p.l1
	  << ", l2: " << p.l2
	  << ", rl: " << p.rl;
	return s;
}
std::ostream& operator<<(std::ostream& s, const gnc::fcc_loader_t::pd_gains_t& g)
{
	s << "f: " << g.f << ", g: " << g.g;
	return s;
}
void fcc_loader_t::print(std::ostream& s) const
{
	using ::operator<<;
	navs::ecef::nav_t nav = m_launch_state.pos;
	s << "\norbit: "
	  << "height "               << m_orbit.height()*1e-3
	  << "km, speed "            << m_orbit.speed()
	  << "m/s, inclination "     << degree(m_orbit.inclination())
	  << ", ascending node "     << degree(m_orbit.RAAN())
	  << "\nspace port : "       << nav
	  << "\nMECO at "                       << m_times.s1_burn
	  << "s\ncoast time: "                  << m_times.s1_coast_time
	  << "s\nseparation time: "             << m_times.separation_time
	  << "s\nsecond stage ignition delay: " << m_times.s2_fire_delay
	  << "s\nrelief height: "    << m_params.hgt_relief * 1e-3
	  << "km\nsteering height: " << m_params.hgt_strn * 1e-3
	  << "km\nvaccum height: "   << m_params.hgt_void * 1e-3
	  << "\nVe: "                << m_params.exit_speed
	  << " m/s, t_burn: "        << m_times.s2_burn
	  << " s, t_pre-cycle: "     << m_params.pre_cycle
	  << " s, ts: "              << m_times.sampling << " s"
	  << "\n\nGaing stage 1"     << "\n pitch | " << m_f_gains[0].ptch << ", " << m_s1_ptch
	                             << "\n roll  | " << m_f_gains[0].roll << ", " << m_s1_roll
	  <<   "\nGaing stage 2"     << "\n pitch | " << m_f_gains[1].ptch << ", " << m_s1_ptch
	                             << "\n roll  | " << m_f_gains[1].roll << ", " << m_s1_roll
	  ;
}




