/*============================================================================*/
/*                                                                            */
/*============================================================================*/
#pragma once
////////////////////////////////////////////////////////////////////////////////
/// \brief
/// \author   Augusto Zumarraga
/// \date     creación: 10/06/2024
/// \date     revisión: 02/10/2024
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
#include "plot.hpp"
#include <dgnc/navs/eci.hpp>

namespace {

using namespace dgnc;
using namespace fsim;

//------------------------------------------------------------------------------
inline double get_xe(const rocket_mdl_t::vect_t& x) { return  x.pos.x() * 1e-3; }
inline double get_ye(const rocket_mdl_t::vect_t& x) { return  x.pos.y() * 1e-3; }
inline double get_ze(const rocket_mdl_t::vect_t& x) { return  x.pos.z() * 1e-3; }

//------------------------------------------------------------------------------
struct nav_data_t
{
	navs::ecef::lla_t lla;
    geom::polar       vel;

	void set_lla(const ecef::state_t& x)
	{
		lla = x.pos;
	}
	nav_data_t(const ecef::state_t& x)
	{
		set_lla(x);
		vel = navs::loc::map(lla, x.vel);
	}
	nav_data_t(){}
};
inline nav_data_t get_nav(const rocket_mdl_t::vect_t& x)
{
	return nav_data_t(x);
}
inline double get_rng(const nav_data_t& N, const ecef::lla_t& r)
{
	return N.lla.distance(r) * wgs84::Rn(N.lla.lat) * 0.001;
}
inline double get_hgt  (const rocket_mdl_t::vect_t& x){ return (x.pos.norm() - wgs84::Re) * 0.001; }

inline double get_alt  (const nav_data_t& N){ return N.lla.alt * 0.001; }
inline double get_lat  (const nav_data_t& N){ return degree(N.lla.lat); }
inline double get_lng  (const nav_data_t& N){ return degree(N.lla.lng); }
inline double get_vel  (const nav_data_t& N){ return N.vel.mod;         }
inline double get_beta (const nav_data_t& N){ return degree(N.vel.hdn); }
inline double get_gamma(const nav_data_t& N){ return degree(N.vel.elv); }

//------------------------------------------------------------------------------
struct dyn_t
{
	geom::euler          eul;
    actuators_t::state_t act;

	void set_eul(const ecef::state_t& x)
	{
		eul = ecef_to_ned(x.pos, x.att);
	}
	dyn_t(const rocket_mdl_t::vect_t& x)
	{
		set_eul(x);
		eul.normalize(degree(0.1));
		act = x.act;
	}
	dyn_t(){}
};

inline dyn_t get_dyn(const rocket_mdl_t::vect_t& x)
{
	return dyn_t(x);
}
// Deformaciones elásticas
inline double get_flx_y(const rocket_mdl_t::vect_t& x) { return x.flx.y_state.pos; }
inline double get_flx_z(const rocket_mdl_t::vect_t& x) { return x.flx.z_state.pos; }
// [⁰/s]
inline double get_flx_p(const rocket_mdl_t::vect_t& x) { return math::r2d(x.flx.y_state.vel); }
inline double get_flx_r(const rocket_mdl_t::vect_t& x) { return math::r2d(x.flx.z_state.vel); }
// [⁰]
inline double get_roll (const dyn_t& x){ return degree(x.eul.roll ); }
inline double get_pitch(const dyn_t& x){ return degree(x.eul.pitch); }
inline double get_yaw  (const dyn_t& x){ return degree(x.eul.yaw  ); }
// [normalizados]
inline double get_tvc_y(const dyn_t& x){ return x.act.dy ; }
inline double get_tvc_z(const dyn_t& x){ return x.act.dz ; }
inline double get_ail  (const dyn_t& x){ return x.act.ail; }

//------------------------------------------------------------------------------
inline double get_Q    (const sim_info_t& x){ return x.env.wnd.Q * (1e-4/wgs84::go); }
inline double get_M    (const sim_info_t& x){ return x.env.wnd.mch; }
inline double get_aoa  (const sim_info_t& x){ return degree(x.env.wnd.aoa); }
inline double get_slp  (const sim_info_t& x){ return degree(x.env.wnd.slp); }
inline double get_fx   (const sim_info_t& x){ return x.env.fb .x( ); }
inline double get_fy   (const sim_info_t& x){ return x.env.fb .y( ); }
inline double get_fz   (const sim_info_t& x){ return x.env.fb .z( ); }
inline double get_wx   (const sim_info_t& x){ return math::r2d(x.env.wbi.x()); }
inline double get_wy   (const sim_info_t& x){ return math::r2d(x.env.wbi.y()); }
inline double get_wz   (const sim_info_t& x){ return math::r2d(x.env.wbi.z()); }
inline double get_T    (const sim_info_t& x){ return x.env.T.norm() / wgs84::go; }
inline double get_mass (const sim_info_t& x){ return x.env.mass.m; }
inline double get_rcs  (const sim_info_t& x){ return x.env.rcs; }

using geom::scalar;

//------------------------------------------------------------------------------
struct gnc_t
{
	ned::att_t       att;
	ecef::att_t      qec;
	eci::state_t     nav;
	fcc_t::tlmy_t    tmy;
	angle_rate_t     wbi;

	geom::direction  dir;
	geom::quaternion qerr;
	geom::radian     eD;
	angle_rate_t     ew;

	gnc_t() // @suppress("Class members should be properly initialized")
	{}

	void operator=(const sim_info_t& x)
	{
		qec = x.ins.att;
		att = ecef_to_ned(x.ins.pos, qec);
		nav = x.ins;
		wbi = x.env.wbi;
	}
	void operator=(const fcc_t::tlmy_t& x)
	{
		tmy = x;
	}
	void compute()
	{
		 qerr = tmy.q_ref.conj() * att;

		 static const vector xb(1,0,0);
		 dir = nav.att << xb;
		 eD  = tmy.gdn.pref.null() ? 0 : geom::acos(tmy.gdn.pref * dir);
		 ew  = tmy.ctl.w_ref - wbi;
	}
};

inline const sim_info_t& get_sim(const sim_info_t& x)
{
	return x;
}
inline const fcc_t::tlmy_t& get_tmy(const fcc_t::tlmy_t& x)
{
	return x;
}

//------------------------------------------------------------------------------
// error en componentes del cuaternion
inline double get_qn(const gnc_t& x){ return x.tmy.q_ref.r(); }
inline double get_qx(const gnc_t& x){ return x.tmy.q_ref.x(); }
inline double get_qy(const gnc_t& x){ return x.tmy.q_ref.y(); }
inline double get_qz(const gnc_t& x){ return x.tmy.q_ref.z(); }
// error en componentes del cuaternion
inline double get_en(const gnc_t& x){ return x.qerr.r(); }
inline double get_ex(const gnc_t& x){ return x.qerr.x(); }
inline double get_ey(const gnc_t& x){ return x.qerr.y(); }
inline double get_ez(const gnc_t& x){ return x.qerr.z(); }
// error de actitud en ángulos de Euler
inline double get_e_roll (const gnc_t& x){ return degree(x.qerr.roll ()); }
inline double get_e_pitch(const gnc_t& x){ return degree(x.qerr.pitch()); }
inline double get_e_yaw  (const gnc_t& x){ return degree(x.qerr.yaw  ()); }
// error en velocidades angulares
inline double get_ep(const gnc_t& x){ return math::r2d(x.ew.x()); }
inline double get_eq(const gnc_t& x){ return math::r2d(x.ew.y()); }
inline double get_er(const gnc_t& x){ return math::r2d(x.ew.z()); }

//------------------------------------------------------------------------------
// error en la dirección
inline double get_eD(const gnc_t& x){ return degree(x.eD); }
// referencia para el apuntamiento ECI
inline double get_Rx(const gnc_t& x){ return x.tmy.gdn.pref.x(); }
inline double get_Ry(const gnc_t& x){ return x.tmy.gdn.pref.y(); }
inline double get_Rz(const gnc_t& x){ return x.tmy.gdn.pref.z(); }
// apuntamiento ECI
inline double get_Dx(const gnc_t& x){ return x.dir.x(); }
inline double get_Dy(const gnc_t& x){ return x.dir.y(); }
inline double get_Dz(const gnc_t& x){ return x.dir.z(); }
// referencia para el apuntamiento terna body
inline double get_Ex(const gnc_t& x){ return x.tmy.e_dir.x(); }
inline double get_Ey(const gnc_t& x){ return x.tmy.e_dir.y(); }
inline double get_Ez(const gnc_t& x){ return x.tmy.e_dir.z(); }

// telemetría LTG
inline double get_pst(const gnc_t& x){ return x.tmy.gdn.state; }
inline double get_tgo(const gnc_t& x){ return x.tmy.gdn.ttgo; }
inline double get_rgo(const gnc_t& x){ return x.tmy.gdn.rtgo; }
inline double get_rbs(const gnc_t& x){ return x.tmy.gdn.rbis; }

//------------------------------------------------------------------------------
typedef std::vector<double> dbl_vect;
typedef std::vector<dbl_vect::const_iterator> itr_vect;
}



