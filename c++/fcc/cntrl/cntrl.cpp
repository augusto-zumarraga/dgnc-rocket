/*============================================================================*/
/*                                               Guiado, Navegación y Control */
/*============================================================================*/

////////////////////////////////////////////////////////////////////////////////
/// \brief
/// \author   Augusto Zumarraga
/// \date     creación: 27/06/2024
/// \date     revisión: 27/09/2024
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
#include "cntrl.hpp"

using namespace gnc;
using namespace ctrl;

//==============================================================================
inline axis_t::axis_t()
: r_lim(0.5), r_rate(r_lim*0.5)
{}
void axis_t::reset(double x)
{
	e_rate.reset(x);
	r_rate.reset(x);
}
void axis_t::setup(const st_params_t& p)
{
	phi.k2 = p.k2;
	stc.setup(p.l1, p.l2);
	r_lim = p.rl;
	r_rate.lim = r_lim*0.5;
}
inline double axis_t::operator()( double& r, const double w
		                        , const double k, const double h)
{
	math::sat(r, r_lim);
	r_rate(r, h);
	double e = r - w;
	double u;
	if(stc)
	{
		// control ST
		e *= -1;
		if(phi)
			// control 3-ST
			e = phi(e, e_rate(e, h));
		out = stc(e, h, 1/k);
		u = k * out.u;
	}
	else
	{
		// control proporcional
		u = k * e;
		math::sat(r, 1);
	}
	return u;
}

//------------------------------------------------------------------------------
tvc_yz_t::tvc_yz_t()
: m_ts(0.01)
, m_dy(0)
, m_dz(0)
{}
void tvc_yz_t::reset()
{
	q_ctrl.reset(0);
	r_ctrl.reset(0);
}
void tvc_yz_t::cloop(const ins_data_t& ins, double q_ref, double r_ref)
{
	const double g = gains(ins.elapsed);
	m_dy = q_ctrl(q_ref, ins.wbi.y(), g, m_ts);
	m_dz = r_ctrl(r_ref, ins.wbi.z(), g, m_ts);
}

//------------------------------------------------------------------------------
roll_fin_t::roll_fin_t()
: m_ts(0.01)
, m_da(0)
{}
void roll_fin_t::reset()
{
	p_ctrl.reset(0);
}
void roll_fin_t::cloop(const ins_data_t& ins, double p_ref)
{
	const double g = gains(ins.elapsed);
	m_da = p_ctrl(p_ref, ins.wbi.x(), g, m_ts);
}

//------------------------------------------------------------------------------
roll_rcs_t::roll_rcs_t()
: m_rc(0)
{}
void roll_rcs_t::reset()
{

}
void roll_rcs_t::cloop(const ins_data_t& ins, double p_ref)
{
	const double g = gains(ins.elapsed);
	double e = ins.wbi.x() - p_ref;
	m_rc = g*e;
}
void roll_rcs_t::update(cmnds::cont_t& c)
{
	c.rc = std::abs(m_rc) > 1 ? sign(m_rc) : 0;
}

//==============================================================================
void ctrl_t::update(tlmy_t& s) const
{
	s.w_ref = m_w_ref;
}

//------------------------------------------------------------------------------
void atm_t::roll_loop(const ins_data_t& ins)
{
    m_fin.cloop(ins, 0);
    m_fin.update(m_cmnd);
}
void atm_t::pqr_loop(const ins_data_t& ins)
{
    m_fin.cloop(ins, m_w_ref.x());
    m_tvc.cloop(ins, m_w_ref.y(), m_w_ref.z());
    m_fin.update(m_cmnd);
    m_tvc.update(m_cmnd);
}
void atm_t::att_loop(const quaternion& qref, const quaternion& qatt, const ins_data_t& ins)
{
	quaternion qerr = qref.conj() * qatt;
    euler err(qerr.conj());

    double
	g = r_gains(ins.elapsed);
    m_w_ref.x() = g * err.roll;
	g = py_gains(ins.elapsed);
	m_w_ref.y() = g * err.pitch;
	m_w_ref.z() = g * err.yaw;

    m_fin.cloop(ins, m_w_ref.x());
    m_fin.update(m_cmnd);

    m_tvc.cloop(ins, m_w_ref.y(), m_w_ref.z());
    m_tvc.update(m_cmnd);
}
void atm_t::load_relief(const att_t& qref, const ins_data_t& ins)
{
	att_t qwr = ecef_to_ned(ins.pos, ins.att);
	// TODO ajustar referencia en función de la carga lateral
	att_loop(qref, qwr, ins);
}
void atm_t::start(second_t elps)
{
	m_tvc.gains.set_offset(elps);
	m_fin.gains.set_offset(elps);
	   py_gains.set_offset(elps);
	    r_gains.set_offset(elps);
	reset();
}
void atm_t::reset()
{
	m_w_ref = 0;
    m_fin.reset();
    m_tvc.reset();
    m_cmnd.reset();
}

//------------------------------------------------------------------------------
void exo_t::roll_loop(const ins_data_t& ins)
{
    m_rcs.cloop(ins, 0);
    m_rcs.update(m_cmnd);
}
void exo_t::pointing_loop(const vector& rx, const ins_data_t& ins)
{
	double ey =  atan2( rx.y(), rx.x()); math::sat(ey, 0.1);
	double ep = -atan2( rx.z(), rx.x()); math::sat(ep, 0.1);
	m_w_ref.x() = 0;
	double g = py_gains(ins.elapsed);
	m_w_ref.y() = g * ep;
	m_w_ref.z() = g * ey;

    m_rcs.cloop(ins, m_w_ref.x());
    m_rcs.update(m_cmnd);
	m_tvc.cloop(ins, m_w_ref.y(), m_w_ref.z());
    m_tvc.update(m_cmnd);
}
void exo_t::start(second_t elps)
{
	m_tvc.gains.set_offset(elps);
	m_rcs.gains.set_offset(elps);
	   py_gains.set_offset(elps);
	    r_gains.set_offset(elps);
	reset();
}
void exo_t::reset()
{
	m_w_ref = 0;
    m_tvc.reset();
    m_cmnd.reset();
}






