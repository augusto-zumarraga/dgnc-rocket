/*============================================================================*/
/*                                               Guiado, Navegación y Control */
/*============================================================================*/

////////////////////////////////////////////////////////////////////////////////
/// \brief    CONFIGURATION GUIDANCE FOR PICO LAUNCHERS (API-PGUID)
///           Algoritmos
/// \author   Alberto Fraguío
/// \author   Augusto Zumarraga (C++ coding)
/// \date     creación: 13/08/2024
/// \date     revisión: 26/08/2024
//______________________________________________________________________________

/*
    Copyright (c) 2024 Alberto Fraguío

	Pguid is free software: you can redistribute it and/or modify it under the
	terms of the GNU General Public License as published by the Free Software
	Foundation, either version 3 of the License, or (at your option) any later
	version.

	Pguid is distributed in the hope that it will be useful, but WITHOUT ANY
	WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
	A PARTICULAR PURPOSE. See the GNU General Public License for more details.

	You should have received a copy of the GNU General Public License along with
    Pguid. If not, see <https://www.gnu.org/licenses/>.
*/
#include "pguid.hpp"
using namespace gnc;
using namespace guid;

namespace {

//atic constexpr auto re = wgs84::Re;  ///< [m] Central body mean radius
static constexpr auto mu = wgs84::GM;  ///< [m^3/s^2] Central body gravitational constant

//	Resuelve la ecuación de Kepler
//
//	   M = E - e*sin(E)
//
//	mediante el Newton-Raphson.
//	Referencia: "Fundamentals of Spacecraft Attitude Determination and Control",
//              F. Landis Markley, John L. Crassidis
//
struct keppler_sol_t
{
	radian Ehat;
	bool   ok;

	keppler_sol_t(radian M, scalar e, scalar relTol = 1e-12, unsigned maxIter = 10)
	{
		// Estimación inicial
		radian A = e*sin(M)/(1-e*cos(M));
		Ehat = M + A - 1/2*A*A*A;
		unsigned k = 0;
		for( ; k<maxIter; ++k)
		{
			radian DeltaE   = (M - (Ehat - e*sin(Ehat)))/(1-e*cos(Ehat));
			radian relError = std::abs(DeltaE/Ehat);
			Ehat = Ehat + DeltaE;
			if(relError <= relTol)
				break;
		}
		ok = k < maxIter;
	}
};

}

void ltg_t::params_t::operator=(const guid::params_t& p)
{
	Tg         = p.ts;
	Ve         = p.Ve;
	VD         = p.VD;
	rD         = p.rD;
	tgomin     = p.tgomin;
	tburn      = p.tburn;
	cycle_time = p.tcycl;
	Fsmin      = p.Fsmin;
}
void ltg_t::fill(tlmy_t& s)
{
	s.ttgo  = m_state.ttgo;
	s.rtgo  = m_res  .rtgo;
	s.rbis  = m_state.rbias_I.norm();
}
inline ltg_t::tgo_t
ltg_t::time_to_go(scalar Fs) const
{
	tgo_t t;
	t.L   = m_state.vgo_I.norm();
	t.tau = params.Ve/Fs;
	t.tgo = t.tau*(1 - exp(-t.L/params.Ve));
	return t;
}

inline ltg_t::itg_t
ltg_t::integrals(const tgo_t& t) const
{
	scalar t1 = t.tgo;
	scalar t2 = t1*t1;
	scalar t3 = t2*t1;

	itg_t i;
	i.S = -(t.tau-t.tgo)*t.L + params.Ve*t.tgo;
	i.J = t.tgo*t.L - i.S;
	i.Q = t.tau*i.S - 0.5*params.Ve*t2;
	i.P = t.tau*i.Q - (params.Ve/6)*(t3);
	i.H = t.tau*i.J - (params.Ve/2)*(t2);
	return i;
}

inline vector
ltg_t::range_to_go(const tgo_t& t, const itg_t& i, const nav_t& nv) const
{
	vector rhatgrav_I;
	if(m_state.ttgo != second_t(0))
	{
		scalar f = t.tgo/m_state.ttgo;
		rhatgrav_I = m_state.rgrav_I*f*f;
	}
	else
		rhatgrav_I = m_state.rgrav_I;

	vector rhatgo_I = m_state.sDI_I - (nv.pos + nv.vel*t.tgo).as_any() - rhatgrav_I;
	vector  rgoxy_I = rhatgo_I - m_state.uZ_I * (m_state.uZ_I * rhatgo_I);
	scalar     rgoz = (i.S - (m_res.ulambda_I * rgoxy_I))/(m_res.ulambda_I * m_state.uZ_I);
	vector rhatth_I = rgoz*m_state.uZ_I + rgoxy_I;

	return rhatth_I + m_state.rbias_I;
}

inline ltg_t::kpl_t
ltg_t::keppler(const vector & sB1I_I, const vector & vB1I_I, double tgo) const
{
	kpl_t kp;
//  % Propagación cónica desde (sB1I_I, vB1I_I, t=0) hasta (sB2I_I, vB2I_I, t=tgo)
	scalar r1   = sB1I_I.norm();
	scalar V1   = vB1I_I.norm();
	vector h_I  = sB1I_I ^ vB1I_I;
	scalar a    = r1/(2-r1*V1*V1/mu);
	vector evec = (vB1I_I ^ h_I)/mu - sB1I_I/r1;
	scalar e    = evec.norm();
	assert(e < 1);
	scalar e2   = e*e;
	scalar a2   = a*a;
	scalar se   = sqrt(1-e2);

	vector    p1 = evec/e;
	direction p3 = h_I;
	vector    p2 = p3 ^ p1;

	dcm T_PI(p1, p2, p3);

	scalar n   = sqrt(mu/(a2*a));
	radian E0  = atan2((sB1I_I*vB1I_I)/(n*a2), 1-r1/a);
	radian M0  = E0 - e*sin(E0);
	radian Mgo = M0 + n*tgo;

	keppler_sol_t kp_sol(Mgo,e);
	scalar rgo = a*(1-e*cos(kp_sol.Ehat));
	vector go(a*(cos(kp_sol.Ehat) - e), a*se*sin(kp_sol.Ehat), 0);
	vector dt(-n*a2/rgo*sin(kp_sol.Ehat), n*a2/rgo*se*cos(kp_sol.Ehat), 0);

	kp.pos = T_PI * go;
	kp.vel = T_PI * dt;

	return kp;
}

inline ltg_t::prd_t
ltg_t::predictor(const tgo_t& t, const itg_t& i, const nav_t& nv) const
{
	prd_t pr;

	radian phi   = acos(m_res.out.dir*m_res.ulambda_I);
	radian phi_p = -phi/(i.J/t.L);

	direction lambdadot_Iunit = m_res.out.rate;

	scalar p2  = phi  *phi;
	scalar pp2 = phi_p*phi_p;
	 // Integral primera de propulsión
	vector vth_I = (t.L - 0.5*t.L*p2 - i.J*phi*phi_p - 0.5*i.H*pp2)*m_res.ulambda_I
			     - (t.L*phi + i.J*phi_p)*lambdadot_Iunit;
	// Integral segunda de propulsión
	pr.rth = (i.S - 0.5*i.S*p2 - i.Q*phi*phi_p - 0.5*i.P*pp2)*m_res.ulambda_I
		   - (i.S*phi + i.Q*phi_p)*lambdadot_Iunit;

	vector sB1I_I = nv.pos.as_any() - (0.1*pr.rth + vth_I*t.tgo/30);
	vector vB1I_I = nv.vel.as_any() + (1.2/t.tgo*pr.rth - 0.1*vth_I);

	kpl_t kp = keppler(sB1I_I, vB1I_I, t.tgo);

	vector vgrav_I = kp.vel - vB1I_I;
	pr.rgv = kp.pos - sB1I_I - vB1I_I*t.tgo;
	pr.pos = (nv.pos + nv.vel*t.tgo).as_any() + pr.rgv + pr.rth;
	pr.vel = nv.vel.as_any() + vgrav_I + vth_I;

	return pr;
}
inline void ltg_t::corrector(const prd_t& pr, const nav_t& nv)
{
	  direction uY_I = nv.vxp;
	  direction uD_I = pr.pos;
	  m_state.uZ_I   = uD_I ^ uY_I;
	  m_state.sDI_I  = params.rD * uD_I;
	  vector vDI_I   = params.VD * m_state.uZ_I;
	  vector vmiss_I = pr.vel - vDI_I;
	  m_state.vgo_I -= vmiss_I;
}


//==============================================================================
void ltg_t::init(const nav_t& nv)
{
	direction uY_I(nv.vxp);
	m_state.sDI_I = direction(nv.pos) * params.rD;
	vector
	vDI_I = params.VD*direction(-uY_I ^ m_state.sDI_I);
	vDI_I-= nv.vel.as_any();

	m_state.uZ_I = direction(m_state.sDI_I ^ uY_I);

	scalar N2 = nv.pos.sum2();
	scalar Np = sqrt(N2);
	scalar N3 = N2*Np;

	m_state.rbias_I = 0;
	m_state.rgrav_I = -mu/(2*N3)*nv.pos.as_any();
	m_state.ttgo    = 0;
	m_state.vgo_I   = vDI_I;

	m_res.out.dir   = direction(nv.vel);
	m_res.t0        = nv.elapsed;
	m_res.ulambda_I = m_res.out.dir;
	m_res.out.rate  = 0;
}
bool ltg_t::update(const nav_t& nv)
{
	if(go(nv))
	{
		// Algoritmo de actualizacion @ sampling_rate = 1/Tg
		m_state.vgo_I  -= nv.sfc.as_any() * params.Tg;
		tgo_t t = time_to_go(nv.Fs);
		itg_t i = integrals(t);
		m_res.ulambda_I = m_state.vgo_I;
		vector rgo_I = range_to_go(t, i, nv);
		scalar den = i.Q - i.S*i.J/t.L;
		if(den != 0)
			m_res.out.rate = (rgo_I - i.S*m_res.ulambda_I) / den;
		else
			m_res.out.rate = 0;

		m_res.out.dir = m_res.ulambda_I - i.J/t.L*m_res.out.rate;
		m_res.t0  = nv.elapsed + second_t(i.J/t.L);

		prd_t pr = predictor(t, i, nv);
		m_state.rgrav_I = pr.rgv;
		m_state.rbias_I = rgo_I - pr.rth;

		corrector(pr, nv);
		m_state.ttgo = t.tgo;
		m_res  .rtgo = rgo_I.norm();
	}
	return done();
}

