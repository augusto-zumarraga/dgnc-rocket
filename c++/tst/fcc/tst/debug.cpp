/*============================================================================*/ 
/*                                                                            */
/*============================================================================*/ 

/*!                                                                             
 \file     debug.cpp
 \brief                                                                         
 \author   Augusto Zumarraga                                                    
 \date     creación: 07/08/2024
 \date     revisión: 11/08/2024
 \note     versión : 0.1
______________________________________________________________________________*/
#include "debug.hpp"
#include <vector>

using namespace gnc;

typedef std::runtime_error check_failure;

fcc_dbg_t::fcc_dbg_t()
: m_stage(0)
, m_ini_st(fcc_t::st_init)
, m_sc_sample(0)
{}

//------------------------------------------------------------------------------
void fcc_dbg_t::fill_params(const double* p, unsigned len)
{
	m_stage = *p; ++p;
	if(m_stage < 0 || m_stage > 2)
		throw check_failure("Invalid stage index");
	--m_stage;

	float  hgt = *p; ++p;
	degree inc(*p); ++p;
	degree ran(*p); ++p;

	if(inc < 0 || inc > 90)
		throw check_failure("Invalid orbit inclination  (0:90)");
	if(ran < 0 || ran > 360)
		throw check_failure("Invalid orbit ascending node (0:360)");
	if(hgt < 100e3 || hgt > 300e3)
		throw check_failure("Invalid orbit height");
	fcc.plan().orbit  = flight_t::orbit_t(inc, ran, hgt + wgs84::a);

	flight_t::times_t  times;

	m_sc_sample   = *p; ++p;
	times.s1_burn = *p; ++p;
	times.s2_burn = *p; ++p;
	times.separation_delay = *p; ++p;
	times.burn_delay       = *p; ++p;
	if(times.separation_delay < second_t(0) || times.separation_delay > second_t(100))
		throw check_failure("Invalid separation_delay");
	if(times.burn_delay < second_t(0) || times.burn_delay > second_t(10))
		throw check_failure("Invalid burn_delay");
	fcc.plan().times  = times;

	flight_t::params_t params;
	params.hgt_relief = *p; ++p;
	params.hgt_strn   = *p; ++p;
	params.hgt_void   = *p; ++p;
	if(params.hgt_relief < 10 || params.hgt_relief > 300e3)
		throw check_failure("Invalid relief height");
	if(params.hgt_strn < 10e3 || params.hgt_strn   > 300e3)
		throw check_failure("Invalid steering height");
	if(params.hgt_void < 10e3 || params.hgt_void   > 100e3)
		throw check_failure("Invalid vaccum height");
	fcc.plan().params = params;

	flight_t::lts_t lts;
	lts.a = *p; ++p;
	lts.b = *p; ++p;
	if(lts.a < 0.0001 || lts.a > 0.1)
		throw check_failure("Invalid LTS a parameter");
	if(lts.b < -0.1 || lts.b > 0.1)
		throw check_failure("Invalid LTS b parameter");

	lts.t_burn = times.s1_burn
			   + times.separation_delay
			   + times.burn_delay
			   + times.s2_burn;
	fcc.plan().lts = lts;
}

//------------------------------------------------------------------------------
void fcc_dbg_t::fill_gains(const double* p, unsigned n)
{
	std::vector<flight_t::gains_t> gains;
	gains.resize(0);
	gains.reserve(n);
	for(const double *q = p+n, *r = q+n, *l = r+n; n; --n, ++p, ++q, ++r, ++l)
	{
		flight_t::gains_t g(*p, *q, *r, *l);
		if(g.kp < 0.1 ) throw check_failure("invalid pitch gain"    );
		if(g.kq < 0.1 ) throw check_failure("invalid rate gain"     );
		if(g.kp > 50  ) throw check_failure("invalid pitch gain"    );
		if(g.kq > 50  ) throw check_failure("invalid rate gain"     );
		if(m_stage == 0)
		{
		if(g.kr < 0.0 ) throw check_failure("invalid roll gain"     );
		if(g.kl < 0.01) throw check_failure("invalid roll rate gain");
		if(g.kr > 50  ) throw check_failure("invalid roll gain"     );
		if(g.kl > 50  ) throw check_failure("invalid roll rate gain");
		}
		gains.push_back(g);
	}
	if(m_stage == 1)
		fcc.plan().gains_s2.fill(&gains.front(), gains.size(), m_sc_sample);
	else
		fcc.plan().gains_s1.fill(&gains.front(), gains.size(), m_sc_sample);
}

//------------------------------------------------------------------------------
void fcc_dbg_t::fill_wire(const double* p, unsigned n)
{
	std::vector<gnc::att_t> wire;
	wire.resize(0);
	wire.reserve(n);
	radian hdg(radian::_90deg);
	for( ; n; --n, ++p)
	{
		if(*p < 0.5 || *p > radian::_90deg)
			throw check_failure("Invalid pitch reference ");
		gnc::att_t qr(0, *p, hdg);
		wire.push_back(qr);
	}
	fcc.plan().wire.fill(&wire.front(), wire.size(), m_sc_sample);
}




