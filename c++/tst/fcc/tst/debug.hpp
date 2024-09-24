/*============================================================================*/ 
/*                                                                            */
/*============================================================================*/ 

/*!                                                                             
 \file     debug.hpp
 \brief                                                                         
 \author   Augusto Zumarraga                                                    
 \date     creación: 07/08/2024
 \date     revisión: 11/08/2024
 \note     versión : 0.1
______________________________________________________________________________*/
#pragma once

#include "../fcc.hpp"

namespace gnc {

//------------------------------------------------------------------------------
class fcc_dbg_t
{
public:

	ins_data_t ins;
	fcc_t      fcc;

	static constexpr auto cmd_len = 3 + 3; // [dy dz da ] [eng sep rel]
	static constexpr auto sns_len = (1 + 3 + 3 + 4) + (3 + 3) + 3; // [t, pe, ve, qe], [wb, fb], lla
	static constexpr auto out_len = 7;

	static constexpr auto prm_len = 14;

	void on_time_step()
	{
		if(m_ini_st > fcc_t::st_init)
		{
			fcc.on_reset(ins, m_ini_st);
			m_ini_st = fcc_t::st_init;
		}
		fcc.on_time(ins);
	}
	void fill_params(const double*, unsigned);
	void fill_gains (const double*, unsigned);
	void fill_wire  (const double*, unsigned);
	void initial_state(unsigned st)
	{
		if(st > fcc_t::st_init && st < fcc_t::st_orbit)
			m_ini_st = fcc_t::e_states(st);
	}
	fcc_dbg_t();

private:

	unsigned m_stage;
	fcc_t::e_states m_ini_st;
	second_t m_sc_sample;
};


}



