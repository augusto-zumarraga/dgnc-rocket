/*============================================================================*/ 
/*                                                                            */
/*============================================================================*/ 

/*!                                                                             
 \file     debug.hpp
 \brief                                                                         
 \author   Augusto Zumarraga                                                    
 \date     creación: 07/08/2024
 \date     revisión: 09/09/2024
 \note     versión : 0.2
______________________________________________________________________________*/
#pragma once

#include "../../../fcc/fcc.hpp"

namespace gnc {

void trace(const char*);

//------------------------------------------------------------------------------
class fcc_dbg_t
{
public:

	ins_data_t ins;
	fcc_t      fcc;

	static constexpr auto sns_len = (1 + 3 + 3 + 4) + (3 + 3) + 3; // [t, pe, ve, qe], [wb, fb], lla
	static constexpr auto aos_len = 4; // [dy dz da rc]
	static constexpr auto dos_len = 3; // [eng sep rel]
	static constexpr auto out_len = 2; // fcc state, pguid state

	static constexpr auto prm_len = 18;

	void on_time_step()
	{
		if(m_ini_st == fcc_t::st_init && ins.elapsed >= m_t_launch)
        {
            fcc.launch(ins);
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
	}
	void fill_params(const double*, unsigned);
	void fill_gains (const double*, unsigned);
	void fill_wire  (const double*, unsigned);
	void initial_state(int st, second_t t_launch, second_t t_sep);

	fcc_dbg_t();

private:

	unsigned m_stage;
	int      m_ini_st;
    second_t m_t_launch;
    second_t m_t_separation;
};

}

