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
class fcc_loader_t
{
public:

	fcc_t fcc;

	void fill_params(const double*, unsigned);
	void fill_gains (const double*, unsigned);
	void fill_wire  (const double*, unsigned);

	fcc_loader_t() : m_stage(0)
	{}

protected:

	unsigned m_stage;
};

//------------------------------------------------------------------------------
class fcc_dbg_t : public fcc_loader_t
{
public:

	ins_data_t ins;

	static constexpr auto sns_len = (1 + 3 + 3 + 4) + (3 + 3) + 3; // [t, pe, ve, qe], [wb, fb], lla
	static constexpr auto aos_len = 4; // [dy dz da rc]
	static constexpr auto dos_len = 3; // [eng sep rel]
	static constexpr auto out_len = 1+4+3+3+(1+1+1+1+3); // state, q_ref[4], e_dir[3], w_ref[3],
                                                         // guid {state, ttgo, rtgo,rbis, pref[3]}
	static constexpr auto prm_len = 18;

	void on_time_step()
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
	}
	void initial_state(int st, second_t t_launch, second_t t_sep);
    void write_tlmy (double*);

	fcc_dbg_t();

private:

	int      m_ini_st;
    second_t m_t_launch;
    second_t m_t_separation;
};

}

