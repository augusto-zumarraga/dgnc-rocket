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

class fcc_loader_t
{
public:

	fcc_t fcc;
	static constexpr auto prm_len = 18;

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
	fcc_dbg_t();
	void on_time_step();
	void initial_state(int st, second_t t_launch, second_t t_sep);
    void write_tlmy (double*);

private:

    void on_state();

    int      m_state;
    int      m_ini_st;
    second_t m_t_launch;
    second_t m_t_separation;
};

}


