/*============================================================================*/
/*                                               Guiado, Navegación y Control */
/*============================================================================*/

////////////////////////////////////////////////////////////////////////////////
/// \brief
/// \author   Augusto Zumarraga
/// \date     creación: 27/06/2024
/// \date     revisión: 05/09/2024
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
#pragma once

#include "fcc_def.hpp"

namespace gnc {

class pguid_t;

//------------------------------------------------------------------------------
class flight_t
{
public:

	scalar R_orbit;

	struct times_t
	{
		second_t sampling;
		second_t s1_burn;
		second_t s2_burn;
		second_t meco_advance;
		second_t s1_coast_time;
		second_t separation_time;
		second_t s2_fire_delay;

	} times;

	struct params_t
	{
		float hgt_relief;  ///< altura para comenzar el alivio de cargas 1ra etapa
		float hgt_sep;     ///< altura para iniciar la separación
		float hgt_strn;    ///< altura para iniciar el guiado activo
		float hgt_void;    ///< altura para liberar cofia
		float exit_speed;  ///< velocidad de escape del motor de 2da etapa
		float pre_cycle;   ///< tiempo de pre-ciclado del guiado activo
		float tgo_min;     ///< tiempo para conmutar del LTG al TERM
		float v_eps;       ///< error de velocidad para finalizar TERM
		float f_min;       ///< aceleración mínima necesaria para el TLG

	} params;

	//t_schedule<att_t, N1> wire;
	t_interp <vector, N1> wire; //_v;

	flight_t();
	bool separation_completed(second_t elps) const { return (elps - m_tm_sep) > times.separation_time; }

protected:

	friend class fcc_t;

	void setup(pguid_t&);
	void start(second_t tm) { m_tm_off = tm; } //wire.set_offset(tm); }

	second_t      time_offset()              const { return m_tm_eci; }
	second_t    s1_coast_time()              const { return times.s1_coast_time; }
	second_t  separation_time()              const { return times.separation_time; }
	second_t    s2_fire_delay()              const { return times.s2_fire_delay; }
	second_t     meco_advance()              const { return times.meco_advance; }
	bool  start_meco_maneuver(second_t elps) const { return (elps - m_tm_off) >= times.s1_burn - times.meco_advance; }
	void     start_separation(second_t elps)       { m_tm_sep = elps; }
	bool             s1_fired(double fx)     const { return fx  > wgs84::go * 1.1; }
	bool             s2_fired(double fx)     const { return fx  > params.f_min; }
	bool        do_separation(float hgt)     const { return hgt > params.hgt_sep; }
	bool          do_steering(float hgt)     const { return hgt > params.hgt_strn; }
	bool           do_release(float hgt)     const { return hgt > params.hgt_void; }
	bool            do_relief(float hgt)     const { return hgt > params.hgt_relief; }

	second_t m_tm_off, m_tm_eci, m_tm_sep;
};

}



