/*============================================================================*/
/*                                               Guiado, Navegación y Control */
/*============================================================================*/

////////////////////////////////////////////////////////////////////////////////
/// \brief
/// \author   Augusto Zumarraga
/// \date     creación: 25/08/2024
/// \date     revisión: 03/09/2024
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

#include "../sched.hpp"
#include "stc.hpp"
#include "rate.hpp"

namespace gnc { namespace ctrl {

//------------------------------------------------------------------------------
/// Estructura de configuración de los controles 2-STC y 3-STC
struct st_params_t
{
	double k2;
	double l1;
	double l2;
	double rl; // rate limit
};

//------------------------------------------------------------------------------
/// Control de velocidad de giro en un eje.
/// En función de la configuración se hace un control proporcional, 2-STC (si
/// l1 y l2 > 0) o 3-STC (si además k2 > 0).
/// Se acondiciona la referencia por amplitud y derivada antes del cálculo
struct axis_t
{
	stc::implicit_t        stc; ///< control ST
	stc::implicit_t::out_t out; ///< resultados del lazo de control ST
	stc::phi_3_t           phi; ///< cálculo del ϕ3 para algoritmo 3-STA
	double               r_lim; ///< límite para la referencia
	rate_t               e_rate; ///< derivada del error
	rate_limit_t         r_rate; ///< derivada de la referencia (limitada)

	axis_t();
	/// Configuración
	void setup(const st_params_t&);
	/// Reinicio de los cómputos de derivada
	void reset(double x);
	/// Cómputo de la acción de control
	/// \return : acción de control u (comando para el actuador)
	/// \param r: referencia
	/// \param y: valor medido
	/// \param k: ganancia (u = k·ν, siendo ν es el resultado del algoritmo)
	/// \param h: tiempo de muestreo
	double operator()(double& r, const double y, const double k, const double h);
};

//------------------------------------------------------------------------------
/// Control de velocidad de giro en los ejes yb y zb.
/// La agenda de ganancias es la misma para ambos ejes.
/// Esta es una base común para el control de velocidad angular para vuelo
/// atmosférico y exo-atmosférico
class tvc_yz_t
{
public:

	t_schedule<double, NM> gains;
	axis_t q_ctrl, r_ctrl;

	tvc_yz_t();
	template <class G>
    void setup(second_t ts, const st_params_t& p, const G& g)
    {
    	m_ts = ts;
    	q_ctrl.setup(p);
    	r_ctrl.setup(p);
    	gains.fill(g.begin(), g.size(), ts);
    }
    void reset ();
	void cloop (const ins_data_t& ins, double q_ref, double r_ref);
	void update(cmnds::cont_t& c) { c.dy = m_dy; c.dz = m_dz; }
	void set_null()	              { m_dy = m_dz = 0; }

protected:

	double m_ts;        ///< tiempo de muestreo
	double m_dy, m_dz;  ///< comandos
};

//------------------------------------------------------------------------------
/// Control aerodinámico de velocidad de giro en el eje xb durante el vuelo
/// atmosférico.
class roll_fin_t
{
public:

	t_schedule<double, NM> gains;
	axis_t p_ctrl;

	roll_fin_t();
	template <class G>
    void setup(second_t ts, const st_params_t& p, const G& g)
    {
    	m_ts = ts;
    	p_ctrl.setup(p);
    	gains.fill(g.begin(), g.size(), ts);
    }
    void reset ();
	void cloop (const ins_data_t& ins, double p_ref);
	void update(cmnds::cont_t& c) { c.da = m_da;}
	void set_null()               { m_da = 0; }

protected:

	double m_ts;   ///< tiempo de muestreo
	double m_da;   ///< comandos
};

//------------------------------------------------------------------------------
/// Control RCS para rolido durante el vuelo exo-atmosférico
class roll_rcs_t
{
public:

	t_schedule<double, NM> gains;
	roll_rcs_t();
	template <class G>
    void setup(second_t ts, const G& g)
    {
    	gains.fill(g.begin(), g.size(), ts);
    }
    void reset();
	void cloop(const ins_data_t& ins, double p_ref);
	void update(cmnds::cont_t& c);
	void set_null() { m_rc = 0; }

protected:

	double m_rc;   ///< comando
};


//==============================================================================
/// Estructura de telemetría para los controles
struct tlmy_t
{
	angle_rate_t w_ref;
	void reset()
	{
		w_ref = 0;
	}
};

//------------------------------------------------------------------------------
/// Base común para el control de apuntamiento (actitud y, z) en vuelo
/// atmosférico y exo-atmosférico.
/// La agenda de ganancias permite establecer anchos de banda de estos lazos
class ctrl_t
{
public:

	t_schedule<double, NM> py_gains; ///< pitch/yaw gains

	const cmnds::cont_t& cmnd() const { return m_cmnd; }
	/// actualizar datos de telemetría
    void update(tlmy_t& s) const;
	/// detener el movimiento (velocidad angular nula)
	void stop_rotation()
	{
		m_w_ref = 0;
	}
	void low_thrust()
	{
		m_cmnd.dy = m_cmnd.dz = 0;
	}
	tvc_yz_t& tvc() { return m_tvc; }

protected:

	void pointing_loop(const ins_data_t& ins, const vector& rx, double g);

	angle_rate_t  m_w_ref; ///< referencia para la velocidad angular
	cmnds::cont_t m_cmnd;
	  tvc_yz_t    m_tvc;
};

//------------------------------------------------------------------------------
/// Control de actitud y velocidad angular en fase atmosférica
class atm_t : public ctrl_t
{
public:

	t_schedule<double, NM> r_gains;

	/// inicializar agenda de ganancias y controles
	void start(second_t elps);
	/// reset de controles y comandos
	void reset();

	/// control individual de velocidad de rolido
	void roll_loop(const ins_data_t& ins);
	/// Control para las tres componentes de velocidad angular
	void pqr_loop(const ins_data_t& ins);

	/// \param q_err cuaternion de error
	void att_loop(const quaternion& r, const quaternion& q, const ins_data_t& ins);
	/// \return referencia de velocidad angular
	void load_relief(const  att_t& qr, const ins_data_t& ins);
	/// \param rx dirección deseada del eje x en terna b
	void pointing_loop(const vector& rx, const ins_data_t& ins);

	roll_fin_t& fin() { return m_fin; }

protected:

	roll_fin_t m_fin;
};

//------------------------------------------------------------------------------
/// Control de actitud y velocidad angular en fase exo-atmosférica
class exo_t : public ctrl_t
{
public:

	t_schedule<double, NM> r_gains;

	/// inicializar agenda de ganancias y controles
	void start(second_t elps);
	/// reset de controles y comandos
	void reset();
	/// control individual de velocidad de rolido
	void roll_loop(const ins_data_t& ins);
	/// \param rx dirección deseada del eje x en terna b
	void pointing_loop(const vector& rx, const ins_data_t& ins);

	roll_rcs_t& rcs() { return m_rcs; }

protected:

	roll_rcs_t m_rcs;
};

}}


