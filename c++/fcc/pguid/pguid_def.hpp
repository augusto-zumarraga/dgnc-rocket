/*============================================================================*/
/*                                               Guiado, Navegación y Control */
/*============================================================================*/

////////////////////////////////////////////////////////////////////////////////
/// \brief    CONFIGURATION GUIDANCE FOR PICO LAUNCHERS (API-PGUID)
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
#pragma once

#include "../fcc_def.hpp"

namespace gnc { namespace guid {

//------------------------------------------------------------------------------
/// Datos de navegación necesarios para el guiado activo
struct nav_t : public eci::state_t
{
	nav_t(const ecef::state_t& e) : eci::state_t(e), Fs(0)
	{}
	nav_t() : Fs(0)
	{}
	force_t sfc;  // fshat_I
	vector  vxp;  // nv.vel ^ nv.pos
	scalar  Fs;
};

/// Datos de salida del guiado activo
struct out_t
{
	direction dir;  ///< dirección ECI del empuje
	vector    rate; ///< derivada de la dirección
};

/// Telemetría del guiado activo
struct tlmy_t
{
	int       state;
	scalar    ttgo;
	scalar    rtgo;
	scalar    rbis;
	direction pref;

	void reset(int st = 0)
	{
		state = st;
		ttgo  = 0;
		rtgo  = 0;
		rbis  = 0;
		pref  = 0;
	}
};

/// Configuración para el guiado activo
struct params_t
{
	second_t ts;
	second_t tburn;
	second_t tcycl;
	second_t tgomin;

	scalar rD;
	scalar Ve;
	scalar VD;
	scalar Veps;
	scalar Fsmin;

	params_t( scalar   _Ve     ///< velocidad de escape del motor
			, scalar   _rD     ///< radio orbital
			, second_t _ts     ///< tiempo de muestreo
			, second_t _tburn  ///< tiempo de quema
			, second_t _tc     ///< tiempo de pre-ciclado para el LTG
			, second_t _tgomin ///< tiempo para conmutar al guiado terminal
			, scalar   _Veps   ///< error de velocidad para finalizar TERM
			, scalar   _Fsmin) ///< aceleración mínima necesaria para el TLG
	: ts    (_ts        )
	, tburn (_tburn     )
	, tcycl (_tc        )
	, tgomin(_tgomin    )
	, rD    (_rD        )
	, Ve    (_Ve        )
	, VD    (sqrt(wgs84::GM/_rD))
	, Veps  (_Veps)
	, Fsmin (_Fsmin)
	{}
};

class ltg_t;
class term_t;

}}


