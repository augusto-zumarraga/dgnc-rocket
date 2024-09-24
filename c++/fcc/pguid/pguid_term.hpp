/*============================================================================*/
/*                                               Guiado, Navegación y Control */
/*============================================================================*/

////////////////////////////////////////////////////////////////////////////////
/// \brief    CONFIGURATION GUIDANCE FOR PICO LAUNCHERS (API-PGUID)
/// \author   Alberto Fraguío
/// \author   Augusto Zumarraga (C++ coding)
/// \date     creación: 13/08/2024
/// \date     revisión: 31/08/2024
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

#include "pguid_def.hpp"

namespace gnc { namespace guid {

//------------------------------------------------------------------------------
/// Navegación Terminal
//------------------------------------------------------------------------------
struct term_t
{
	struct params_t
	{
		second_t Tg      ;  // [s]       Terminal guidance sampling time (please, don't change before consulting)
		scalar   VD      ;  // [m/s]     Rapidez objetivo
		scalar   Vepsilon;  // [m/s]     Terminal guidance stop criteria

		void operator=(const guid::params_t&);

	} params;
	const out_t& out() const
	{
		return m_out;
	}
	void fill  (tlmy_t&);
	void init  (const nav_t&, const ltg_t&);
	///  \return true if is done
	bool update(const nav_t&);

protected:

	second_t  ti, tf, Tg;
	direction ulambda_I  ;
	scalar    Vmiss;
	out_t     m_out;
} ;

}}


