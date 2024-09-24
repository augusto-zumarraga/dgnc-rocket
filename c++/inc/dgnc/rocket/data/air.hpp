/*============================================================================*/
/*                                                                            */
/*============================================================================*/

////////////////////////////////////////////////////////////////////////////////
/// \brief
/// \author   Augusto Zumarraga
/// \date     creación: 26/06/2024
/// \date     revisión: 26/06/2024
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
#include "data.hpp"

namespace dgnc { namespace rocket {

//------------------------------------------------------------------------------
class air_t
{
public:

	air_t(scalar h);
	air_t(scalar r, scalar p, scalar s, scalar t)
	: rho(r), po(p), snd(s), T(t)
	{}
	air_t()
	: rho(0), po(0), snd(0), T(0)
	{}
    
	scalar rho; // densidad
	scalar po;  // presión
	scalar snd; // velocidad del sonido
    scalar T;   // temperatura absoluta
};


//------------------------------------------------------------------------------
/// coordenadas ISO 1151
class wind_t
{
public:

	/// vb : velocidad lineal del cuerpo respecto del aire
	/// wb : velocidad angular del cuerpo respecto del aire
	/// pr : posición del punto de referencia aerodinámico en coordenadas b
	wind_t(air_t, velocity_t vb, angle_rate_t wb, position_t pr);
	/// xref: posición del centro de referencia aerodinámico en el eje xb
	wind_t(air_t, velocity_t vb, angle_rate_t wb, double xref = 0);

	wind_t(radian a = 0, radian b = 0, double m = 0, double v = 0, double q = 0, vector w = vector(0,0,0))
	: aoa(a), slp(b), mch(m), vel(v), Q(q), wb(w)
	{}

	radian aoa;  // ángulo de ataque
	radian slp;  // ángulo de deslizamiento
	scalar mch;  // número de Mach
	scalar vel;  // módulo de la velocidad
	scalar Q  ;  // presión dinámica
	angle_rate_t wb ;  //
};

}}

