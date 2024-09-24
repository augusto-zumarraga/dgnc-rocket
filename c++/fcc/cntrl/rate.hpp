/*============================================================================*/
/*                                               Guiado, Navegación y Control */
/*============================================================================*/

////////////////////////////////////////////////////////////////////////////////
/// \brief
/// \author   Augusto Zumarraga
/// \date     creación: 13/05/2019
/// \date     revisión: 05/09/2024
//______________________________________________________________________________
#pragma once
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
#include <math.h>

namespace gnc {

template <typename T>
inline T sign(T x)
{
	return std::signbit(x) ? -1 : 1; //(T(0) < x) - (x < T(0));
}

//______________________________________________________________________________
/// Derivada por diferencias finitas
class rate_t
{
public:

	rate_t() : m_prev(NAN)
	{}
    /// reinicio del cómputo de derivada
    /// \param x  valor inicial de entrada
	void reset(double x = NAN) { m_prev = x;	}
    /// idem reset()
	void operator=(double x)   { m_prev = x;	}
    /// \return   valor previo
	operator double() const	   { return m_prev;	}
    /// \param x  valor actual
    /// \param h  tiempo de muestreo
    /// \return   derivada
    double operator()(double x, double h)
    {
    	double v = isnan(m_prev) ? 0 : (x - m_prev)/h;
    	m_prev = x;
    	return v;
    }

protected:

    double m_prev;
};

//______________________________________________________________________________
/// Derivada con saturación
class rate_limit_t
{
public:

	rate_limit_t(double l = 0.5)
	: lim(l)
	{}
    double lim; ///< límite para la derivada
    /// \param x  magnitud (se ajusta si excede la derivada)
    /// \param h  tiempo de muestreo
    /// \return   derivada saturada
    double operator()(double& x, const double h)
	{
    	double v = m_rate(x, h);
    	if(std::abs(v) > lim)
    	{
			v = lim * stc::sign(v);
			x = m_rate + h * v;
	    	m_rate = x;
    	}
    	return v;
	}
    /// reinicio del cómputo de derivada
    /// \param x  valor inicial de entrada
	void reset(double x = NAN)
	{
		m_rate.reset(x);
	}

protected:

    rate_t m_rate;
};

}


