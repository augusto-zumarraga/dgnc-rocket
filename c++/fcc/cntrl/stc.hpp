/*============================================================================*/
/*                                               Guiado, Navegación y Control */
/*============================================================================*/

////////////////////////////////////////////////////////////////////////////////
/// \brief	   Super-Twisting Control
/// \author   Augusto Zumarraga
/// \date     creación: 13/05/2019
/// \date     revisión: 03/09/2024
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

namespace gnc { namespace stc {

template <typename T>
inline T sign(T x)
{
	return std::signbit(x) ? -1 : 1; //(T(0) < x) - (x < T(0));
}

template <typename T, typename P>
inline T sabs(T x, P p)
{
    return x ? sign(x) * pow(std::abs(x), p) : 0;
}

//______________________________________________________________________________
struct phi_3_t
{
	phi_3_t(double k = 1) : k2(k)
	{}
    double operator()(double x1, double x2) const
	{
    	return k2*sabs(x1, 2.0/3.0) + x2;
	}
	double k2;
	operator bool() const { return k2 > 0; }
};

//______________________________________________________________________________
// Controlador ST con discretización implícita
class implicit_t
{
public:

	struct out_t
	{
		double u;
		double u_dot;
	};

	implicit_t(double l1 = 1, double l2 = 1)
    : m_l1(l1), m_l2(l2), m_nk(0)
    {}
	double   p_gain() const { return m_l1; }
	double   i_gain() const { return m_l2; }
	operator double() const { return m_nk; }
	operator bool  () const { return m_l1 > 0; }

	void setup(double l1, double l2)
    {
		m_l1 = l1;
		m_l2 = l1 ? l2 : 0;
	}
	/// \param xk variable de conmutación
	/// \param h  tiempo de integración
	out_t operator()(const double xk, const double h, const double u_max)
	{
	    double l1   = m_l1;
	    double l2   = m_l2;
	    double a    = h*l1;
	    double a2   = a*a;
	    double h2   = h*h;
	    double hl2  = h *l2;
	    double h2l2 = h2*l2;

	    double up;

	    double z, u, ud, bk = -xk - h*m_nk;
	    if(bk <-h2l2)
	    {
	        z  = -a + sqrt(a2 - 4*(bk + h2l2))/2;
	        m_nk -=  hl2;
	        up = -l1*z;
	        ud = -l2;
	    }
	    else
	    if(bk > h2l2)
	    {
	        z  = -a + sqrt(a2 + 4*(bk - h2l2))/2;
	        m_nk +=  hl2;
	        up =  l1*z;
	        ud =  l2;
	    }
	    else
	    {
	        ud   = -m_nk;
	    	m_nk = -xk / h;
	    	up   = 0;
	        u    = m_nk;
	        ud  += m_nk;
	        ud  /= h;
	    }
        u = up + m_nk;

	    if(sat(u, u_max))
	    {
	    	u  = sign(u) * u_max;
	    	up = sat(up, u_max) ? sign(up) * u_max : up;
			m_nk = u - up;
	    }
		return { u, ud };
    }
	void reset(double u, double s)
	{
		m_nk = u + m_l1*sabs(s, 0.5);
	}
	void reset()
	{
		m_nk = 0;
	}

protected:

	static bool sat(double val, double max)
	{
		return std::abs(val) > max;
	}
	double m_l1, m_l2, m_nk;
};

}}


