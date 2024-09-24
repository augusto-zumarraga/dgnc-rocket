/*============================================================================*/
/*                                                                            */
/*============================================================================*/

////////////////////////////////////////////////////////////////////////////////
/// \brief    Modelo para la integración numérica de la dinámica de cuerpos
///           rígidos
/// \author   Augusto Zumarraga
/// \date     creación: 10/06/2024
/// \date     revisión: 05/07/2024
/// \date     revisión: 23/08/2024 refactoring
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

#include <dgnc/navs/wgs84.hpp>
#include <dgnc/navs/eci.hpp>
#include <dgnc/numeric/navs_integ.hpp>

namespace dgnc { namespace fsim {

using navs::position_t;
using navs::velocity_t;
using navs::angle_rate_t;

//------------------------------------------------------------------------------
struct nav_ecef_t
{
	typedef navs::ecef::state_t state_t;
	static state_t rate(const double t, const state_t& s, const navs::force_t& fb, navs::angle_rate_t wb)
	{
		return navs::wgs84::rate(s, fb, wb);
	}
};
struct nav_eci_t
{
	typedef navs::eci::state_t state_t;
	static state_t rate(const double t, const state_t& s, const navs::force_t& fb, navs::angle_rate_t wb)
	{
		return navs::eci::rate(s, fb, wb);
	}
};


//------------------------------------------------------------------------------
/// Estado de la dinámica angular
struct rot_state_t
{
	angle_rate_t wbi;

	rot_state_t& operator=(const angle_rate_t& w)
	{
		wbi = w;
		return *this;
	}
	rot_state_t operator+(const rot_state_t& rhs) const
	{
		rot_state_t res;
	    res.wbi = wbi + rhs.wbi;
	    return res;
	}
	rot_state_t operator*(double f) const
	{
		rot_state_t res;
	    res.wbi = wbi * f;
	    return res;
	}
};

//______________________________________________________________________________
//
/// Estado para la dinámica y navegación de un cuerpo rígido con 6GL
///
/// \tparam SFC: Base para el cómputo de fuerzas y momentos específicos
///              Esta clase debe definir su propio vector de estados state_t
///              e implementar un método compute(...) para calcular fuerza
///              específica y momento en terna b, junto con las propiedades
///              másicas. El resultado del cómputo debe ser un objeto rates_t
///              con métodos para acceder a estos resultados:
///                    - operator state_t  ()   d/dt(state_t)
///                    - specifc_force     ()   Fb/m
///                    - moment            ()   Mb
///                    - inertia_tensor    ()   J
///                    - inertia_tensor_inv()   J^{-1}
///
/// \tparam NAV: Estado para la navegación state_t: (t, pos, vel, att) y método
///              para calcular su derivada.
///
template <class SFC, class NAV = nav_ecef_t>
class t_rigid_body : public SFC, public NAV
{
public:

	typedef SFC                           force_mdl_t;
	typedef typename force_mdl_t::state_t frc_state_t;
	typedef typename force_mdl_t::rates_t rates_t;
	typedef typename NAV::state_t         nav_state_t;

	struct vect_t : public nav_state_t
	              , public frc_state_t
				  , public rot_state_t
	{
		typedef nav_state_t nav_t;
		typedef frc_state_t frc_t;
		typedef rot_state_t rot_t;

		const nav_t& nav() const { return *this; }
		const frc_t& frc() const { return *this; }
		const rot_t& rot() const { return *this; }

		nav_t& nav() { return *this; }
		frc_t& frc() { return *this; }
		rot_t& rot() { return *this; }

		vect_t operator*(double h) const
		{
			vect_t r;
			r.nav() = nav() * h;
			r.frc() = frc() * h;
			r.rot() = rot() * h;
			return r;
		}
		vect_t operator+(const vect_t& rhs) const
		{
			vect_t r;
			r.nav() = nav() + rhs.nav();
			r.frc() = frc() + rhs.frc();
			r.rot() = rot() + rhs.rot();
			return r;
		}
	};

	vect_t operator()(double t, const vect_t& x) const
	{
		 vect_t xdot;
		rates_t f = force_mdl_t::compute(t, x);
		xdot.nav() = NAV::rate(t, x, f.specifc_force(), x.wbi);
		xdot.rot() = f.inertia_tensor_inv()*(f.moment() - (x.wbi^(f.inertia_tensor()*x.wbi)));
		xdot.frc() = f;
		return xdot;
	}

	template <typename P1, typename P2, typename P3>
	t_rigid_body(const P1& p1, const P2& p2, const P3& p3) : SFC(p1, p2, p3)
	{}
	template <typename P1, typename P2>
	t_rigid_body(const P1& p1, const P2& p2) : SFC(p1, p2)
	{}
	template <typename P>
	t_rigid_body(const P& p) : SFC(p)
	{}
	t_rigid_body()
	{}
};

}}

inline std::ostream& operator<<(std::ostream& s, const dgnc::fsim::rot_state_t& x)
{
	s << x.wbi;
	return s;
}
template <class SFC, class NAV>
std::ostream& operator<<(std::ostream& s, const typename dgnc::fsim::t_rigid_body<SFC, NAV>::vect_t& v)
{
	s << v.nav() << ',' << v.rot() << ',' << v.frc();
	return s;
}


