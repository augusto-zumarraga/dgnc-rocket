/*============================================================================*/
/*                                                                            */
/*============================================================================*/

////////////////////////////////////////////////////////////////////////////////
/// \brief
/// \author   Augusto Zumarraga
/// \date     creación: 10/06/2024
/// \date     revisión: 17/06/2024
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

namespace msdat {
class  coef_t;
struct ranges_t;
struct data_t;
}

namespace aero {

struct interp_set_t;

//------------------------------------------------------------------------------
struct ranges_t
{
    range_t  alfa;   // α ángulo de ataque
    range_t  beta;   // β ángulo de deslizamiento
    range_t  mach;   // número de Mach

    interp_set_t operator()(scalar aoa, scalar slp, scalar mch) const;

    const
    ranges_t& save(file_t&) const;
    ranges_t& load(const file_t&);
};

//------------------------------------------------------------------------------
class simetry
{
public:

	simetry(bool _aoa = true, bool _slp = true)
	: m_aoa(_aoa > 0 ? 1 : -1), m_slp(_slp > 0 ? 1 : -1)
	{}
	double aoa() const { return m_aoa; }
	double slp() const { return m_slp; }

private:

	double m_aoa;
	double m_slp;
};

//------------------------------------------------------------------------------
class coefs_t
{
public:

	typedef spline_2_t               incidence_t;
	typedef std::vector<incidence_t> mach_t;
	typedef data::t_table_3<float>   set_t;

	coefs_t& update(const msdat::ranges_t&, const msdat::coef_t&, simetry s);
	/// verificar previamente la validez del argumento
	scalar   operator()(const interp_set_t& s) const;
	set_t    operator()(const ranges_t&, const values_t& m, const values_t& s, const values_t& a) const;

    const
	coefs_t& save(file_t&) const;
	coefs_t& load(const file_t&);

private:

	mach_t  m_data;
	simetry m_sym;
};

//------------------------------------------------------------------------------
struct force_coefs
{
    coefs_t cL ;
    coefs_t cD ;
    coefs_t cx ;
    coefs_t cy ;
    coefs_t cz ;
    coefs_t cyp;    // ∂cy/∂p
    coefs_t cxq;    // ∂cx/∂q
    coefs_t czq;    // ∂cz/∂q
    coefs_t cyr;    // ∂cy/∂r

    const
	force_coefs& save(file_t&) const;
    force_coefs& load(const file_t&);
};

struct moment_coefs
{
    coefs_t cl ;
    coefs_t cm ;
    coefs_t cn ;
    coefs_t clp;    // ∂cl/∂p
    coefs_t cnp;    // ∂cn/∂p
    coefs_t cmq;    // ∂cm/∂q
    coefs_t clr;    // ∂cl/∂r
    coefs_t cnr;    // ∂cn/∂r

    const
	moment_coefs& save(file_t&) const;
    moment_coefs& load(const file_t&);
};

struct roll_finset_t
{
	spline_1_t Ld;    // (∂L(M)/∂δ) /Q
	// El momento de rolido se obtiene multiplicando Q.Ld.δ

	struct params_t
	{
    scalar S;    // superficie de la aleta
    scalar b;    // envergadura de la aleta
    scalar r;    // radio hasta la CAM
    scalar N;    // cantidad de aletas
    scalar dmax; // δ máx
	};
    operator bool() const { return Ld.size() > 0;}
    scalar operator()(const range_t::frac_t& mach) const;

    // REF: Sref*Lref
	void build(const range_t& mach, const params_t& s, scalar REF, scalar a0 = 2*math::pi_);
};

//------------------------------------------------------------------------------
struct model
{
    scalar sref;
    scalar lref;
    scalar xref;

    ranges_t     R;
    force_coefs  F;
    moment_coefs M;

    roll_finset_t roll_fins;

    /// resultante aerodinámica
    force_t force (const wind_t&) const;
    /// momento respecto del punto de referencia aerodinámico en coordenadas b
    moment_t moment(const wind_t&, scalar da = 0) const;

    struct result_t
    {
    	force_t force;
    	moment_t moment;
    	result_t() : force(0), moment(0)
    	{}
    };
    result_t operator()(const wind_t&, scalar da = 0) const;
    /// p_ref: posición del punto de referencia aerodinámico en coordenadas b
    result_t operator()(const wind_t& wnd, const position_t& p_ref, scalar da = 0) const
    {
    	result_t r = (*this)(wnd, da);
    	r.moment += p_ref ^ r.force;
    	return r;
    }

    const
	model& save(std::string fpath) const;
    model& load(std::string fpath);
    void  setup(const msdat::data_t&);
    void  setup(const roll_finset_t::params_t&);
};


}}}



