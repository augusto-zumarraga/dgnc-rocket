/*============================================================================*/
/*                                                                            */
/*============================================================================*/

////////////////////////////////////////////////////////////////////////////////
/// \brief
/// \author   Augusto Zumarraga
/// \date     creación: 10/06/2024
/// \date     revisión: 17/06/2024
/// \date     revisión: 26/06/2024 refactoring
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
#include <map>

namespace dgnc { namespace rocket { namespace msdat {

class coef_t : public data::t_table_3<scalar, values_t>
{
public:

	typedef data::t_table_3<scalar, values_t> base_t;

	/// Layout: [M [β,α]]
	coef_t(mchs m, slps b, aoas a);
	coef_t()
	{}
	template <typename I>
    void set(mch, slp, I);
    coef_t& load(const file_t&);
    const coef_t& save(file_t&) const;

	unsigned mach_size() const { return num_blks(); }
	unsigned beta_size() const { return num_rows(); }
	unsigned alfa_size() const { return num_cols(); }
};

//------------------------------------------------------------------------------
struct finset_t
{
	values_t SSPAN;
	values_t CHORD;
	scalar  SWEEP;
	scalar  STA  ;
	scalar  XLE  ;
	//scalar NPANEL PHIF.size();
	values_t PHIF;

	finset_t& load(const file_t&);
	const finset_t& save(file_t&) const;
};

//------------------------------------------------------------------------------
struct axibod_t
{
	string TNOSE;
	scalar LNOSE ;
	scalar DNOSE ;
	scalar LCENTR;
	scalar DCENTR;

	axibod_t& load(const file_t&);
	const axibod_t& save(file_t&) const;
};

//------------------------------------------------------------------------------
struct ranges_t
{
    range_t alfa; ///< rango de ángulos de ataque
    range_t beta; ///< rango de ángulos de deslizamiento
    range_t mach; ///< rango de números de Mach
};

//------------------------------------------------------------------------------
struct data_t : public ranges_t
{
    scalar xref; ///< posición del punto de referencia
    scalar sref; ///< superficie de referencia
    scalar lref; ///< longitud de referencia

    coef_t cx   ; ///< fuerzas axiales
    coef_t cy   ; ///< fuerzas laterales
    coef_t cz   ; ///< fuerzas normales
    coef_t cl   ; ///< momento de rolido
    coef_t cm   ; ///< momento de cabeceo
    coef_t cn   ; ///< momento de guiñada

    coef_t cL   ; ///< sustentacion
    coef_t cD   ; ///< resistencia
    coef_t xcp  ; ///< posición del centro de presión (desde la proa)

    coef_t cza  ; ///< derivada del coeficiente de fuerzas normales con alpha
    coef_t cyb  ; ///< derivada del coeficiente de fuerzas laterales con beta
    coef_t clb  ; ///< derivada del coeficiente de momentos de rolido con beta
    coef_t cma  ; ///< derivada del coeficiente de momentos con alpha
    coef_t cnb  ; ///< derivada del coeficiente de momentos de guiñada con beta

    coef_t clp  ; ///< momento de rolido debido a la velocida de rolido
    coef_t cnp  ; ///< momento de guiñada debido a la velocida de rolido

    coef_t cxq  ; ///< fuerzas axiales debido a la velocida de cabeceo
    coef_t czq  ; ///< fuerzas normales debido a la velocida de cabeceo
    coef_t cmq  ; ///< momento de cabeceo debido a la velocida de cabeceo

    coef_t cyr  ; ///< fuerzas laterales debido a la velocida de guiñada
    coef_t cyp  ; ///< fuerzas laterales debido a la velocida de rolido

    coef_t clr  ; ///< momento de rolido debido a la velocida de guiñada
    coef_t cnr  ; ///< momento de guiñada debido a la velocida de guiñada

    coef_t czad ; ///< fuerzas axiales debido la velocidad de alpha
    coef_t cmad ; ///< momento de cabeceo debido a la velocida de alpha

    axibod_t axibod;
    typedef std::vector<finset_t> fins_t;
    fins_t fins;

    const
    data_t& save(std::string fpath) const;
    data_t& load(std::string fpath);
    data_t& import(std::string fpath, std::string fname = std::string());
};

}}}




