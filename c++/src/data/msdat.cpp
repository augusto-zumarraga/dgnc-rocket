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
#include <dgnc/rocket/data/msdat.hpp>

using namespace dgnc;
using namespace rocket;
using namespace msdat;

//------------------------------------------------------------------------------
coef_t::coef_t(mchs m, slps b, aoas a)
{
	resize(b, a, m);
}
coef_t& coef_t::load(const file_t& f)
{
	unsigned Nb;
	unsigned Nr;
	unsigned Nc;
	f.read(Nb);
	f.read(Nr);
	f.read(Nc);
	resize(data::rows(Nr), data::cols(Nc), data::blks(Nb));
	f.read(static_cast<values_t&>(*this));
	return *this;
}
const coef_t& coef_t::save(file_t& f) const
{
	unsigned Nb = num_blks();
	unsigned Nr = num_rows();
	unsigned Nc = num_cols();
	f.write(Nb);
	f.write(Nr);
	f.write(Nc);
	f.write(static_cast<const values_t&>(*this));
	return *this;
}

//------------------------------------------------------------------------------
//	values_t SSPAN;
//	values_t ;
//	scalar  SWEEP;
//	scalar  STA  ;
//	scalar  XLE  ;
//	//scalar NPANEL PHIF.size();
//	values_t ;

finset_t& finset_t::load(const file_t& f)
{
	f.read(SSPAN);
	f.read(CHORD);
	f.read(SWEEP);
	f.read(STA  );
	f.read(XLE  );
	f.read(PHIF);
	return *this;
}
const finset_t& finset_t::save(file_t& f) const
{
	f.write(SSPAN);
	f.write(CHORD);
	f.write(SWEEP);
	f.write(STA  );
	f.write(XLE  );
	f.write(PHIF);
	return *this;
}

//------------------------------------------------------------------------------
//	string TNOSE;
//	scalar ;
//	scalar ;
//	scalar ;
//	scalar ;
axibod_t& axibod_t::load(const file_t& f)
{
	f.read(TNOSE );
	f.read(LNOSE );
	f.read(DNOSE );
	f.read(LCENTR);
	f.read(DCENTR);
	return *this;
}
const axibod_t& axibod_t::save(file_t& f) const
{
	f.write(TNOSE );
	f.write(LNOSE );
	f.write(DNOSE );
	f.write(LCENTR);
	f.write(DCENTR);
	return *this;
}

//------------------------------------------------------------------------------
data_t& data_t::load(std::string fpath)
{
	file_t f;
	f.open(fpath, "r");

	f.read(static_cast<values_t&>(alfa));
	f.read(static_cast<values_t&>(beta));
	f.read(static_cast<values_t&>(mach));

	f.read(xref);
	f.read(sref);
	f.read(lref);

    cx  .load(f);
    cy  .load(f);
    cz  .load(f);
    cl  .load(f);
    cm  .load(f);
    cn  .load(f);
    cL  .load(f);
    cD  .load(f);
    xcp .load(f);
    cza .load(f);
    cyb .load(f);
    clb .load(f);
    cma .load(f);
    cnb .load(f);
    clp .load(f);
    cnp .load(f);
    cxq .load(f);
    czq .load(f);
    cmq .load(f);
    cyr .load(f);
    cyp .load(f);
    clr .load(f);
    cnr .load(f);
    czad.load(f);
    cmad.load(f);

    axibod.load(f);

	unsigned n_count = 0;
	f.read(n_count);
	fins.resize(n_count);
	for(fins_t::iterator i = fins.begin(), e = fins.end(); i != e; ++i)
		i->load(f);

	return *this;
}
const data_t& data_t::save(std::string fpath) const
{
	file_t f;
	f.open(fpath, "w");

	f.write(static_cast<const values_t&>(alfa));
	f.write(static_cast<const values_t&>(beta));
	f.write(static_cast<const values_t&>(mach));

	f.write(xref);
	f.write(sref);
	f.write(lref);

    cx  .save(f);
    cy  .save(f);
    cz  .save(f);
    cl  .save(f);
    cm  .save(f);
    cn  .save(f);
    cL  .save(f);
    cD  .save(f);
    xcp .save(f);
    cza .save(f);
    cyb .save(f);
    clb .save(f);
    cma .save(f);
    cnb .save(f);
    clp .save(f);
    cnp .save(f);
    cxq .save(f);
    czq .save(f);
    cmq .save(f);
    cyr .save(f);
    cyp .save(f);
    clr .save(f);
    cnr .save(f);
    czad.save(f);
    cmad.save(f);

    axibod.save(f);

	unsigned n_count = fins.size();
	f.write(n_count);
	for(fins_t::const_iterator i = fins.begin(), e = fins.end(); i != e; ++i)
		i->save(f);

	return *this;
}


