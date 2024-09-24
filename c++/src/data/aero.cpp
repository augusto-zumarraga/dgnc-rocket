/*============================================================================*/
/*                                                                            */
/*============================================================================*/

// /////////////////////////////////////////////////////////////////////////////
/// \brief
/// \author   Augusto Zumarraga
/// \date     creacion: 12/06/2024
/// \date     revision: 09/09/2024
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
//#include <dgnc/numeric/interp.hpp>
#include <dgnc/rocket/data/aero.hpp>
#include <dgnc/rocket/data/msdat.hpp>
#include <dgnc/store/data_file.hpp>

namespace dgnc { namespace rocket { namespace aero {

struct interp_set_t
{
	range_t::frac_t mach;
	range_t::frac_t alfa;
	range_t::frac_t beta;
	scalar      sgn_alfa;
	scalar      sgn_beta;
};

}}}

using namespace dgnc;
using namespace rocket;
using namespace rocket::aero;
using numeric::interp::spline_2_t;

namespace {

typedef spline_2_t::input_t input_t;

struct patch_src_t
{
	scalar m_data[16];
	scalar operator()(unsigned i, unsigned j) const
	{
		return m_data[i*4+j];
	}
	template <typename T>
	patch_src_t(const T* s)
	{
		for(scalar* d = m_data, *e = d + 16; d < e; ++d, ++s)
			*d = *s;
	}
	template <typename T>
	patch_src_t& operator=(const T* s)
	{
		for(scalar* d = m_data, *e = d + 16; d < e; ++d, ++s)
			*d = *s;
		return *this;
	}
};
typedef msdat::coef_t::blocks_t src_t;

struct mach_data_t
{
	mach_data_t(msdat::coef_t::block_t src)
	: data(src)
	{}
	float operator()(unsigned i, unsigned j) const
	{
		return data(slp(i), aoa(j));
	}
	msdat::coef_t::block_t data;
};

//------------------------------------------------------------------------------
file_t& operator<<(file_t& f, const range_t& r)
{
	unsigned cnt = r.size();
	f.write(cnt);
	f.write(r.data(), cnt, sizeof(*r.data()));
	return f;
}
const
file_t& operator>>(const file_t& f, range_t& r)
{
	unsigned cnt = 0;
	f.read(cnt);
	if(cnt > 1000)
		throw std::runtime_error("range_t read: invalid count");
	r.resize(cnt);
	f.read(r.data(), cnt, sizeof(*r.data()));
	return f;
}

//------------------------------------------------------------------------------
file_t& operator<<(file_t& f, const spline_2_t::patch_t& p)
{
	f.write(&*p.coef.begin(), 16, sizeof(p.coef.get(0)));
	return f;
}
const
file_t&operator>>(const file_t& f, spline_2_t::patch_t& p)
{
	f.read(&*p.coef.begin(), 16, sizeof(p.coef.get(0)));
	return f;
}

file_t& operator<<(file_t& f, const spline_2_t& s)
{
	unsigned
	cnt = s.rows(); f.write(cnt);
	cnt = s.cols(); f.write(cnt);
	for(spline_2_t::patchs_t::const_iterator p = s.begin(), e = s.end(); p < e; ++p)
		f << *p;
	return f;
}
const
file_t& operator>>(const file_t& f, spline_2_t& s)
{
	unsigned rows = 0;
	unsigned cols = 0;
	f.read(rows);
	f.read(cols);
	if(rows > 100)
		throw std::runtime_error("spline_2_t read: invalid rows");
	if(cols > 100)
		throw std::runtime_error("spline_2_t read: invalid cols");

	spline_2_t::patchs_t& P = s.resize(rows, cols);
	for(spline_2_t::patchs_t::iterator p = P.begin(), e = P.end(); p < e; ++p)
		f >> *p;
	return f;
}

}

//------------------------------------------------------------------------------
interp_set_t ranges_t::operator()(scalar aoa, scalar slp, scalar mch) const
{
//	static constexpr auto _360d = math::pi_*2;
	static constexpr auto _180d = math::pi_;
	assert(std::abs(aoa) <= _180d);
	assert(std::abs(slp) <= _180d);
	//	while(std::abs(aoa) > _180d)
	//		aoa -= math::sgn(aoa)*_360d;
	//	while(std::abs(slp) > _180d)
	//		aoa -= math::sgn(aoa)*_360d;

	static constexpr auto _90d = math::pi_/2;
	interp_set_t s;
    s.sgn_alfa = aoa < 0 ? -1 : 1;
    s.sgn_beta = slp < 0 ? -1 : 1;
	scalar a = std::abs(aoa); if(a > _90d) a = _180d - a;
	scalar b = std::abs(slp); if(b > _90d) b = _180d - b;
	s.alfa = alfa.find(a);
	s.beta = beta.find(b);
	s.mach = mach.find(mch);
    return s;
}
const ranges_t& ranges_t::save(file_t& f) const
{
    f << alfa;
    f << beta;
    f << mach;
	return *this;
}
ranges_t& ranges_t::load(const file_t& f)
{
    f >> alfa;
    f >> beta;
    f >> mach;
	return *this;
}

//------------------------------------------------------------------------------
coefs_t& coefs_t::update(const msdat::ranges_t& r, const msdat::coef_t& cf, simetry s)
{
	assert(r.mach.size() == cf.mach_size());
	assert(r.beta.size() == cf.beta_size());
	assert(r.alfa.size() == cf.alfa_size());

	m_data.resize(r.mach.size());
	m_sym = s;

	mach_t::iterator i_mch = m_data.begin();
	for(unsigned m = 0, me = m_data.size(); m != me; ++m, ++i_mch)
		i_mch->update(r.alfa, r.beta, mach_data_t(cf[mch(m)]));
	return *this;
}

scalar coefs_t::operator()(const interp_set_t& s) const
{
	input_t inp(s.alfa.frc, s.beta.frc);

	const incidence_t& inc = m_data[s.mach.pos];
	const incidence_t::patch_t pch = inc(s.beta.pos, s.alfa.pos);
	scalar cf = pch(inp);
	if(s.mach.frc > 0 && s.mach.pos < m_data.size()-1)
	{
		assert(s.mach.frc < 1 || s.mach.pos == m_data.size()-2);
		const incidence_t& inc = m_data[s.mach.pos+1];
		const incidence_t::patch_t pch = inc(s.beta.pos, s.alfa.pos);
		scalar dc = pch(inp) - cf;
		cf += dc * s.mach.frc;
	}
	return cf * (s.sgn_alfa < 0 ? m_sym.aoa() : 1)
			  * (s.sgn_beta < 0 ? m_sym.slp() : 1);
}
coefs_t::set_t coefs_t::operator()(const ranges_t& rng, const values_t& mach, const values_t& beta, const values_t& alfa) const
{
	set_t cf;
	cf.resize(data::rows(beta.size()), data::cols(alfa.size()), data::blks(mach.size()));

	interp_set_t s;
	set_t::iterator pc = cf.begin();
	for(values_t::const_iterator pm = mach.begin(), me = mach.end(); pm < me; ++pm)
	{
		s.mach = rng.mach.find(*pm);
		for(values_t::const_iterator pb = beta.begin(), be = beta.end(); pb < be; ++pb)
		{
			s.beta = rng.beta.find(std::abs(*pb));
			s.sgn_beta = *pb < 0 ? -1 : 1;
			for(values_t::const_iterator pa = alfa.begin(), ae = alfa.end(); pa < ae; ++pa, ++pc)
			{
				s.alfa = rng.alfa.find(std::abs(*pa));
				s.sgn_alfa = *pa < 0 ? -1 : 1;
				*pc = (*this)(s);
			}
		}
	}
	return cf;
};
const coefs_t& coefs_t::save(file_t& f) const
{
	unsigned cnt = m_data.size(); f.write(cnt);
	for(mach_t::const_iterator i = m_data.begin(), e = m_data.end(); i < e; ++i)
		f << *i;
	bool
	b = m_sym.aoa() > 0; f.write(b);
	b = m_sym.slp() > 0; f.write(b);
	return *this;
}
coefs_t& coefs_t::load(const file_t& f)
{
	unsigned cnt = 0; f.read(cnt);
	m_data.resize(cnt);
	for(mach_t::iterator i = m_data.begin(), e = m_data.end(); i < e; ++i)
		f >> *i;
	bool x; f.read(x);
	bool y; f.read(y);
	m_sym = simetry(x, y);
	return *this;
}

//------------------------------------------------------------------------------
const force_coefs& force_coefs::save(file_t& f) const
{
    cL .save(f);
    cD .save(f);
    cx .save(f);
    cy .save(f);
    cz .save(f);
    cyp.save(f);
    cxq.save(f);
    czq.save(f);
    cyr.save(f);
	return *this;
}
force_coefs& force_coefs::load(const file_t& f)
{
    cL .load(f);
    cD .load(f);
    cx .load(f);
    cy .load(f);
    cz .load(f);
    cyp.load(f);
    cxq.load(f);
    czq.load(f);
    cyr.load(f);
	return *this;
}

//------------------------------------------------------------------------------
const moment_coefs& moment_coefs::save(file_t& f) const
{
    cl .save(f);
    cm .save(f);
    cn .save(f);
    clp.save(f);
    cnp.save(f);
    cmq.save(f);
    clr.save(f);
    cnr.save(f);
	return *this;
}
moment_coefs& moment_coefs::load(const file_t& f)
{
    cl .load(f);
    cm .load(f);
    cn .load(f);
    clp.load(f);
    cnp.load(f);
    cmq.load(f);
    clr.load(f);
    cnr.load(f);
	return *this;
}

//------------------------------------------------------------------------------
void model::setup(const msdat::data_t& md)
{
	sref   = md.sref;
	lref   = md.lref;
	xref   = md.xref;

	R.alfa = md.alfa;
	R.beta = md.beta;
	R.mach = md.mach;

	F.cD .update(md, md.cD , { true , true  });
	F.cx .update(md, md.cx , { true , true  });
	F.cL .update(md, md.cL , { false, true  });
	F.cz .update(md, md.cz , { false, true  });

	F.cy .update(md, md.cy , { true , false });

    F.cyp.update(md, md.cyp, { true , true  });
    F.cxq.update(md, md.cxq, { true , true  });
    F.czq.update(md, md.czq, { true , true  });
    F.cyr.update(md, md.cyr, { true , true  });

	M.cl .update(md, md.cl , { true , false });
	M.cm .update(md, md.cm , { false, true  });
	M.cn .update(md, md.cn , { true , false });
    M.clp.update(md, md.clp, { true , true  });
    M.cnp.update(md, md.cnp, { true , true  });
    M.cmq.update(md, md.cmq, { true , true  });
    M.clr.update(md, md.clr, { true , true  });
    M.cnr.update(md, md.cnr, { true , true  });
}
void model::setup(const roll_finset_t::params_t& p)
{
	roll_fins.build(R.mach, p, sref*lref);
}
const model& model::save(std::string fpath) const
{
	file_t f;
	f.open(fpath, "w");

    f.write(sref);
    f.write(lref);
    f.write(xref);

    R.save(f);
    F.save(f);
    M.save(f);

	return *this;
}
model& model::load(std::string fpath)
{
	file_t f;
	f.open(fpath, "r");

    f.read(sref);
    f.read(lref);
    f.read(xref);

    R.load(f);
    F.load(f);
    M.load(f);

	return *this;
}

//------------------------------------------------------------------------------
force_t model::force(const wind_t& wnd) const
{
	if(wnd.vel > 0)
	{
		interp_set_t s = R(wnd.aoa, wnd.slp, wnd.mch);
		angle_rate_t wb = wnd.wb * lref/(2*wnd.vel);

		double cl = F.cL(s);
		double cd = F.cD(s);
		double CY = F.cy(s);
		double sn = sin(wnd.aoa);
		double cs = cos(wnd.aoa);
		double CX = -cd*cs + cl*sn;
		double CZ = -cl*cs - cd*sn;

		CY += F.cyp(s) * wb.x();
		CX += F.cxq(s) * std::abs(wb.y());
		CZ += F.czq(s) * wb.y();
		CY += F.cyr(s) * wb.z();

		double QS = wnd.Q*sref;

		return force_t(QS*CX, QS*CY, QS*CZ);
	}
	else
		return force_t(0);
}
moment_t model::moment(const wind_t& wnd, scalar da) const
{
	if(wnd.vel > 0)
	{
		interp_set_t s = R(wnd.aoa, wnd.slp, wnd.mch);
		angle_rate_t wb = wnd.wb * lref/(2*wnd.vel);

		double CL = M.cl(s);
		double CM = M.cm(s);
		double CN = M.cn(s);

		CL += M.clp(s) * wb.x();
		CN += M.cnp(s) * wb.x();
		CM += M.cmq(s) * wb.y();
		CL += M.clr(s) * wb.z();
		CN += M.cnr(s) * wb.z();

		if (da && roll_fins)
			CL += roll_fins(s.mach)*da;

		double QSL = wnd.Q*sref*lref;

		return moment_t(QSL*CL, QSL*CM, QSL*CN);
	}
	else
		return moment_t(0);
}
model::result_t model::operator()(const wind_t& wnd, scalar da) const
{
	result_t res;
	if(wnd.vel > 0)
	{
		interp_set_t s = R(wnd.aoa, wnd.slp, wnd.mch);
		angle_rate_t wb = wnd.wb * lref/(2*wnd.vel);

		double cl = F.cL(s);
		double cd = F.cD(s);
		double CY = F.cy(s);
		double sn = sin(wnd.aoa);
		double cs = cos(wnd.aoa);
		double CX =-cd*cs + cl*sn;
		double CZ =-cl*cs - cd*sn;
		double CL = M.cl(s);
		double CM = M.cm(s);
		double CN = M.cn(s);

		CY += F.cyp(s) * wb.x();
		CL += M.clp(s) * wb.x();
		CN += M.cnp(s) * wb.x();

		CX += F.cxq(s) * std::abs(wb.y());
		CZ += F.czq(s) * wb.y();
		CM += M.cmq(s) * wb.y();

		CY += F.cyr(s) * wb.z();
		CL += M.clr(s) * wb.z();
		CN += M.cnr(s) * wb.z();

		if (da && roll_fins)
			CL += roll_fins(s.mach)*da;

		double QS  = wnd.Q*sref;
		double QSL = QS*lref;

		res.force  = force_t (QS*CX, QS*CY, QS*CZ);
		res.moment = moment_t(QSL*CL, QSL*CM, QSL*CN);
	}
	return res;
}

//------------------------------------------------------------------------------
//
inline scalar roll_finset_t::operator()(const range_t::frac_t& m) const
{
	spline_1_t::input_t u(m.frc);
	return Ld(m.pos)(u);
}
void roll_finset_t::build(const range_t& mach, const params_t& s, scalar REF, scalar a0)
{
    float AR = s.b*s.b/s.S;
    float f  = (s.S*s.r*s.N*s.dmax)/REF;

	std::vector<float> ld;
	ld.reserve(mach.size());
	float am = 0;
    for(range_t::const_iterator m = mach.begin(), e = mach.end(); m < e; ++m)
    {
    	float a;
		if(*m > 1)
		{
			// supersónico
			float PG = sqrt(*m * *m - 1);
			float sw = 1/(2*AR*PG); // esto debería ir multiplicado por ep taper ratio l = ct/cr?
			a = 4/PG * (1 - sw);
			if (a > am)
				a = am;
		}
		else
		{
			// subsónico
			float PG  = 1 - *m * *m;
			float apr = a0/(math::pi_*AR);
			a = a0 / (sqrt(PG + apr*apr) + apr);
			if (am < a)
				am = a;
		}
		if(a > 0)
			ld.push_back(a*f);
		else
			ld.push_back(ld.back());
    }
    Ld.update(mach.begin(), mach.end(), ld.begin());
}





