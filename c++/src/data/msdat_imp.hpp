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

#include <dgnc/rocket/data/msdat.hpp>
#include <algorithm>

namespace dgnc { namespace rocket { namespace msdat {

inline values_t operator*(const values_t& lhs, scalar rhs)
{
	values_t x = lhs;
	for(values_t::iterator i = x.begin(), e = x.end(); i != e; ++i)
		*i *= rhs;
	return x;
}

class table2_t : public values_t
{
private:

	unsigned m_rows;
	unsigned m_cols;

public:

	class column_iterator : public values_t::const_iterator
	{
		unsigned m_cols;
	public:
		typedef values_t::const_iterator base_t;

		column_iterator(base_t it, unsigned cols)
		: base_t(it), m_cols(cols)
		{}
		column_iterator& operator--() { base_t::operator-=(m_cols); return *this; }
		column_iterator& operator++() { base_t::operator+=(m_cols); return *this; }
	};

	table2_t(unsigned n, unsigned m)
	: m_rows(n), m_cols(m)
	{
		values_t::resize(m_rows*m_cols);
	}
	table2_t()
	: m_rows(0), m_cols(0)
	{}
	scalar* row(unsigned n)
	{
		assert(n < m_rows);
		return &values_t::operator[](n * m_cols);
	}
	column_iterator col(unsigned n) const
	{
		assert(n < m_cols);
		return column_iterator(begin()+n, m_cols);
	}
	table2_t& resize(unsigned n, unsigned m)
	{
		m_rows = n;
		m_cols = m;
		values_t::resize(m_rows*m_cols);
		return *this;
	}
	table2_t& reserve(unsigned n, unsigned m)
	{
		m_rows = n;
		m_cols = m;
		values_t::reserve(m_rows*m_cols);
		return *this;
	}
};

class collection_t
{
public:

	typedef values_t alfa_t;
	typedef std::map<unsigned, alfa_t> beta_t;
	typedef std::map<unsigned, beta_t> mach_t;

	template <class IT>
	collection_t& set(unsigned m, unsigned b, unsigned n_alfa, IT src, scalar f = 1, scalar s = 0)
	{
		beta_t& bta = m_mach[m];
		alfa_t& aoa = bta[b];
		aoa.resize(n_alfa);
		for(alfa_t::iterator dst = aoa.begin(), end = aoa.end(); dst < end; ++dst, ++src)
			*dst = *src * f + s;
		return *this;
	}
	const values_t& get(unsigned m, unsigned b) const
	{
		mach_t::const_iterator i_mch = m_mach.find(m);
		if(i_mch == m_mach.end())
			throw std::logic_error("Mach index not found");
		beta_t::const_iterator i_bta = i_mch->second.find(b);
		if(i_bta == i_mch->second.end())
			throw std::logic_error("beta index not found");
		return i_bta->second;
	}
	const mach_t& mach() const { return m_mach; }

private:

	mach_t m_mach;
};


}}}




