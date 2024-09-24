/*============================================================================*/
/*                                               Guiado, Navegación y Control */
/*============================================================================*/

////////////////////////////////////////////////////////////////////////////////
/// \brief
/// \author   Augusto Zumarraga
/// \date     creación: 27/06/2024
/// \date     revisión: 05/09/2024
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

#include "fcc_def.hpp"

namespace gnc {

template <class T, unsigned C>
class t_schedule
{
public:

	static unsigned capacity()
	{
		return C;
	}

	t_schedule() : m_ts(0.02), m_off_tm(0), m_size(0)
	{}
	unsigned size       () const { return m_size;	}
	bool     empty      () const { return m_size == 0; }
	second_t sample_time() const { return m_ts; }

	template<class I>
	void fill(I p, unsigned n, second_t ts)
	{
		assert(ts.value > 0);
		assert(n <= capacity());
		m_ts   = ts;
		m_size = n < capacity() ? n : capacity() ;
		std::copy(p, p + m_size, m_data);
	}
	const T& operator()(second_t t) const
	{
		assert(t >= m_off_tm);
		if( t  > m_off_tm)
			t -= m_off_tm;
		else
			t = 0;
		unsigned k = double(t/m_ts) + 0.5;
		return k < size() ? m_data[k] : back();
	}
	const T& back() const
	{
		assert(m_size > 0);
		return m_data[m_size-1];
	}
	void set_offset(second_t t)
	{
		m_off_tm = t;
	}
	second_t time_offset() const
	{
		return m_off_tm;
	}

private:

	second_t m_ts, m_off_tm;
	unsigned m_size;
	T        m_data[C];
};

}



