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
class t_interp
{
public:

	static unsigned capacity()
	{
		return C;
	}

	t_interp() : m_size(0)
	{}
	unsigned size () const { return m_size;	}
	bool     empty() const { return m_size == 0; }

	template<class I, class J>
	void fill(I r, J v, unsigned n)
	{
		assert(n <= capacity());
		m_size = n < capacity() ? n : capacity() ;
		std::copy(r, r + m_size, m_range);
		std::copy(v, v + m_size, m_data );
	}
	T operator()(float x) const
	{
		if(x <= front())
			return m_data[0];
		if(x >= back())
			return m_data[m_size-1];

		unsigned k = find(x);
		float f = x-get(k);
		if(f == 0)
			return m_data[k];
		f /= get(k+1)-get(k);
		return m_data[k] + (m_data[k+1] - m_data[k])*f;
	}

	float front() const
	{
		assert(m_size > 0);
		return m_range[0];
	}
	float back() const
	{
		assert(m_size > 0);
		return m_range[m_size-1];
	}
	float get(unsigned k) const
	{
		assert(k < m_size);
		return m_range[k];
	}
	/// \return k : [0:size()-1]
	unsigned find(float v) const
	{
	    if(empty() || v <= front())
	    	return 0;
	    if(v >= back())
	    	return size()-1;
	    unsigned i_left = 0;
	    unsigned i_right = size() - 1;
		while(i_right > i_left + 1)
		{
			unsigned i_cur = i_left + unsigned((i_right - i_left) / 2);
	        if(v == get(i_cur))
	        	return i_cur;
	        if(v < get(i_cur))
	            i_right = i_cur;
	        else
	        if(v > get(i_cur))
	            i_left = i_cur;
		}
	    return i_left;
	}

private:

	unsigned m_size;
	float    m_range[C];
	T        m_data [C];
};

}



