/*============================================================================*/
/*                                                                            */
/*============================================================================*/

////////////////////////////////////////////////////////////////////////////////
/// \brief
/// \author   Augusto Zumarraga
/// \date     creación: 10/06/2024
/// \date     revisión: 23/06/2024
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

#include <dgnc/lang/alphanumeric.hpp>

namespace dgnc {  

inline bool is_sep(char c)
{
    return c == ',' || c == ' ' || c == '\t';
}
inline bool is_term(char c)
{
    return c == '=' || c == ' ';
}

inline const char* find_start(const char* txt)
{
    assert(txt);
	while(*txt && !is_sep(*txt))
		++txt;
	while(*txt &&  is_sep(*txt))
		++txt;
	return txt;
}
inline const char* find_char(const char* txt, char c)
{
    assert(txt);
	while(*txt != c && *txt)
		++txt;
	if(*txt)
		return txt + 1;
	return 0;
}
inline const char* find_digit(const char* txt)
{
    assert(txt);
	while(*txt && !is_digit(*txt))
		++txt;
	return *txt ? txt : 0;
}
inline const char* find_number(const char* txt)
{
    assert(txt);
	while(*txt && !is_numeric(*txt) && !is_nan(txt))
		++txt;
	return *txt ? txt : 0;
}
//inline const char* findtag(const char* txt, const char c)
//{
//    assert(txt);
//	for(const char* p = txt; *p; ++p)
//		if(*p == c)
//			return p + 1;
//	return 0;
//}

//-----------------------------------------------------------------------------
inline const char* findtag(const char* txt, const char* tag)
{
    assert(txt);
	int len = ::strlen(tag);
	while(*txt)
	{
		txt = find_start(txt);
		if(*txt && strncmp(txt, tag, len) == 0 && is_term(*(txt+len)))
			return txt + len;
	}
	return 0;
}

inline const char* strfind(const char* txt, const char* tag)
{
    assert(txt);
	int len = ::strlen(tag);
	for(const char* p = txt; *p; ++p)
		if(strncmp(p, tag, len) == 0)
			return p + len;
	return 0;
}
/*
inline const char* strfind(const char* txt, const char c)
{
    assert(txt);
	for(const char* p = txt; *p; ++p)
		if(*p == c)
			return p + 1;
	return 0;
}
inline const char* findtext(const char* txt, const char* tag)
{
    assert(txt);
	for(int len = ::strlen(tag);
	while(*txt)
	{
		if(*txt && strncmp(txt, tag, len) == 0)
			return txt + len;
	}
	return 0;
}
*/

}




