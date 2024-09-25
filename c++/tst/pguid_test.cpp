/*============================================================================*/
/*                                                                            */
/*============================================================================*/

////////////////////////////////////////////////////////////////////////////////
/// \brief
/// \author   Augusto Zumarraga
/// \date     creación: 10/06/2024
/// \date     revisión: 22/06/2024
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
#include <dgnc/utest.hpp>
#include "../fcc/pguid/pguid.hpp"

using namespace dgnc;

typedef utest::handler_t test_handler_t;
using geom::scalar;
using geom::vector;
using geom::direction;

namespace {

// clase para acceder a elementos protegidos de ltg_t
class ltg_dbg_t : public gnc::guid::ltg_t
{
public:

	void reset( vector rbias_I
			  , vector sDI_I
			  , vector rgrav_I
			  , vector vgo_I
			  , vector uZ_I
	          , scalar ttgo)
	{
		m_state.rbias_I = rbias_I;
		m_state.sDI_I   = sDI_I  ;
		m_state.rgrav_I = rgrav_I;
		m_state.vgo_I   = vgo_I  ;
		m_state.uZ_I    = uZ_I   ;
		m_state.ttgo    = ttgo   ;

//		direction u_I        ;
//		direction ulambda_I  ;
//		vector    lambdadot_I;
//		second_t  t0         ;
//		scalar    Vmiss      ;
	}
	void init(const gnc::guid::nav_t& nv)
	{
		gnc::guid::ltg_t::init(nv);
	}
	void update(const gnc::guid::nav_t& nv)
	{
		gnc::guid::ltg_t::update(nv);
	}
};


}

std::ostream& operator<<(std::ostream& s, const gnc::guid::out_t& out)
{
	s << static_cast<const dgnc::geom::vector&>(out.dir) << ", " << out.rate;
	return s;
}


bool pguid_test()
{
	test_handler_t tst = "init";

	ltg_dbg_t ltg;
	gnc::guid::nav_t nav;
	vector u_I;

	ltg.params.Tg         = 0.05;
	ltg.params.Ve         = 3081;
	ltg.params.rD         = 6.570987308000000e+06;
	ltg.params.VD         = 7.788495489197944e+03;
	ltg.params.tgomin     = 4;
	ltg.params.tburn      = 2.991316956697326e+02;
	ltg.params.cycle_time = 15*ltg.params.Tg;
	ltg.params.Fsmin      = 0.430840042151289;

	nav.elapsed = 0;
	nav.pos = vector(6.435799804111536e6, 0.124258486990317e6, -0.000002587391566e6);
	nav.vel = vector(1.183580207069113e3, 2.403206728382260e3, -0.000083494038451e3);
	nav.sfc = vector(3.862190257332670, 7.842004755806380, -0.000272452901733);
	nav.vxp = nav.vel.as_any() ^ nav.pos.as_any();
	nav.Fs  = nav.sfc.norm();
	ltg.init(nav);
	ltg.update(nav);

	tst = "pre-cycling";
	ltg.reset( vector(0,0,0) // _rbias_I
			 , vector(6.569762899592869e6,  0.126844964516609e6,  -0.000002641248894e6) // _sDI_I
			 , vector(-4.809057054748463,  -0.092850332773153,   0.000001933390417) // _rgrav_I
			 , vector(-1.333927694862748e3,   5.383837482638428e3,  -0.000188130803762e3) // _vgo_I
			 , vector(-0.019303790828685,   0.999813663861105,  -0.000034875136358) // _uZ_I
			 , double(0)); // _tgo)

	nav.elapsed = 0;
	nav.pos = vector(6.435858975926606e6,   0.124378656896116e6,  -0.000002591566604e6);
	nav.vel = vector(1.183292401859655e3,   2.403589511420979e3,  -0.000083507466637e3);
	nav.sfc = vector(3.862190257332670,   7.842004755806380,  -0.000272452901733);
	nav.vxp = nav.vel.as_any() ^ nav.pos.as_any();
	nav.Fs  = nav.sfc.norm();
	ltg.update(nav);
	u_I = vector(-0.650487717686334,  0.759516772973556, -0.000026665712670);

	std::cout << "out : " << ltg.out() << std::endl
	          << "u_I : " << u_I       << std::endl;


	tst = "running 1";
	ltg.reset( vector( 3.363039433943588e3,  6.490849549177801e3, -0.000225464916211e3) // _rbias_I
			 , vector( 6.423505017917428e6,  1.384361760711728e6, -0.000046538589909e6) // _sDI_I
			 , vector(-4.007980290392768e5, -0.259145569200262e5,  0.000007950534155e5) // _rgrav_I
			 , vector(-0.106850896169073e3,  5.503386871479639e3, -0.000191967299403e3) // _vgo_I
			 , vector(-0.210677893078169  ,  0.977555535098578  , -0.000034150780672  ) // _uZ_I
			 , double(2.927851268338499e+02)); // _tgo)

	nav.elapsed = 5.899999999999987;
	nav.pos = vector(6.436803766925527e6, 0.126303981221589e6, -0.000002658458619e6);
	nav.vel = vector(1.178684000146458e3, 2.409724617045117e3, -0.000083722687467e3);
	nav.sfc = vector(3.850909667682026  , 7.869696003857086  , -0.000273421733586  );
	nav.vxp = nav.vel.as_any() ^ nav.pos.as_any();
	nav.Fs  = nav.sfc.norm();
	ltg.update(nav);
	u_I = vector(0.248880519692836,   0.968534194430409,  -0.000033711451742);

	std::cout << "out : " << ltg.out() << std::endl
	          << "u_I : " << u_I       << std::endl;

	tst = "running 2";
	nav.elapsed = 5.949999999999987;
	nav.pos = vector(6.436862691832738e6, 0.126424477824148e6,  -0.000002662645118e6);
	nav.vel = vector(1.178312292420992e3, 2.410139493921935e3,  -0.000083737257737e3);
	nav.sfc = vector(2.180844630721272  , 8.486894033331250  ,  -0.000295400534428  );
	nav.vxp = nav.vel.as_any() ^ nav.pos.as_any();
	nav.Fs  = nav.sfc.norm();
	ltg.update(nav);
	u_I = vector(0.248802815937770, 0.968554158343727, -0.000033712169096);

	std::cout << "out : " << ltg.out() << std::endl
	          << "u_I : " << u_I       << std::endl;


    return true;
}


