#include <dgnc/rocket/data/air.hpp>
#include <dgnc/numeric/interp.hpp>
#include <cmath>

namespace dgnc { namespace isa {

constexpr auto go = 9.80665;    // Acceleration of gravity (m/s/s)
constexpr auto Ra = 287;        // Gas constant for air (J/kg-K)
constexpr auto gamma = 1.4;    // Ratio of specific heats

struct s_air
{
	double T;    // K
	double P;    // Pa
	double rho;  // kg/m³
	double a;    // m/s
};

struct lapse
{
	double lower, upper; //
	double Lapse; // Lapse Rates (K/m)
	const double eg;

	lapse(double h1, double h2, double L)
	: lower(h1), upper(h2), Lapse(L)
	, eg(-go/(Lapse*Ra))
	{}
	s_air process(double h, s_air s) const
	{
		s_air r;
		assert(h >= lower);
		double z = (h > upper ? upper : h) - lower;
		if(Lapse)
		{
			r.T = s.T + Lapse * z;
			r.P = s.P*std::pow(r.T/s.T, eg);
		}
		else
		{
			constexpr auto _ei = -go/Ra;
			r.T = s.T;
			r.P = s.P*exp(_ei * z/r.T);
		}
		return r;
	}
};

lapse lapses[7] =
{ lapse(    0, 11000, -0.0065)
, lapse(11000, 20000,  0     )
, lapse(20000, 32000,  0.001 )
, lapse(32000, 47000,  0.0028)
, lapse(47000, 51000,  0     )
, lapse(51000, 71000, -0.0028)
, lapse(71000, 84852, -0.002 )
};

//------------------------------------------------------------------------------
// Original matlab Richard Rieber
// rrieber@gmail.com
// Updated 3/17/2006
//
// Function to determine temperature, pressure, density, and speed of sound
// as a function of height.  Function based on US Standard Atmosphere of
// 1976.  All calculations performed in metric units, but converted to
// english if the user chooses.
// Assuming constant gravitational acceleration.
s_air mdl_1976(float hgt)
{
	s_air s = { 288.16, 1.01325e5, 1.225, 340.27 };
	if(hgt > 0)
	{
		if(hgt <= lapses[6].upper)
		{
			for(unsigned k=0; k<7; ++k)
			{
				s = lapses[k].process(hgt, s);
				if(hgt < lapses[k].upper)
					break;
			}
			s.rho = s.P/(Ra*s.T);
		}
		else
		{
			s.T = 186.96;
			s.P = 0.37272 * exp(-1.24426778e-4 * (hgt - lapses[6].upper));

			static numeric::interp::spline_1 hp; // harris_priester;
			if(hp.range().empty())
			{
				static float h [51] = { 84.852
									  , 100, 120, 130, 140, 150, 160, 170, 180, 190, 200, 210, 220, 230, 240, 250
									  , 260, 270, 280, 290, 300, 320, 340, 360, 380, 400, 420, 440, 460, 480, 500
									  , 520, 540, 560, 580, 600, 620, 640, 660, 680, 700, 720, 740, 760, 780, 800
									  , 840, 880, 920, 960, 1000 };
	//			static float rm[50] = { 4.974e+05, 24900,  8377,  3899,  2122,  1263, 800.8, 528.3, 361.7, 255.7, 183.9, 134.1, 99.49, 74.88, 57.09
	//								  , 44.03,  34.3, 26.97, 21.39, 17.08, 10.99, 7.214, 4.824, 3.274, 2.249, 1.558, 1.091, 0.7701, 0.5474, 0.3916
	//								  , 0.2819, 0.2042, 0.1488, 0.1092, 0.0807, 0.06012, 0.04519, 0.0343, 0.02632, 0.02043, 0.01607, 0.01281, 0.01036, 0.008496, 0.007069
	//								  , 0.00468, 0.0032, 0.00221, 0.00156, 0.00115};
	//			static float rM[50] = { 4.974e+05, 2.449e+05,  8710,  4059,  2215,  1344, 875.8, 601, 429.7, 316.2, 239.6, 185.3, 145.5, 115.7, 93.08
	//								  , 75.55, 61.82, 50.95, 42.26, 35.26, 25.11, 18.19, 13.37, 9.955, 7.492, 5.684, 4.355, 3.362, 2.612, 2.042
	//								  , 1.605, 1.267, 1.005, 0.7997, 0.639, 0.5123, 0.4121, 0.3325, 0.2691, 0.2185, 0.1779, 0.1452, 0.119, 0.09776, 0.08059
	//								  , 0.05741, 0.0421, 0.0313, 0.0236, 0.0181 };
			   // hp.rho = [6.9465e-06 ; atm.rm + (atm.rM - atm.rm)*0.7];
				static float r [51] = { 6.9465e+06
									  , 4.974e+05, 1.789e+05, 8610.1, 4011, 2187.1, 1319.7, 853.3, 579.19, 409.3, 298.05, 222.89, 169.94, 131.7, 103.45, 82.283
									  , 66.094, 53.564, 43.756, 35.999, 29.806, 20.874, 14.897, 10.806, 7.9507, 5.9191, 4.4462, 3.3758, 2.5844, 1.9926, 1.5469
									  , 1.2081, 0.94816, 0.74814, 0.59255, 0.47151, 0.37665, 0.30203, 0.24304, 0.19627, 0.15908, 0.12935, 0.10548, 0.086408, 0.070981, 0.058534
									  , 0.041591, 0.03043, 0.022573, 0.016988, 0.013015 };
				hp.setup(h, h+51, r);
			}
			s.rho = hp(hgt*1e-3)*1e-12;
		}
		s.a = sqrt(Ra*gamma*s.T);
	}
	return s;
}

}}

using namespace dgnc;
using namespace rocket;

//------------------------------------------------------------------------------
 air_t::air_t(scalar h)
 {
	 isa::s_air s = isa::mdl_1976(h);
	 T   = s.T;
	 po  = s.P;
	 rho = s.rho;
	 snd = s.a;

//    // ISA
//    T = 288.15 - (h < 11000 ? 6.5e-3*h : 71.5);
//    // https://en.wikipedia.org/wiki/Speed_of_sound
//    snd = 331.3 * sqrt(T/273.15);
//
//    // https://en.wikipedia.org/wiki/Barometric_formula
//    po  = 1013.25e-3 * exp(-1.24426778e-4 * h);
//    rho = 1.225      * exp(-1.1856e-4 * h);

};



//------------------------------------------------------------------------------
wind_t::wind_t(air_t a, velocity_t v, angle_rate_t w, double x_ref)
: wb(w)
{
	v.y() += w.z()*x_ref;
	v.z() -= w.y()*x_ref;
	geom::incidence inc(v, 0.1);

	vel = inc.mod;
	aoa = inc.aoa;
	slp = inc.slp;
	Q   = 0.5*a.rho*inc.md2;
	mch = vel/a.snd;
}
wind_t::wind_t(air_t a, velocity_t v, angle_rate_t w, position_t p_ref)
: wb(w)
{
	v += w ^ p_ref;
	geom::incidence inc(v);

	vel = inc.mod;
	aoa = inc.aoa;
	slp = inc.slp;
	Q   = 0.5*a.rho*inc.md2;
	mch = vel/a.snd;
}





