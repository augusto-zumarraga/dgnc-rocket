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
#include "msdat_parse.hpp"

/*
 Extracción de datos de archivos "for006.dat"
 Por ahora solo se considera el caso con simetría XY/XZ, aunque
 se incluye una dimensión en las tablas para el deslizamiento por
 compatibilidad a futuro.

 Argumentos:

   fpath: ruta del atchivo
   fname: nombre del archivo ("for006.dat")

 Resultados:

   Salvo indicación en contrario se utiliza el sistema métrico, ángulos
   en radianes y las "derivativas dinámicas" en rad/seg.
   El sistema de coordenadas está centrado en el CG con el eje X hacia
   adelante, el eje Z hacia abajo y el Y a estribor.
   Las distancias se toman desde el origen geométrico usando en el
   "for005.dat" (en general, la proa)

   data: estructura de datos extraídos

     .XREF  posición del punto de referencia
     .SREF  superficie de referencia
     .LREF  longitud de referencia

     .alpha rango de ángulos de ataque
     .beta  rango de ángulos de deslizamiento
     .mach  rango de números de Mach

     .CX    fuerzas axiales
     .CY    fuerzas laterales
     .CZ    fuerzas normales
     .CL    momento de rolido
     .CM    momento de cabeceo
     .CN    momento de guiñada

     .CL    sustentacion
     .CD    resistencia
     .XCP   posición del centro de presión (desde la proa)

     .CZA   Derivada del coeficiente de fuerzas normales con alpha
     .CYB   Derivada del coeficiente de fuerzas laterales con beta
     .CLB   Derivada del coeficiente de momentos de rolido con beta
     .CMA   Derivada del coeficiente de momentos con alpha
     .CNB   Derivada del coeficiente de momentos de guiñada con beta

   (las derivativas "dinámicas" están adimensionalizados con
    q_ref = 2Vo/Lref)

     .CLP   momento de rolido debido a la velocida de rolido
     .CNP   momento de guiñada debido a la velocida de rolido

     .CXQ   fuerzas axiales debido a la velocida de cabeceo
     .CZQ   fuerzas normales debido a la velocida de cabeceo
     .CMQ   momento de cabeceo debido a la velocida de cabeceo

     .CYR   fuerzas laterales debido a la velocida de guiñada
     .CYP   fuerzas laterales debido a la velocida de rolido

     .CLR   momento de rolido debido a la velocida de guiñada
     .CNR   momento de guiñada debido a la velocida de guiñada

     .CZAD  fuerzas axiales debido la velocidad de alpha
     .CMAD  momento de cabeceo debido a la velocida de alpha


     .FIN() hasta 10
         .SSPAN
         .CHORD
         .SWEEP (en grados)
         .XLE
         .NPANEL
         .PHIF  (en grados)

 log_text: registro de datos encontrados
*/

using namespace dgnc;
using namespace rocket;
using namespace msdat;

namespace {

void operator<<(coef_t& dst, const collection_t& src)
{
	dst.clear();
	unsigned m = 0;
	for(collection_t::mach_t::const_iterator i_mch = src.mach().begin(), m_end = src.mach().end(); i_mch != m_end; ++i_mch, ++m)
	{
		unsigned b = 0;
		for(collection_t::beta_t::const_iterator i_bta = i_mch->second.begin(), b_end = i_mch->second.end(); i_bta != b_end; ++i_bta, ++b)
		{
			if(dst.empty())
				dst.resize(slps(i_mch->second.size()), aoas(i_bta->second.size()), mchs(src.mach().size()));
			else
			{
				assert(dst.beta_size() == i_mch->second.size());
				assert(dst.alfa_size() == i_bta->second.size());
			}
			dst.set(mch(m), slp(b), i_bta->second.begin());
		}
	}
}
}

//------------------------------------------------------------------------------
template <typename I>
void coef_t::set(mch m, slp b, I s)
{

	for(row_iterator d = blocks_t::begin(m, b), e = blocks_t::end(m, b); d < e; ++d, ++s)
		*d = *s;
}

//------------------------------------------------------------------------------
data_t& data_t::import(std::string fpath, std::string fname)
{
    if(fname.empty())
        fname = "for006";
    text_file inp(fname, fpath, ".dat");
    if(inp.is_bad())
        throw runtime_error("No se pudo abrir el archivo");
    const
    text_file::iterator end  = inp.end();
    text_file::iterator line = inp.begin();

    constexpr auto pi = 3.141592653589793238462643383279502884L;
    scalar d2r = pi/180;
    //scalar r2d = 1/d2r;
    scalar i2m = 0.0254;

    scalar f_lng = 1;
    string DIM   = find_control(line, end, "DIM");
    //string DIM   = find_control(inp, "DIM");
    if(DIM == "FT")
    	f_lng = 12*i2m;
    else if(DIM == "IN")
    	f_lng = i2m;
    else if(DIM == "CM")
    	f_lng = 0.01;
    else //"M"
    	f_lng = 1;

    scalar f_drv = 1;
    line = inp.begin();
    string DERIV = find_control(line, end, "DERIV");
    if(DERIV == "RAD")
        f_drv = 1;
    else
        f_drv = d2r; // acá decía r2d!!

    line = inp.begin();
    string REFQ = find_card(line, end, "$REFQ");
    xref = parse_val(REFQ, "XCG" ) * f_lng;
    lref = parse_val(REFQ, "LREF") * f_lng;
    sref = parse_val(REFQ, "SREF") * f_lng*f_lng;

    line = inp.begin();
    string bdy = find_card(line, end, "$AXIBOD");
    if(!bdy.empty())
    {
    	axibod.TNOSE  = parse_text(bdy, "TNOSE" );
    	axibod.LNOSE  = parse_val (bdy, "LNOSE" ) * f_lng;
    	axibod.DNOSE  = parse_val (bdy, "DNOSE" ) * f_lng;
    	axibod.LCENTR = parse_val (bdy, "LCENTR") * f_lng;
    	axibod.DCENTR = parse_val (bdy, "DCENTR") * f_lng;
    }

    line = inp.begin();
    for(unsigned k=1; k<10; ++k)
    {
    	char buff[24];
    	sprintf(buff, "$FINSET%d", k);
        string FIN = find_card(line, end, buff);
        if(FIN.empty())
            break;
        finset_t fs;
        fs.SSPAN  = parse_val(FIN, "SSPAN", 2) * f_lng;
        fs.CHORD  = parse_val(FIN, "CHORD", 2) * f_lng;
        fs.SWEEP  = parse_val(FIN, "SWEEP" );
        fs.STA    = parse_val(FIN, "STA"   );
        fs.XLE    = parse_val(FIN, "XLE"   ) * f_lng;
        unsigned NPANEL = parse_val(FIN, "NPANEL");
        fs.PHIF   = parse_val(FIN, "PHIF", NPANEL);

        fins.push_back(fs);
    }
    line = inp.begin();
    string FLTCON = find_card(line, end, "$FLTCON");
    FLTCON_t  flt(FLTCON.c_str());

    mach = flt.mach;
    alfa = flt.alfa;
    beta.clear();

    collection_t CX   ; ///< fuerzas axiales
    collection_t CY   ; ///< fuerzas laterales
    collection_t CZ   ; ///< fuerzas normales
    collection_t CL   ; ///< momento de rolido
    collection_t CM   ; ///< momento de cabeceo
    collection_t CN   ; ///< momento de guiñada

    collection_t Cl   ; ///< sustentacion
    collection_t Cd   ; ///< resistencia
    collection_t XCP  ; ///< posición del centro de presión (desde la proa)

    collection_t CZA  ; ///< Derivada del coeficiente de fuerzas normales con alpha
    collection_t CYB  ; ///< Derivada del coeficiente de fuerzas laterales con beta
    collection_t CLB  ; ///< Derivada del coeficiente de momentos de rolido con beta
    collection_t CMA  ; ///< Derivada del coeficiente de momentos con alpha
    collection_t CNB  ; ///< Derivada del coeficiente de momentos de guiñada con beta

    collection_t CLP  ; ///< momento de rolido debido a la velocida de rolido
    collection_t CNP  ; ///< momento de guiñada debido a la velocida de rolido

    collection_t CXQ  ; ///< fuerzas axiales debido a la velocida de cabeceo
    collection_t CZQ  ; ///< fuerzas normales debido a la velocida de cabeceo
    collection_t CMQ  ; ///< momento de cabeceo debido a la velocida de cabeceo

    collection_t CYR  ; ///< fuerzas laterales debido a la velocida de guiñada
    collection_t CYP  ; ///< fuerzas laterales debido a la velocida de rolido

    collection_t CLR  ; ///< momento de rolido debido a la velocida de guiñada
    collection_t CNR  ; ///< momento de guiñada debido a la velocida de guiñada

    collection_t CZAD ; ///< fuerzas axiales debido la velocidad de alpha
    collection_t CMAD ; ///< momento de cabeceo debido a la velocida de alpha

    // Continuacion de lectura de archivo
    fcond_t FCOND;
    line  = inp.begin();
	while(line < end)
	{
        FCOND.search(line, end);
        if(!FCOND)
        	break;
        unsigned M = mach.find_index(FCOND.MACH);
        assert(M < mach.size());
        unsigned b = beta.find_index(FCOND.BETA);
        if(b == beta.not_found)
            b = beta.append(FCOND.BETA);

        unsigned n_alfa = alfa.size();
        case_t CASE(line, end, n_alfa, f_drv == 1);
        if(!CASE.COEF.empty())     //       0    1    2    3    4    5    6   7    8    9
        {                          // ALFA  CN   CM   CA   CY   CLN  CLL  CL  CD  L/D  XCP.
			CX.set(M,b, n_alfa, CASE.COEF.col(2), -1);
			CY.set(M,b, n_alfa, CASE.COEF.col(3));
			CZ.set(M,b, n_alfa, CASE.COEF.col(0), -1);

			CL.set(M,b, n_alfa, CASE.COEF.col(5));
			CM.set(M,b, n_alfa, CASE.COEF.col(1));
			CN.set(M,b, n_alfa, CASE.COEF.col(4));

			Cl.set(M,b, n_alfa, CASE.COEF.col(6));
			Cd.set(M,b, n_alfa, CASE.COEF.col(7));
			//    .XCP   Center of pressure position, measures\d from the moment
			//           reference center, divided by reference length.
			//           Positive values indicate c.p. forward of the moment reference
			//           point.
			XCP.set(M,b, n_alfa, CASE.COEF.col(9), -lref, xref);
        }
        if(!CASE.DERIV.empty())
        {
			CZA.set(M,b, n_alfa, CASE.DERIV.col(0), -f_drv);
			CMA.set(M,b, n_alfa, CASE.DERIV.col(1),  f_drv);

			CYB.set(M,b, n_alfa, CASE.DERIV.col(2),  f_drv);
			CLB.set(M,b, n_alfa, CASE.DERIV.col(4),  f_drv);
			CNB.set(M,b, n_alfa, CASE.DERIV.col(3),  f_drv);
        }
        if(!CASE.DYN_N.empty())
        {
			CXQ .set(M,b, n_alfa, CASE.DYN_N.col(2), -f_drv);
			CZQ .set(M,b, n_alfa, CASE.DYN_N.col(0), -f_drv);
			CMQ .set(M,b, n_alfa, CASE.DYN_N.col(1),  f_drv);

			CZAD.set(M,b, n_alfa, CASE.DYN_N.col(3), -f_drv);
			CMAD.set(M,b, n_alfa, CASE.DYN_N.col(4),  f_drv);
        }
		if(!CASE.DYN_L.empty())
		{
			CYR.set(M,b, n_alfa, CASE.DYN_L.col(0), f_drv);
			CLR.set(M,b, n_alfa, CASE.DYN_L.col(2), f_drv);
			CNR.set(M,b, n_alfa, CASE.DYN_L.col(1), f_drv);

			CYP.set(M,b, n_alfa, CASE.DYN_L.col(3), f_drv);
			CLP.set(M,b, n_alfa, CASE.DYN_L.col(5), f_drv);
			CNP.set(M,b, n_alfa, CASE.DYN_L.col(4), f_drv);
		}
    }

    cx   << CX  ;
    cy   << CY  ;
    cz   << CZ  ;
    cl   << CL  ;
    cm   << CM  ;
    cn   << CN  ;
    cL   << Cl  ;
    cD   << Cd  ;
    xcp  << XCP ;
    cza  << CZA ;
    cyb  << CYB ;
    clb  << CLB ;
    cma  << CMA ;
    cnb  << CNB ;
    clp  << CLP ;
    cnp  << CNP ;
    cxq  << CXQ ;
    czq  << CZQ ;
    cmq  << CMQ ;
    cyr  << CYR ;
    cyp  << CYP ;
    clr  << CLR ;
    cnr  << CNR ;
    czad << CZAD;
    cmad << CMAD;

    alfa *= d2r;
    beta *= d2r;

    return *this;
}

