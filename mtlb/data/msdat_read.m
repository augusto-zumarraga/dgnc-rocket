% Extracción de datos de archivos "for006.dat"
% Por ahora solo se considera el caso con simetría XY/XZ, aunque
% se incluye una dimensión en las tablas para el deslizamiento por 
% compatibilidad a futuro.
%
% Argumentos:
%
%   fpath: ruta del atchivo
%   fname: nombre del archivo ("for006.dat") 
%
% Resultados:
%
%   Salvo indicación en contrario se utiliza el sistema métrico, ángulos 
%   en radianes y las "derivativas dinámicas" en rad/seg.
%   El sistema de coordenadas está centrado en el CG con el eje X hacia
%   adelante, el eje Z hacia abajo y el Y a estribor.
%   Las distancias se toman desde el origen geométrico usando en el 
%   "for005.dat" (en general, la proa)
%
%   data: estructura de datos extraídos
%
%     .XREF  posición del punto de referencia   
%     .SREF  superficie de referencia 
%     .LREF  longitud de referencia
%
%     .alpha rango de ángulos de ataque
%     .beta  rango de ángulos de deslizamiento 
%     .mach  rango de números de Mach 
%
%     .CX    fuerzas axiales
%     .CY    fuerzas laterales
%     .CZ    fuerzas normales 
%     .CL    momento de rolido
%     .CM    momento de cabeceo
%     .CN    momento de guiñada
% 
%     .CL    sustentacion
%     .CD    resistencia
%     .XCP   posición del centro de presión (desde la proa) 
% 
%     .CZA   Derivada del coeficiente de fuerzas normales con alpha
%     .CYB   Derivada del coeficiente de fuerzas laterales con beta
%     .CLB   Derivada del coeficiente de momentos de rolido con beta
%     .CMA   Derivada del coeficiente de momentos con alpha
%     .CNB   Derivada del coeficiente de momentos de guiñada con beta
% 
%   (las derivativas "dinámicas" están adimensionalizados con 
%    q_ref = 2Vo/Lref)
%
%     .CLP   momento de rolido debido a la velocida de rolido
%     .CNP   momento de guiñada debido a la velocida de rolido
%
%     .CXQ   fuerzas axiales debido a la velocida de cabeceo
%     .CZQ   fuerzas normales debido a la velocida de cabeceo
%     .CMQ   momento de cabeceo debido a la velocida de cabeceo
%
%     .CYR   fuerzas laterales debido a la velocida de guiñada
%     .CYP   fuerzas laterales debido a la velocida de rolido
%
%     .CLR   momento de rolido debido a la velocida de guiñada
%     .CNR   momento de guiñada debido a la velocida de guiñada
%
%     .CZAD  fuerzas axiales debido la velocidad de alpha
%     .CMAD  momento de cabeceo debido a la velocida de alpha
% 
%
%     .FIN() hasta 10
%         .SSPAN 
%         .CHORD 
%         .SWEEP (en grados)
%         .XLE 
%         .NPANEL 
%         .PHIF  (en grados) 
%
% log_text: registro de datos encontrados
%
function [data, log_text] = msdat_read(fpath, fname)

    global msread_log;
    msread_log = "";

    if nargin < 2 || isempty(fname)
        fname = "for006";
    end
    inp = text_file(fname, fpath, '.dat');
    if inp.is_bad()
        error('No se pudo abrir el archivo');
    end
    
    d2r = pi/180;
    r2d = 1/d2r;
    i2m = 0.0254;

    DIM   = find_control(inp, 'DIM'); 
    DERIV = find_control(inp, 'DERIV'); 

    switch DIM 
        case 'FT'
            f_lng = 12*i2m;
        case 'IN'
            f_lng = i2m;
        case 'CM'
            f_lng = 0.01;
        otherwise %'M'
            f_lng = 1;
    end
    if DERIV == 'RAD'
        f_drv = 1;
    else
        f_drv = d2r; % acá decía r2d!!
    end
    %%
    FLTCON = find_card(inp, "$FLTCON");
    [mach, alpha, beta] = parse_FLTCON(FLTCON);
    n_M = length(mach );
    n_a = length(alpha);
    n_b = length(beta );

    REFQ = find_card(inp, "$REFQ");
    data.XREF = parse_val(REFQ, "XCG" ) * f_lng; 
    data.XCG  = data.XREF;               % por compatibilidad 
    data.LREF = parse_val(REFQ, "LREF") * f_lng; 
    data.SREF = parse_val(REFQ, "SREF") * f_lng^2; 
    
    AXIBOD = find_card(inp, '$AXIBOD');
    if ~isempty(AXIBOD)
        data.AXIBOD.TNOSE  = parse_text(AXIBOD, "TNOSE");
        data.AXIBOD.LNOSE  = parse_val(AXIBOD, "LNOSE" ) * f_lng; 
        data.AXIBOD.DNOSE  = parse_val(AXIBOD, "DNOSE" ) * f_lng; 
        data.AXIBOD.LCENTR = parse_val(AXIBOD, "LCENTR") * f_lng; 
        data.AXIBOD.DCENTR = parse_val(AXIBOD, "DCENTR") * f_lng; 
    end
    
    %%    
    for k=1:10
        FIN = find_card(inp, sprintf('$FINSET%d', k));
        if isempty(FIN)
            break;
        end
        data.FIN(k).SSPAN  = parse_val(FIN, "SSPAN", 2) * f_lng; 
        data.FIN(k).CHORD  = parse_val(FIN, "CHORD", 2) * f_lng; 
        data.FIN(k).SWEEP  = parse_val(FIN, "SWEEP" ); 
        data.FIN(k).STA    = parse_val(FIN, "STA"   ); 
        data.FIN(k).XLE    = parse_val(FIN, "XLE"   ) * f_lng; 
        data.FIN(k).NPANEL = parse_val(FIN, "NPANEL"); 
        data.FIN(k).PHIF   = parse_val(FIN, "PHIF", data.FIN(k).NPANEL); 
    end
    
    data.alpha = alpha;
    data.beta  = beta;
    data.mach  = mach;

    %%
    Z = zeros(n_M,n_a,n_b);
    data.CX  = Z;
    data.CY  = Z;
    data.CZ  = Z;
    data.CL  = Z;
    data.CM  = Z;
    data.CN  = Z;

    data.cl  = Z;
    data.cd  = Z;
    data.XCP = Z;

    data.CZA = Z;
    data.CZAD= Z;
    data.CMA = Z;
    data.CMAD= Z;
    
    data.CYB = Z;
    data.CLB = Z;
    data.CNB = Z;

    data.CYP = Z;
    data.CLP = Z;
    data.CNP = Z;    

    data.CXQ = Z;
    data.CZQ = Z;
    data.CMQ = Z;
    
    data.CYR = Z;
    data.CLR = Z;
    data.CNR = Z;
    

    %% Continuacion de lectura de archivo
    inp.rewind();
    %%
    while ~inp.eof()
        FCOND = search_flight_conditions(inp); 
%        M = interp1(mach, 1:length(mach), FCOND.MACH);
        M = find(data.mach == FCOND.MACH);
        assert(~isempty(M));
        b = find(data.beta == FCOND.BETA);
        if isempty(b)
            data.beta = [data.beta FCOND.BETA];
            b = length(data.beta);
        end
        CASE = pop_coeficients(inp, n_a, f_drv == 1);
        if ~isempty(CASE)
            log(sprintf("CASE %s", CASE.name));
                                     % 1     2    3    4    5    6    7    8    9   10   11
            if isfield(CASE, 'COEF') % ALFA  CN   CM   CA   CY   CLN  CLL  CL   CD  L/D  XCP.
                data.CX (M,:,b) = -CASE.COEF(:,4);         
                data.CY (M,:,b) =  CASE.COEF(:,5);         
                data.CZ (M,:,b) = -CASE.COEF(:,2);
                
                data.CL (M,:,b) =  CASE.COEF(:,7);        
                data.CM (M,:,b) =  CASE.COEF(:,3);         
                data.CN (M,:,b) =  CASE.COEF(:,6);
                
                data.cl (M,:,b) =  CASE.COEF(:,8);
                data.cd (M,:,b) =  CASE.COEF(:,9);   
                %     .XCP   Center of pressure position, measures\d from the moment 
                %            reference center, divided by reference length. 
                %            Positive values indicate c.p. forward of the moment reference 
                %            point.
                data.XCP(M,:,b) = data.XCG - CASE.COEF(:,11) * data.LREF;  
            end
            if isfield(CASE, 'DERIV')
                data.CZA (M,:,b) = -CASE.DERIV(:,2) * f_drv;
                data.CMA (M,:,b) =  CASE.DERIV(:,3) * f_drv;         

                data.CYB (M,:,b) =  CASE.DERIV(:,4) * f_drv;         
                data.CLB (M,:,b) =  CASE.DERIV(:,6) * f_drv;         
                data.CNB (M,:,b) =  CASE.DERIV(:,5) * f_drv;        
            end
            if isfield(CASE, 'DYN_N')
                data.CXQ (M,:,b) = -CASE.DYN_N(:,4) * f_drv;         
                data.CZQ (M,:,b) = -CASE.DYN_N(:,2) * f_drv;
                data.CMQ (M,:,b) =  CASE.DYN_N(:,3) * f_drv;         
                
                data.CZAD(M,:,b) = -CASE.DYN_N(:,5) * f_drv;         
                data.CMAD(M,:,b) =  CASE.DYN_N(:,6) * f_drv;        
            end
            if isfield(CASE, 'DYN_L')
                
                data.CYR (M,:,b) =  CASE.DYN_L(:,2) * f_drv;
                data.CLR (M,:,b) =  CASE.DYN_L(:,4) * f_drv;         
                data.CNR (M,:,b) =  CASE.DYN_L(:,3) * f_drv;         
                
                data.CYP (M,:,b) =  CASE.DYN_L(:,5) * f_drv;         
                data.CLP (M,:,b) =  CASE.DYN_L(:,7) * f_drv;
                data.CNP (M,:,b) =  CASE.DYN_L(:,6) * f_drv;        
            end
        end
    end
    inp.close();
    data.alpha = data.alpha * d2r;
    data.beta  = data.beta  * d2r;
    log_text   = msread_log;
    clear msread_log;

end

%% 
function log(txt)
    global msread_log;
    msread_log = msread_log + txt + newline;
end
%%
function txt = find_val(txt, tag)  
    h = strfind(txt, tag); 
    if h ~= 0
        h = h + strlength(tag);
        txt = txt(h:end);
        h = strfind(txt, '='); 
        assert(~isempty(h));
        h = h + 1; %strlength(tag) + 1;
        txt = txt(h:end);
    else
        txt = [];
    end
end
function [txt,b] = parse_text(txt, name) 
    txt = find_val(txt,name);
    assert(~isempty(txt));
    b = strfind(txt,","); 
    assert(~isempty(b));
    txt = txt(1:b-1);
end
function [val,b] = parse_val(txt, name, count) 
    if nargin < 3
        count = 1;
    end
    txt = find_val(txt,name);
    assert(~isempty(txt));
            %    b = strfind(txt,","); 
            %    assert(~isempty(b));
            %    val = sscanf(txt(1:b),'%f');
    val = zeros(1,count);
    b = 1;
    for k=1:count
        txt = txt(b:end);
        [v,~,~,b] = sscanf(txt,'%f');
        if isempty(v)
            break;
        end
        b = b + 1;
        val(k) = v;
    end
end
function A = parse_array(txt, count) 
    A = zeros(1,count);
    for k=1:count
        [v,~,~,nxt] = sscanf(txt,'%f');
        txt(1:nxt) = [];
        A(k) = v;
    end
end
function c = scan_row(txt, len) 
    c = [];
    while length(c) < len          
        c = [c ; sscanf(txt,'%f')]; 
        if length(c) < len 
            c(end+1) = NaN;
            log('NaN coef');
            k = strfind(txt, '*');
            txt(1:k(end)) = [];
        end
    end
end
function [mach, alpha, beta] = parse_FLTCON(txt) 
    [N,k] = parse_val(txt, "NALPHA"); 
    alpha = find_val(txt(k:end), ",ALPHA"); 
    alpha = parse_array(alpha,N);
    
    [N,k] = parse_val(txt, "NMACH"); 
    mach = find_val(txt(k:end), ",MACH"); 
    mach = parse_array(mach,N);

    beta  = 0;
end

%%-------------------------------------------------------------------------
function CARD = find_card(inp, nombre) 
    CARD = [];
    inp.rewind();
    while ~inp.eof()
        txt = inp.pop_line('*');
        h = strfind(txt, nombre); 
        if h ~= 0
            txt(1:h) = [];
            e = [];
            log(sprintf("CARD %s at line %d", nombre, inp.line));
            while isempty(e)
                CARD = strcat(CARD, txt);
                e = strfind(txt,"$");
                txt = inp.pop_line(4);
            end
            break;
        end
    end
end
function CTRL = find_control(inp, nombre) 
    CTRL = [];
    len  = length(nombre);
    inp.rewind();
    while ~inp.eof()
        txt = inp.pop_line('*');
        h = strfind(txt, nombre); 
        if h ~= 0  
            if txt(h-1) == ' ' && txt(h+len) == ' '
                h = h + len + 1;
                CTRL = sscanf(txt(h:end),'%s');
                log(sprintf("CONTROL CARD %s at line %d", nombre, inp.line));
                break;
            end
        end
    end
end
function FCOND = search_flight_conditions(inp) 
    FCOND = [];
    while ~inp.eof()
        txt = inp.pop_line();
        h = strfind(txt, "******* FLIGHT CONDITIONS AND REFERENCE QUANTITIES *******"); 
        if h ~= 0
            nline = inp.line;
            txt = inp.pop_line(4);
            [FCOND.MACH,k  ] = parse_val(txt, "MACH NO"); 
            [FCOND.REYNOLDS] = parse_val(txt(k:end), "REYNOLDS NO"); 
            
            txt = inp.pop_line(4);
            [FCOND.ALT,k   ] = parse_val(txt, "ALTITUDE"); 
            [FCOND.Q       ] = parse_val(txt(k:end), "DYNAMIC PRESSURE"); 

            txt = inp.pop_line(4);
            [FCOND.BETA, k ]  = parse_val(txt, "SIDESLIP");
            [FCOND.ROLL    ]  = parse_val(txt(k:end), "ROLL");

            inp.skip_line();
            %[FCOND., k  ] = parse_val(tline, "REF AREA =");
            %[FCOND. ] = parse_val(tline(k:end), "MOMENT CENTER =");

            inp.skip_line();
            %[FCOND.,k  ] = parse_val(tline, "REF LENGTH =");
            %[FCOND. ] = parse_val(tline(k:end), "LAT REF LENGTH =");
            log(sprintf("FCOND DATA at line %d: MACH = %f, BETA = %f, ROLL = %f", nline, FCOND.MACH, FCOND.BETA, FCOND.ROLL));
            break;
        end
    end
end
% CASE.            1     2    3    4    5    6    7    8    9   10   11
%   COEF [nalpha]: ALPHA CN   CM   CA   CY   CLN  CLL  CL   CD  L/D  XCP.
%   DERIV[nalpha]: ALPHA CNA  CMA  CYB  CLNB CLLB 
%   DYN_N[nalpha]: ALPHA CNQ  CMQ  CAQ  CNAD CMAD
%   DYN_L[nalpha]: ALPHA CYR  CLNR CLLR CYP  CLNP CLLP
function CASE = pop_coeficients(inp, nalpha, deriv_rad) 
    CASE = [];
    name = [];
    while ~inp.eof()
        nline = inp.line;
        txt = inp.pop_line();
        h = strfind(txt, "******* FLIGHT CONDITIONS AND REFERENCE QUANTITIES *******"); 
        if h ~= 0
            inp.goto_line(nline);
            break;
        end
        h = strfind(txt, "STATIC AERODYNAMICS FOR ");
        if h ~= 0
            name = txt(h+24:end);
            continue;
        end
        h = strfind(txt, "----- LONGITUDINAL -----     -- LATERAL DIRECTIONAL --");
        if h ~= 0
            log(sprintf("COEFS at line %d", inp.line));
            % "ALPHA       CN        CM        CA        CY       CLN       CLL"); 
            inp.skip_line();
            % 
            inp.skip_line();
            COEF = zeros(nalpha, 11);
            for a=1:nalpha
                txt = inp.pop_line();
%                 c = sscanf(txt,'%f'); 
%                 assert(length(c) == 7);
                c = scan_row(txt, 7);
                COEF(a,1:7) = c';
            end
            % "ALPHA       CL        CD      CL/CD     X-C.P."); 
            inp.skip_line(3);
            for a=1:nalpha
                txt = inp.pop_line();
                c = scan_row(txt, 5);
                COEF(a,8:11) = c(2:5)';
            end            
            CASE.COEF = COEF;
            continue;
        end
        if deriv_rad
            h = strfind(txt, "---------- DERIVATIVES (PER RADIAN) ----------");
        else
            h = strfind(txt, "---------- DERIVATIVES (PER DEGREE) ----------");
        end
        if h ~= 0
            log(sprintf("DERIVS at line %d", inp.line));
            %"ALPHA       CNA         CMA         CYB         CLNB        CLLB"); 
            inp.skip_line();
            DERIV = zeros(nalpha, 6);
            for a=1:nalpha
                txt = inp.pop_line();
%                 c = sscanf(txt,'%f'); 
%                 assert(length(c) == 6);
                c = scan_row(txt, 6);
                DERIV(a,:) = c';
            end
            CASE.DERIV = DERIV;
            continue;
        end
        if deriv_rad
            h = strfind(txt, "------------ DYNAMIC DERIVATIVES (PER RADIAN) -----------");
        else
            h = strfind(txt, "------------ DYNAMIC DERIVATIVES (PER DEGREE) -----------");
        end
        if h ~= 0
            txt = inp.pop_line();
            h = strfind(txt, "ALPHA       CNQ        CMQ        CAQ       CNAD       CMAD");
            if h ~= 0
                log(sprintf("DYNAMIC N DATA at line %d", inp.line));
                DYN_N = zeros(nalpha, 6);
                for a=1:nalpha
                    txt = inp.pop_line();
%                     c = sscanf(txt,'%f'); 
%                     assert(length(c) == 6);
                    c = scan_row(txt, 6);
                    DYN_N(a,:) = c';
                end
                CASE.DYN_N = DYN_N;
                continue;
            else
                h = strfind(txt, "ALPHA       CYR       CLNR       CLLR        CYP       CLNP       CLLP");
                if h ~= 0
                    log(sprintf("DYNAMIC L DATA at line %d", inp.line));
                    DYN_L = zeros(nalpha, 7);
                    for a=1:nalpha
                        txt = inp.pop_line();
%                         c = sscanf(txt,'%f'); 
%                         assert(length(c) == 7);
                        c = scan_row(txt, 7);
                        DYN_L(a,:) = c';
                    end
                    CASE.DYN_L = DYN_L;
                    continue;
                end
            end
        end
    end
    if ~isempty(CASE)
        CASE.name = name;
    end
end



