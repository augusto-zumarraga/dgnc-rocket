% Cómputo de EFECTOS AERODINAMICOS
%
% h   : altura
% vb  : velocidad lineal relativa del aire en terna b
% wb  : velocidad angular relativa del aire en terna b
% dca : desplazamiento del CG respecto de X_REF (+ si el CG está más a proa) 
% coef: coeficientes aerodinámicos
% fin : efecto por desvío de aletas
%
% Fa : fuerza aerodinámica en terna-b
% Ma : momento aerodinámica en terna-b
% air
%    .a  : ángulo de ataque
%    .b  : ángulo de deslizamiento
%    .M  : número de Mach
%    .po : presión
%    .ro : densidad del aire
%    .Q  : presión dinámica
function [Fa, Ma, air] = aero(h, vb, wb, dca, coef, fin)

    [~, vs, po, rho] = atmosfera(h);
    vb = vb + [0 ; -wb(3) ; wb(2)] * dca;  % vb en x_ref     
 
    V2 = vb'*vb;
    V  = sqrt(V2);
    Q  = 0.5*rho*V2; % presión dinámica
    M  = V/vs;       % número de Mach  

    if V > 0.1
        a = atan2(vb(3), vb(1)); % α
        b = asin(vb(2)/V);       % β
    else
        a = 0;
        b = 0;
    end
    if V > 0
        t_= coef.LREF / (2*V); % [s]
        p = wb(1) * t_;
        q = wb(2) * t_;
        r = wb(3) * t_;
    else
        p = 0;
        q = 0;
        r = 0;
    end
    aa = abs(a);
    bb = abs(b);
    
    if M <= coef.mach(1)
        idx = 1;    
        frc = [];
    elseif M >= coef.mach(end)
        idx = length(coef.mach);    
        frc = [];
    else
        idx = floor(interp1(coef.mach, 1:length(coef.mach), M));
        M1  = coef.mach(idx  );
        M2  = coef.mach(idx+1);
        frc = (M-M1)/(M2-M1);
    end
    CD  = interp_M(coef, coef.cd , aa, bb, idx, frc);
    cy  = interp_M(coef, coef.CY , aa, bb, idx, frc) * sign(b);
    CL  = interp_M(coef, coef.cl , aa, bb, idx, frc) * sign(a);
    cs = cos(a);
    sn = sin(a);
    cx = CL * sn - CD * cs;
    cz =-CD * sn - CL * cs;
    
    cl  = interp_M(coef, coef.CL , aa, bb, idx, frc) * sign(b);
    cm  = interp_M(coef, coef.CM , aa, bb, idx, frc) * sign(a);
    cn  = interp_M(coef, coef.CN , aa, bb, idx, frc) * sign(b);

    cyp = interp_M(coef, coef.CYP, aa, bb, idx, frc) * p;
    clp = interp_M(coef, coef.CLP, aa, bb, idx, frc) * p;
    cnp = interp_M(coef, coef.CNP, aa, bb, idx, frc) * p;
    
    cxq = interp_M(coef, coef.CXQ, aa, bb, idx, frc) * abs(q);
    czq = interp_M(coef, coef.CZQ, aa, bb, idx, frc) * q;
    cmq = interp_M(coef, coef.CMQ, aa, bb, idx, frc) * q;

    cyr = interp_M(coef, coef.CYR, aa, bb, idx, frc) * r;
    clr = interp_M(coef, coef.CLR, aa, bb, idx, frc) * r;
    cnr = interp_M(coef, coef.CNR, aa, bb, idx, frc) * r;
    
%     Todo esto es muy pesado!!!
%     method = 'spline';

%     cx  = interp3(coef.alpha, coef.mach, coef.beta, coef.CX , aa, M, bb, method);
%     cy  = interp3(coef.alpha, coef.mach, coef.beta, coef.CY , aa, M, bb, method) * sign(b);
%     cz  = interp3(coef.alpha, coef.mach, coef.beta, coef.CZ , aa, M, bb, method) * sign(a);
%     
%     cl  = interp3(coef.alpha, coef.mach, coef.beta, coef.CL , aa, M, bb, method) * sign(b);
%     cm  = interp3(coef.alpha, coef.mach, coef.beta, coef.CM , aa, M, bb, method) * sign(a);
%     cn  = interp3(coef.alpha, coef.mach, coef.beta, coef.CN , aa, M, bb, method) * sign(b);
% 
%     cyp = interp3(coef.alpha, coef.mach, coef.beta, coef.CYP, aa, M, bb, method) * p;
%     clp = interp3(coef.alpha, coef.mach, coef.beta, coef.CLP, aa, M, bb, method) * p;
%     cnp = interp3(coef.alpha, coef.mach, coef.beta, coef.CNP, aa, M, bb, method) * p;
%     
%     cxq = interp3(coef.alpha, coef.mach, coef.beta, coef.CXQ, aa, M, bb, method) * abs(q);
%     czq = interp3(coef.alpha, coef.mach, coef.beta, coef.CZQ, aa, M, bb, method) * q;
%     cmq = interp3(coef.alpha, coef.mach, coef.beta, coef.CMQ, aa, M, bb, method) * q;
% 
%     cyr = interp3(coef.alpha, coef.mach, coef.beta, coef.CYR, aa, M, bb, method) * r;
%     clr = interp3(coef.alpha, coef.mach, coef.beta, coef.CLR, aa, M, bb, method) * r;
%     cnr = interp3(coef.alpha, coef.mach, coef.beta, coef.CNR, aa, M, bb, method) * r;

    if nargin > 5 && ~isempty(fin)
        for k=1:length(fin)
            fn = interp1(coef.mach, fin(k).CNA, M, 'pchip') * fin(k).delta;
            cx = cx + fn * fin(k).x_ref; % resistencia
            cl = cl + fn * fin(k).l_ref; % momento de rolido
        end
    end
    QS = Q * coef.SREF;
    Fa = [cx + cxq       ; 
          cy + cyp + cyr ; 
          cz + czq       ] * QS;
    Ma = [cl + clp + clr ; 
          cm + cmq       ; 
          cn + cnp + cnr ] * QS * coef.LREF;
 	Ma(2) = Ma(2) + Fa(3) * dca;
    Ma(3) = Ma(3) - Fa(2) * dca;
    
    air.a  = a;
    air.b  = b;
    air.M  = M;
    air.ro = rho;
    air.po = po;
    air.Q  = Q;
end

%
% Dado que el número de Mach no cambia tan rápido, resolvemos primero 
% esa interpolación (lineal) y después vemos que pasa con alfa y beta
% idx: índice en la tabla al número de mach inmediatamente inferior
% frc: fracción para interpolar linealmente. Si es vacío no se interpola
%
% function c = interp_M(coef, CF, aa, bb, idx, frc)
%     if isempty(frc)
%         CF = squeeze(CF(idx,:,:));
%     else
%         CF = squeeze(CF(idx,:,:)*(1-frc) + CF(idx+1,:,:)*frc);
%     end
%     c  = interp2(coef.beta, coef.alpha, CF, bb, aa, 'spline');
% end
function c = interp_M(coef, CF, aa, bb, idx, frc)
    C  = squeeze(CF(idx,:,:));
    c  = interp2(coef.beta, coef.alpha, C, bb, aa, 'spline');
    if ~isempty(frc)
        C  = squeeze(CF(idx+1,:,:));
        c2 = interp2(coef.beta, coef.alpha, C, bb, aa, 'spline');
        c  = c + (c2-c)*frc;
    end
end

