% x = { r, h, vn[2], θ, ωy, m, tvc } 
%
% param
%   .t    : tiempo de vuelo
%   .mdot : dm/dt para cada t 
%   .ve   : velocidad de salida de la tobera 
%   .pe   : presión de salida de la tobera
%   .Ae   : área de salida de la tobera
%   .sref : superficie de referencia
%   .lref : longitud de referencia
%   .mach : rango de números de Mach
%
%           para cada número de mach:
%
%   .cx   : coeficiente de fuerza axial 
%   .cz   : derivada del coeficiente de fuerza normal con el ángulo de ataque 
%   .czq  : derivadativa de fuerza normal con la velocidad angular
%   .cma  : derivada del coeficiente de momento de cabeceo con el ángulo de 
%           ataque 
%   .cmq  : derivadativa del momento de cabeceo con la velocidad angular
%   .cme  : derivadativa del momento de cabeceo con la deflexión del
%   elevador
%
%   .mass : valores de masa total 
%   .Iy   : momentos de inercia
%   .x_ref: corrimiento de REF respecto del CG (> 0 si REF está más adelante 
%           que el CG).
%
% maneouver: ver función flight_plan()
%
function xd = rocket_xdot(t, x, param, maneouver)

    h  = x(2); % altura 
    if h > 0
        vn  = x(3:4); % velocidad en coordenadas de navegación
        p   = x(5);   % θ
        q   = x(6);   % dθ/dt
        m   = x(7);   % masa 
        tvc = x(8);   % posición normalizada del TVC  
        
        p = mod(p, 2*pi);
        if p > pi
            p = 2*pi - p;
        elseif p < -pi
            p = 2*pi + p;
        end

        % desplazamiento del CG (+ si el CG está más a popa del punto de referencia)
        dca = interp1(param.mass, param.x_ref, m, 'pchip','extrap');
        % brazo de palanca del TVC
        ltv = interp1(param.mass, param.ltvc , m, 'pchip','extrap');
        % Momento de inercia
        Iy  = interp1(param.mass, param.Iy   , m, 'pchip','extrap');  
        
        y   = atan2(-vn(2), vn(1));  % γ    
        sp  = sin(p);
        cp  = cos(p);
        dcm_bn = [cp -sp 
                  sp  cp];   % proyección n → b
        
        %% aerodinámica
        if isfield(param, 'wind')
        	uw = interp1(param.wind.h, param.wind.u, h, 'pchip','extrap');  
        else
            uw = 0;
        end        
        vb = dcm_bn * (vn + [uw ; 0]);     
        [F, M, Ta, v, a, QS, Mch] = aero(h, vb, q, dca, param);

        %% propulsión
        mdot = interp1(param.t, param.mdot, t);
        if mdot == 0
            T  = 0;
            Mt = 0;
        else
            T = mdot*param.ve + Ta;
            f_tvc = T * tvc * param.d_tvc;
            F  = F + [T ; f_tvc];
            Mt = f_tvc * ltv;
        end
        
        %% efectos adicionales
        [g, R] = gravedad(h);
        fb = F/m;
        fg = dcm_bn(:,2) * g;  %dcm_bn * [0 ; gravedad(h)];
        if vb(1) > 0.1
            % da/dt
            ab = fb + fg; 
            ad = 1/v^2 * (vb(1)*ab(2) - vb(2)*ab(1));      
            [Fad, Mad] = aero_ad(Mch, a, ad, param, v, QS);
            fb = fb + Fad/m;
            M  = M  + Mad;
        end
        mp = (M + Mt)/Iy;
        
        %==================================================================
        [r_tvc, ~] = fcc(y, a, p, q, v, T, R, Iy, ltv, param, maneouver);
        d_tvc = (r_tvc - tvc) * param.w_tvc;
       
        %==================================================================
        ab = fb + fg;  
        an = dcm_bn' *  ab;
        xd = [vn(1)
             -vn(2)
              an 
              q
              mp 
             -mdot
              d_tvc];
    else
        xd = zeros(size(x));
    end

end
% dca: desplazamiento del CG respecto de X_REF (+ si el CG está más a proa) 
function [Fa, Ma, Ta, V, a, QS, M] = aero(h, vb, q, dca, param)

    vb(2) = vb(2) + q*dca;  % vb en x_ref         
        
    [~, vs, po, rho] = atmosfera(h);
 
    V2 = vb'*vb;
    V  = sqrt(V2);
    Q  = 0.5*rho*V2;  % presión dinámica
    M  = V/vs;        % número de Mach  

    if V > 0.1
        a = atan2(vb(2), vb(1)); % α
    else
        a = 0;
    end
    if V > 0
        q = q * param.lref / (2*V);
    end
    aa = abs(a);
    cx = interp2(param.alpha, param.mach, param.cx, aa, M, 'spline');
    cz = interp2(param.alpha, param.mach, param.cz, aa, M, 'spline') * sign(a);
    cm = interp2(param.alpha, param.mach, param.cm, aa, M, 'spline') * sign(a);
    
%     ca = interp1(param.mach, param.ca , M,'spline','extrap');
%     cn = interp1(param.mach, param.cna, M,'spline','extrap') * a; 
%     cm = interp1(param.mach, param.cma, M,'spline','extrap') * a;
%    cq = interp1(param.mach, param.cmq, M,'spline','extrap') * q;
 
    czq = interp2(param.alpha, param.mach, param.czq, aa, M, 'spline') * q;
    cmq = interp2(param.alpha, param.mach, param.cmq, aa, M, 'spline') * q;

    %ce = interp1(param.mach, param.cme, M,'spline','extrap');
    QS = Q * param.sref;
    Fa = QS * [cx ; cz+czq];
    Ma = QS * param.lref * (cm + cmq);
    %Me = QS * param.lref * ce;
    %
    Ta = (param.pe - po)*param.Ae;

    Ma = Ma + dca * Fa(2);
end
function [Fa, Ma] = aero_ad(M, a, ad, param, V, QS)
    ad = ad * param.lref / (2*V);
    aa = abs(a);
    czad = interp2(param.alpha, param.mach, param.czad, aa, M, 'spline') * ad;
    cmad = interp2(param.alpha, param.mach, param.cmad, aa, M, 'spline') * ad;
    Fa = QS * [0 ; czad];
    Ma = QS * param.lref * cmad;
end

function [tvc, elv] = fcc(y, a, p, q, v, T, R, Iy, dgm, param, maneouver) 

    elv = 0;    
    if T > 0 
        % -----------------------------------------------------   guiado
        if v > 1
            switch maneouver.mode 
                case 1   % pitch maneouver
                    rp = p + (maneouver.ref - y)*param.ky;
                    ep = rp - p;
                    rq = param.kp*ep;
                case 2   % constant pitch 
                    ep = maneouver.ref - p;
                    rq = param.kp*ep;
                case 3   % prescribed gravity turn
                    rq = (v/R - g/v) * cos(y);                    
                otherwise % gravity turn: α = 0, β = 0
                    ep = -a; % gravity turn 
                    rq = param.kp*ep;
            end
        else
            rq = 0;
        end
        % -----------------------------------------------------   control
        if abs(rq) > param.pitch_rate
            rq = sign(rq) * param.pitch_rate;
        end        
        eq  = q - rq;
        
        kq = param.kq*Iy/dgm;
        
        tvc = -kq*eq;
        if abs(tvc) > 1
            tvc = sign(tvc);
        end
    else
        tvc = 0;
    end
    
end

