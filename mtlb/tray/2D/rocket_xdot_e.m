% x = { r, h, vn[2], θ, m } 
%
% param
%   .t    : tiempo de vuelo
%   .mdot : dm/dt para cada t 
%   .ve   : velocidad de salida de la tobera 
%   .pe   : presión de salida de la tobera
%   .Ae   : área de salida de la tobera
%
%   .sref : superficie de referencia
%   .mach : rango de números de Mach
%   .alpha: rango de ángulos de ataque
%
%   para cada número de mach y ángulo de ataque:
%
%   .cx   : coeficiente de fuerza en xb  
%   .cz   : coeficiente de fuerza en zb 
%
% maneouver: ver función flight_plan()
%
function xd = rocket_xdot_xze(t, x, param, maneouver)

    h  = x(2);
    if h > 0
        vn  = x(3:4); 
        p   = x(5);
        m   = x(6);    
        
        sp  = sin(p);
        cp  = cos(p);
        Cbn = [cp -sp ; sp cp];
        vb  = Cbn * vn;
        
        %%
        [F, Ta, v, a] = aero(h, vb, param);
        if t < param.t(end)
            mdot = interp1(param.t, param.mdot, t);
            T = mdot*param.ve + Ta;
            F(1) = F(1) + T;
        else
            mdot = 0;
            T = 0;
        end
        [g, R] = gravedad(h);
        ac = (vn(1)+param.vt)^2/R;
        fb = F/m + Cbn(:,2) * (g - ac); 
                    
        %==================================================================
        % q = ωb: velocidad angular en terna móvil 
        q = fcc(t, h, a, p, v, T, R, g, param, maneouver);

        fn = Cbn' * fb;
        vt =  vn(1)-param.we*vn(2);
        vn = -vn(2)+param.we*vn(1);
        xd = [vt *(6375.4e3/R)
              vn
              fn 
              q - param.we
             -mdot];
    else
        xd = zeros(size(x));
    end

end

%%
function [Fa, Ta, V, a, QS, M] = aero(h, vb, param)

    [~, s, po, rho] = atmosfera(h);
                    
    V2 = vb'*vb;
    V  = sqrt(V2);
    Q  = 0.5*rho*V2; % presión dinámica
    M  = V/s;        % número de Mach  

    if V > 0.1
        a = atan2(vb(2), vb(1)); % α [rad] 
    else
        a = 0;
    end
    aa = abs(a);
    cx = interp2(param.alpha, param.mach, param.cx, aa, M, 'spline');
    cz = interp2(param.alpha, param.mach, param.cz, aa, M, 'spline') * sign(a);
    QS = Q * param.sref;
    Fa = QS * [cx ; cz];
    %
    Ta = (param.pe - po)*param.Ae;
end
        
%% -----------------------------------------------------   guiado
% h: altura
% a: ángulo de ataque
% p: ángulo de cabeceo
% v: velocidad
% T: empuje
% R: radio 
% g: aceleración gravitatoria
%
% q = ωb: velocidad angular en terna móvil requerida
function [q] = fcc(t, h, a, p, v, T, R, g, param, maneouver)
    y = p - a; % γ   
    if T > 0 
        if v > 1
            switch maneouver.mode 
                case 1 % pitch maneouver
                    ep = maneouver.ref - p;
                    q = ep * param.kp;
                case {2,6} % constant pitch 
                    ep = maneouver.ref - p;
                    q = ep * param.kp;
                case 3 % prescribed gravity turn
                    q = (v/R - g/v) * cos(y); % 
                case 4 % proportional steering
                    yr = (maneouver.ref - h)/maneouver.ref;
                    y  = p - a;
                    ey = yr - y;
                    fn = ey * param.kn;
                    rp = y + atan(fn/T);
                    if rp < param.p_min
                       rp = param.p_min;
                    end
                    ep = rp - p;
                    q  = ep * param.kp;
                case 5 % linear tangent steering
                    rp = atan(((maneouver.ref(1) - t) * g + maneouver.ref(3))/maneouver.ref(2));
                    ep = rp - p;
                    q  = ep * param.kp;
                otherwise % α = 0
                    q = - a * param.ka;
            end
            
            % -----------------------------------------------------   control
            if abs(q) > param.pitch_rate
                q = sign(q) * param.pitch_rate;
            end
        else
            q = 0;
        end
    % coasting
    elseif v < 0.01 || y == pi/2
        q = -1;
    else
        q = -a * param.ka;
    end
end


