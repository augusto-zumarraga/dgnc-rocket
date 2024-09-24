% x = { pe[3], ve[3], qeb[4], m }
%
%    pe : posición
%    ve : velocidad
%    qbe: cuaternion de actitud
%    m  : masa
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
function xd = rocket_xdot_ecef(t, x, param, maneouver)

    pe = x(1:3);
    [nav.lat, nav.lng, nav.h] = WGS84.ecef_to_lgv(pe);
    if nav.h > 0
        qeb = quaternion(x(7:10));
        qbe = qeb';
        ve  = x(4:6); 
        vb  = qbe.trnsf(ve); 
        m   = x(11);       
        
        %%
        [F, Ta, v, a, b] = aero(nav.h, vb, param);
        mdot = interp1(param.t, param.mdot, t);
        if mdot > 0
            T = mdot*param.ve + Ta;
            F(1) = F(1) + T;
        end
        fb = F/m; 
         
        %==================================================================
        % ωb: velocidad angular en terna móvil 
        wb = fcc(a, b, qeb, nav, v, mdot > 0, param, maneouver);  
        wb = wb + qbe.trnsf([0 0 WGS84.We]);
        xd = [ecef(pe, ve, qeb, qbe, fb, wb) ; -mdot];
    else
        xd = zeros(size(x));
    end

end

%% 
% h: altura
function [Fa, Ta, V, a, b, Q] = aero(h, vb, param)

    [~, a, po, rho] = atmosfera(h);

    V2 = vb'*vb;
    V  = sqrt(V2);
    Q  = 0.5*rho*V2; % presión dinámica
    M  = V/a;        % número de Mach  

    if V > 0.1
        a = atan2(vb(3), vb(1)); % α
        b = asin(vb(2)/V);       % β
    else
        a = 0;
        b = 0;
    end
    
    aa = abs(a);
    bb = abs(b);
    cx = interp3(param.alpha, param.mach, param.beta, param.cx, aa, M, bb, 'spline');
    cy = interp3(param.alpha, param.mach, param.beta, param.cy, aa, M, bb, 'spline') * sign(b);
    cz = interp3(param.alpha, param.mach, param.beta, param.cz, aa, M, bb, 'spline') * sign(a);
    QS = Q * param.sref;
    Fa = QS * [cx ; cy ; cz];
    %
    Ta = (param.pe - po)*param.Ae;    
end

%% -----------------------------------------------------   guiado
function [w] = fcc(a, b, qeb, nav, v, prop, param, maneouver)

    if prop
        if v > 1
            switch maneouver.mode 
                case 1 % pitch maneouver
                    [ep, ey] = att_err(qeb, nav, maneouver.ref);
                    q = -ep * param.kp;
                    r = -ey * param.kp;
                case 2 % constant pitch 
                    [ep, ey] = att_err(qeb, nav, maneouver.ref);
                    q = -ep * param.kp;
                    r = -ey * param.kp;
                otherwise % α = 0, β = 0
                    q = - a * param.ka;
                    r =   b * param.ka;
            end
            
            % -----------------------------------------------------   control
            if abs(q) > param.pitch_rate
                q = sign(q) * param.pitch_rate;
            end
            if abs(r) > param.pitch_rate
                r = sign(r) * param.pitch_rate;
            end
        else
            q = 0;
            r = 0;
        end
    elseif v < 0.01 % || y == pi/2
        q = -1;
        r =  0;
    else
        q = -param.ka * a; % sign(a)*sqrt(abs(a))
        r =  param.ka * b; % sign(b)*sqrt(abs(b)) 
    end
    w = [0;q;r];
end

%% x = { pe, ve, qbe }
%  u = { fb, wib }
%  f = dx/dt
function f = ecef(pe, ve, qeb, qbe, uf, uw)

    fe = qeb.trnsf(uf);
    
    ge = WGS84.gravity(pe); 
    ae = fe + ge;
    cr = WGS84.coriolis(ve);

    f = zeros(10,1);
    f(1:3) = ve;
    f(4:6) = ae + cr;
    
    wb = uw - qbe.trnsf([0 0 WGS84.We]);
    
    qdot = qeb.angular_velocity_transform() * wb;
    f(7:10) = qdot(1:4)';
   
end


