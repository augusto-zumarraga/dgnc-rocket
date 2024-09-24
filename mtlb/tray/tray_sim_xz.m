% Simulador de trayectoria
%
% Argumentos:
%    mi  : masa inicial
%    fp[]: plan de vuelo. Se trata de un array de estructuras que representan
%          las sucesivas fase del vuelo. Este arreglo se construye con la 
%          función flight_plan()
%    tf  : tiempo máximo para la simulación
%   param: parámetros del vehículo. Sus campos necesarios son los requeridos
%          por la función rocket_xdot()
%
% Las salidas son arrays con valores en los cuales cada fila corresponde 
% a un instante del vuelo.
%
%  t[]: tiempo de vuelo en cada instante
%  x[]: estados { h, r, v, θ, m } en cada instante 
%  Q[]: presión dinámica
%  M[]: número de Mach
%  f[]: fuerza específica
%  T[]: empuje del motor
%  g[]: aceleración gravitatoria
%
%  log: contiene un registro de texto de los resultados de cada fase
%
function [out, log, tf, xf] = tray_sim_xz(fp, tf, param, to, xo)

    if nargin < 4 || isempty(to)
        % = { r, h, u, w, θ, m }
        x = [ 0  3  0  0 pi/2  param.mi];
        t = 0;
        if isempty(fp) || fp(1).start > 10 || fp(1).mode ~= 2 
            % agregar el lift off
            lo = flight_plan(2, 0, 0);
            fp = [lo ; fp];
        end
    else
        t = to;
        x = xo;
        if iscolumn(x)
            x = x';
        end
    end    
    r2d = 180/pi;
    p_tol = 0.01*pi/180; 
    
    param.cx = squeeze(param.cx(:,:,1));
    param.cz = squeeze(param.cz(:,:,1));
    
    log = '';
    for k = 1:length(fp)  
        switch fp(k).mode 
            case 1     % pitch over 
                fp(k).ref = (90-fp(k).ref)*pi/180; 
                odst = odeset('Event', @(t,y) pitch_maneuver(t, y, fp(k).ref, p_tol)); 
            case 2     % constant pitch 
                fp(k).ref = (90-fp(k).ref)*pi/180; 
                if k < length(fp) && fp(k+1).start > 0
                    odst = odeset('Event', @(t,y) height(t, y, fp(k+1).start)); 
                else
                    odst = odeset('Event', @(t,y) grounded(t, y));
                end
            case {4, 5, 6} % steering
                if k < length(fp) && fp(k+1).start > 0
                    odst = odeset('Event', @(t,y) in_orbit(t, y, param.vt, fp(k+1).start));
                else
                    odst = odeset('Event', @(t,y) in_orbit(t, y, param.vt));
                end
            otherwise  % gravity turn (0)
                if k < length(fp) && fp(k+1).start > 0
                    odst = odeset('Event', @(t,y) height(t, y, fp(k+1).start)); 
                else
                    odst = odeset('Event', @(t,y) grounded(t, y));
                end
        end
        [t_, x_] = ode23t(@(t,y) rocket_xdot_xz(t, y, param, fp(k)) , [t(end) tf], x(end,:)', odst); 
        t = [t ; t_(2:end)]; 
        x = [x ; x_(2:end,:)];
        switch fp(k).mode
            case 1   ; man = 'pitch maneuver';
            case 2   ; man = 'constant pitch';
            case 3   ; man = 'prescribed gravity turn'; 
            case 4   ; man = 'proportional steering';
            case 5   ; man = 'linear steering';
            case 6   ; man = 'insertion';
            otherwise; man = 'gravity turn';
        end
        h = x(end,2);
        y = atan2(x(end,4), x(end,3));
        p = x(end,5);
        log = strcat(log, sprintf('\n%0.1fs : %s complete (%0.0fm, γ=%0.2f, θ=%0.2f⁰)', t(end), man, h, y*r2d, p*r2d));
        
        xf = x(end,:)';
        if in_orbit(t(end), xf, param.vt)
            vt  = x(end,3);
            log = strcat(log, sprintf('\n*************\nIN ORBIT (%0.0fm, γ=%0.2f, vt=%0.2fm/s)', h, y*r2d, vt));
            break;
        elseif h <= 0 
            log = strcat(log, sprintf('\n*************\n%s', 'GROUNDED'));
            break;
        elseif t(end) >= tf
            break;
        end    
    end
    %%
    %m = x(:,end);
    out.t = t;
    out.r = x(:,1);
    out.h = x(:,2);
    out.p = x(:,5);
    out.m = x(:,6);
    [out.Q, out.M, out.f, out.T, out.Fn, out.y, out.v, out.g] = compute(t, out.h, x(:,3:4), out.p, param);

    tf = t(end);         
    
end

% Q: presión dinámica
% M: número de mach;
% f: fuerza específica
% T: empuje del motor
function [Q, M, f, T, Fn, y, v, g] = compute(t, h, v, p, param)
    y = atan2(-v(:,2), v(:,1));  
    v2= sum(v.^2,2);
    v = sqrt(v2);
    g = gravedad(h);
    a = [0 ; (v(2:end)-v(1:end-1))./(t(2:end)-t(1:end-1))]; 
    a(1) = a(2);
    f = a + g .* sin(p); 

    [~, a, po, rho] = atmosfera(h);
    M = v./a;
    Q = 0.5*rho.*v2;
    
    mdot = interp1(param.t, param.mdot, t);
    T    = (param.ve*mdot + param.Ae*(param.pe - po)) .* (mdot > 0);
    a    = p - y;
    %cx   = interp2(param.alpha, param.mach, param.cx, a, M, 'spline');
    cz   = interp2(param.alpha, param.mach, param.cz, a, M, 'spline');    
    %cna  = interp1(param.mach, param.cna, M,'spline','extrap');
    Fn   = Q .* param.sref .* cz .* a;
end

% flight termination function
function [v, a, b] = height(~, x, h)
    if x(2) >= h || x(2) <= 0
        v = 1;
        a = 1;
    else
        v = 0;
        a = 0;
    end
    b = 0;
end
function [v, a, b] = pitch_maneuver(~, x, ref, tol)
%     y  = atan2(-x(4), x(3));  
%     ep = ref - y;
    ep = ref - x(5);
    if abs(ep) < tol || x(2) <= 0
        v = 1;
        a = 1;
    else
        v = 0;
        a = 0;
    end
    b = 0;
end
function [v, a, b] = grounded(~, x)
    if x(2) <= 0
        v = 1;
        a = 1;
    else
        v = 0;
        a = 0;
    end
    b = 0;
end
function [v, a, b] = in_orbit(~, x, vt, hf)
    if nargin > 3 && x(2) > hf
        ac = 1;
        g  = ac;
    else
        [g, R] = gravedad(x(2));
        ac = (x(3)+vt)^2/R;
    end
    if ac >= g
        v = 1;
        a = 1;
    else
        v = 0;
        a = 0;
    end
    b = 0;
end




