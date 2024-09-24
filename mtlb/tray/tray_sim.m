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
function [sim, log, tf, xf] = tray_sim(fp, tf, param, to, xo)

    % = { r, h, v, θ, m }
    
    p_tol = 0.01*pi/180; 
    param.cx = squeeze(param.cx(:,:,1));
    
    if nargin < 4 || isempty(to)
        x = [ 0  3  0  pi/2  param.mi];
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
    log = '';
    for k = 1:length(fp)  
        if fp(k).mode == 1 || fp(k).mode == 2 % pitch over / constant pitch
            fp(k).ref = (90-fp(k).ref)*pi/180; 
        end
        if fp(k).mode == 1 % pitch over 
            odst = odeset('Event', @(t,y) pitch(t, y, fp(k).ref, p_tol)); 
        elseif k < length(fp) && fp(k+1).start > 0
            odst = odeset('Event', @(t,y) height(t, y, fp(k+1).start)); 
        else
            odst = odeset('Event', @(t,y) grounded(t, y));
        end
        [t_, x_] = ode45(@(t,y) rocket_xdot(t, y, param, fp(k)) , [t(end) tf], x(end,:)', odst); 
        t = [t ; t_(2:end)]; 
        x = [x ; x_(2:end,:)];
        switch fp(k).mode
            case 1   ; man = 'pitch over';
            case 2   ; man = 'constant pitch';
            case 3   ; man = 'prescribed gravity turn'; 
            case 4   ; man = 'steering';
            otherwise; man = 'gravity turn';
        end
        log = strcat(log, sprintf('\n%0.1fs : %s complete (%0.0fm, %0.2f⁰)', t(end), man, x(end,1), x(end,4)*180/pi));
        if t(end) >= tf || x(end,1) <= 0
            break;
        end
    end
    %%
    sim.t = t;
    sim.r = x(:,1);    
    sim.h = x(:,2);
    sim.v = x(:,3);
    sim.y = x(:,4);
    sim.p = sim.y;
    sim.m = x(:,5);
    [sim.Q, sim.M, sim.f, sim.T, sim.g] = compute(t, sim.h, sim.v, sim.y, param);
    
    xf = x(end,:)';
    tf = t(end); 

end

% Q: presión dinámica
% M: número de mach;
% f: fuerza específica
% T: empuje del motor
function [Q, M, f, T, g] = compute(t, h, v, p, param)
    g = gravedad(h);
    a = [0 ; (v(2:end)-v(1:end-1))./(t(2:end)-t(1:end-1))]; 
    a(1) = a(2);
    f = a + g .* sin(p); 

    [~, a, p, rho] = atmosfera(h);
    M = v./a;
    Q = 0.5*rho.*v.^2;
    
    mdot = interp1(param.t, param.mdot, t);
    T = (param.ve*mdot + param.Ae*(param.pe - p)) .* (mdot > 0);
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
function [v, a, b] = pitch(~, x, ref, tol)
    if abs(ref-x(4)) < tol || x(2) <= 0
        v = 1;
        a = 1;
    else
        v = 0;
        a = 0;
    end
    b = 0;
end
function [v, a, b] = grounded(~, x)
    if x(1) <= 0
        v = 1;
        a = 1;
    else
        v = 0;
        a = 0;
    end
    b = 0;
end


