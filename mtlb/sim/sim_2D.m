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
function [sim, log] = sim_2D(fp, tf, param, x)

    w = what;
    k = strfind(w.path, 'mtlb');
    root_path = w.path(1:k+4);
    txt = strcat(root_path,'rocket/sim/2D;', root_path,'aero;', path);
    path(txt);

    % = { r, h, u, w, θ, q, m, tvc }
    if nargin < 4
        x = [0 
             param.ltvc(end)+1 
             0  
             0 
             pi/2 
             0 
             param.mi 
             0]';
    end
    t = 0;
    r2d = 180/pi;
    p_tol = 0.01*pi/180; 
    
    param.cx = squeeze(param.cx(:,:,1));
    param.cz = squeeze(param.cz(:,:,1));    
    param.cm = squeeze(param.cm(:,:,1));
    
    param.czq = squeeze(param.czq(:,:,1));    
    param.cmq = squeeze(param.cmq(:,:,1));    

    param.czad = squeeze(param.czad(:,:,1));    
    param.cmad = squeeze(param.cmad(:,:,1));    
    
    if isempty(fp) || fp(1).start > 10 || fp(1).mode ~= 2 
        % agregar el lift off
        lo = flight_plan(2, 0, 0);
        fp = [lo ; fp];
    end
    log = '';
    for k = 1:length(fp)  
        switch fp(k).mode 
            case 1     % pitch over 
                fp(k).ref = (90-fp(k).ref)*pi/180; 
                odst = odeset('Event', @(t,y) pitch_over(t, y, fp(k).ref, p_tol, param.max_Qa)); 
            case 2     % constant pitch 
                fp(k).ref = (90-fp(k).ref)*pi/180; 
                if k < length(fp) && fp(k+1).start > 0
                    odst = odeset('Event', @(t,y) height(t, y, fp(k+1).start, param.max_Qa)); 
                else
                    odst = odeset('Event', @(t,y) grounded(t, y, param.max_Qa));
                end
            otherwise  % gravity turn (0)
                if k < length(fp) && fp(k+1).start > 0
                    odst = odeset('Event', @(t,y) height(t, y, fp(k+1).start, param.max_Qa)); 
                else
                    odst = odeset('Event', @(t,y) grounded(t, y, param.max_Qa));
                end
        end
        [t_, x_] = ode23t(@(t,y) rocket_xdot(t, y, param, fp(k)) , [t(end) tf], x(end,:)', odst); 
        if flight_state(x_(end,:)', param.max_Qa)
            disp('FTS')
        end
        t = [t ; t_(2:end)]; 
        x = [x ; x_(2:end,:)];
        switch fp(k).mode
            case 1   ; man = 'pitch over';
            case 2   ; man = 'constant pitch';
            otherwise; man = 'gravity turn';
        end
        h = x(end,2);
        v = x(end,3:4);
        y = atan2(-v(2), v(1));
        p = x(end,5);
        log = strcat(log, sprintf('\n%0.1fs : %s complete (%0.0fm, γ=%0.2f, θ=%0.2f⁰)', t(end), man, h, y*r2d, p*r2d));
        if h <= 0 || t(end) >= tf
            break;
        end    
    end
    %%
    %m = x(:,end);
    sim.t = t;
    sim.p = mod(x(:,5), 2*pi);
    sim.h = x(:,2);
    sim.r = x(:,1); 
    
    Z1 = zeros(size(t));
    sim.pn  = [sim.r*cosd(30) sim.r*sind(-30) sim.h];
    sim.wb  = [Z1 x(:,6) Z1];
    sim.m   =  x(:,7);
    sim.tvc = [x(:,8) Z1];    
    
    [sim.Q, sim.M, sim.f, sim.T, sim.Fn, sim.y, sim.v, sim.g] = compute(t, x(:,2), x(:,3:4), x(:,5), param);

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
    cza  = -interp1(param.mach, param.cza(:,1), M, 'pchip','extrap');
    a    = p - y;
    Fn   = Q .* param.sref .* cza .* a;
end

% flight termination function
function [fts, h, y] = flight_state(x, max_Qa)
    h = x(2);
    v = x(3:4);
    y = atan2(-v(2), x(1)); 
    [~, ~, ~, rho] = atmosfera(h);
    v2 = v'*v;
    Q = 0.5*rho*(v2);
    if v2 > 1
        p   = x(5);
        sp  = sin(p);
        cp  = cos(p);
        dcm_bn = [cp -sp ; sp cp];
        vb  = dcm_bn * v;         
        a   = atan2(vb(2), vb(1));
%        a_  = p - y;
        a   = abs(a);
    else
        a = 0;
    end
    Qa = Q*a;
    fts = (h <= 0) || (h > 1e6) || (Qa > max_Qa); % || (a > 0.5);
end

function [v, a, b] = height(~, x, ref, max_Qa)
    [fts, h] = flight_state(x, max_Qa);
    if h >= ref || fts
        v = 1;
        a = 1;
    else
        v = 0;
        a = 0;
    end
    b = 0;
end
function [v, a, b] = pitch_over(~, x, ref, tol, max_Qa)
    [fts, ~, y] = flight_state(x, max_Qa);
    ep = ref - y;
    if abs(ep) < tol || fts
        v = 1;
        a = 1;
    else
        v = 0;
        a = 0;
    end
    b = 0;
end
function [v, a, b] = grounded(~, x, max_Qa)
    fts = flight_state(x, max_Qa);
    if fts
        v = 1;
        a = 1;
    else
        v = 0;
        a = 0;
    end
    b = 0;
end




