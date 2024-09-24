% Simulador de trayectoria
%
% Argumentos:
%     
%    fp[]: plan de vuelo. Se trata de un array de estructuras que representan
%          las sucesivas fase del vuelo. Este arreglo se construye con la 
%          función flight_plan()
%    tf  : tiempo máximo para la simulación
%   param: parámetros del vehículo. Sus campos necesarios son los requeridos
%          por la función rocket_xdot_3d()
%     x[]: estados  { pe[3], ve[3], qeb[4], wb(3), m, tvc(2)} en cada instante 
%    aero: coeficientes aerodinámicos
%
% Las salidas son arrays con valores en los cuales cada fila corresponde 
% a un instante del vuelo.
%
%
%  log: contiene un registro de texto de los resultados de cada fase
%
function [sim, log] = sim_3D(fp, tf, x, param, aero)

    w = what;
    k = strfind(w.path, 'mtlb');
    root_path = w.path(1:k+4);
    txt = strcat(root_path,'rocket/sim/3D;', root_path,'aero;', path);
    path(txt);

    r2d = 180/pi;
    p_tol = 0.01*pi/180;     
    ts  = 0.01;
    po  = x(1:3)';
    t   = 0;
    log = '';
    for k = 1:length(fp)  
        switch fp(k).mode 
            case 1     % pitch over 
                odst = odeset('MaxStep', ts, 'Event', @(t,y) pitch_maneuver(t, y, fp(k).ref, p_tol, param.max_Qa)); 
            case 2     % constant pitch 
                if k < length(fp) && fp(k+1).start > 0
                    odst = odeset('MaxStep', ts,'Event', @(t,y) height(t, y, fp(k+1).start, param.max_Qa)); 
                else
                    odst = odeset('MaxStep', ts,'Event', @(t,y) grounded(t, y, param.max_Qa));
                end
            otherwise  % gravity turn (0)
                if k < length(fp) && fp(k+1).start > 0
                    odst = odeset('MaxStep', ts,'Event', @(t,y) height(t, y, fp(k+1).start, param.max_Qa)); 
                else
                    odst = odeset('MaxStep', ts,'Event', @(t,y) grounded(t, y, param.max_Qa));
                end
        end
        [t_, x_] = ode23t(@(t,y) rocket_xdot(t, y, param, aero, fp(k)) , [t(end) tf], x(end,:)', odst); 
        t = [t ; t_(2:end)]; 
        x = [x ; x_(2:end,:)];

        %%
        [fts, h, p] = flight_state(x_(end,:)', param.max_Qa);
        if fts
            disp('FTS')
        end
        switch fp(k).mode
            case 1   ; man = 'pitch maneuver';
            case 2   ; man = 'constant pitch';
            otherwise; man = 'gravity turn';
        end
        log = strcat(log, sprintf('\n%0.1fs : %s complete (%0.0fm, θ=%0.2f⁰)', t(end), man, h, p*r2d));
        if h <= 0 || t(end) >= tf
            break;
        end    
    end
    %%
    sim.t   = t;
    sim.pe  = x(:,1:3);
    sim.ve  = x(:,4:6);
    sim.qeb = x(:,7:10);
    sim.wb  = x(:,11:13);
    sim.m   = x(:,14);
    sim.tvc = x(:,15:16);
    
    Z  = zeros(size(t));
    sim.r = Z;
    sim.h = Z;
    sim.p = Z;
    sim.y = Z;
    sim.a = Z;
    sim.b = Z;
    sim.v = Z;

    Z3 = zeros(length(t),3);
    sim.eul= Z3;
    for k=1:length(t)
        pe = sim.pe(k,:)';
        ve = sim.ve(k,:)';
        qeb= quaternion(sim.qeb(k,:));
        [sim.h(k), sim.r(k), e, sim.y(k), sim.a(k), sim.b(k), sim.v(k), pn] = ecef_to_flight(pe, ve, qeb, po);
        sim.eul(k,:) = [e.roll e.pitch e.yaw];
        sim.pn (k,:) = pn';
        sim.f(k)     = 0; % TODO 
    end
    [sim.Q, sim.M, sim.T, sim.Fn, sim.g] = compute(t, sim.h, sim.a, sim.b, sim.v, param);

end

%%
% Q: presión dinámica
% M: número de mach;
% f: fuerza específica
% T: empuje del motor
function [Q, M, T, Fn, g] = compute(t, h, a, b, v, param)
    [~, vs, po, rho] = atmosfera(h);
    M    = v./vs;
    Q    = 0.5*rho.*v.*v;
    mdot = interp1(param.t, param.mdot, t);
    T    = (param.ve*mdot + param.Ae*(param.pe - po)) .* (mdot > 0);
    cz   = interp3(param.alpha, param.mach, param.beta, param.cz, a, M, b, 'pchip');    
    Fn   = Q .* param.sref .* cz;
    g    = gravedad(h);
end

% flight termination function
function [fts, h, p] = flight_state(x, max_Qa)

    [h, ~, eul, ~, a, ~, V] = ecef_to_flight(x(1:3)', x(4:6)', quaternion(x(7:10)));
    p = eul.pitch;
    [~, ~, ~, rho] = atmosfera(h);
    v2 = V*V;
    Q = 0.5*rho*(v2);
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
function [v, a, b] = pitch_maneuver(~, x, ref, tol, max_Qa)
    %[fts, ~, y] = flight_state(x, max_Qa);
    pe  = x(1:3)';
    qeb = quaternion(x(7:10));
    
    [nav.lat, nav.lng, h] = WGS84.ecef_to_lgv(pe);
    [ep, ey] = att_err(qeb, nav, ref);
   
    if (abs(ep) < tol && abs(ey) < tol) || h <= 0
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




