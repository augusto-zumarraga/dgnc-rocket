% Simulador de trayectoria
%
%
function [sim, log] = tray_sim_ecef(fp, tf, param)

	d2r = pi/180;
    ts = 0.1;
    % = { pe, ve, qeb, m }
    po = param.launch.pos;
    qo = param.launch.att;
    x  = [ po' [0 0 0] qo.q' param.mi];
    t  = 0;
    
    p_tol = 0.1*d2r;   
    log   = '';
    for k = 1:length(fp) 
        switch fp(k).mode 
            case 1     % pitch over 
                odst = odeset('MaxStep', ts, 'Event', @(t,y) pitch_maneuver(t, y, fp(k).ref, p_tol)); 
            case 2     % constant pitch 
                if k < length(fp) && fp(k+1).start > 0
                    odst = odeset('MaxStep', ts,'Event', @(t,y) height(t, y, fp(k+1).start)); 
                else
                    odst = odeset('MaxStep', ts,'Event', @(t,y) grounded(t, y));
                end
            otherwise  % gravity turn (0)
                if k < length(fp) && fp(k+1).start > 0
                    odst = odeset('MaxStep', ts,'Event', @(t,y) height(t, y, fp(k+1).start)); 
                else
                    odst = odeset('MaxStep', ts,'Event', @(t,y) grounded(t, y));
                end
        end
            
        [t_, x_] = ode23t(@(t,y) rocket_xdot_ecef(t, y, param, fp(k)) , [t(end) tf], x(end,:)', odst); 
        t = [t ; t_(2:end)]; 
        x = [x ; x_(2:end,:)];
        switch fp(k).mode
            case 1   ; man = 'pitch maneuver';
            case 2   ; man = 'constant pitch';
            otherwise; man = 'gravity turn';
        end
        [h, r, e, y] = ecef_to_flight(x(end,1:3)', x(end,4:6)', quaternion(x(end,7:10)), po);
        log = strcat(log, sprintf('\n%0.1fs : %s complete (%0.0f/%0.0f m, θ=%0.2f⁰, γ=%0.2f⁰)'...
                     , t(end), man, h, r, e.pitch*180/pi, y*180/pi));
        
        if h <= 0 || t(end) >= tf
            break;
        end        
    end
    %%
    %m = x(:,end);
    sim.t   = t;
    sim.pe  = x(:,1:3);
    sim.ve  = x(:,4:6);
    sim.qbe = x(:,7:10);
    sim.m   = x(:,11);
    Z = zeros(size(t));
    sim.r = Z;
    sim.h = Z;
    sim.a = Z;
    sim.b = Z;
    sim.y = Z;
    sim.v = Z;
    Z3 = zeros(length(t),3);
    sim.eul= Z3;    
    
    for k=1:length(t)
        pe = sim.pe(k,:)';
        ve = sim.ve(k,:)';
        qbe= quaternion(sim.qbe(k,:));
        [sim.h(k), sim.r(k), e, sim.y(k), sim.a(k), sim.b(k), sim.v(k), pn] = ecef_to_flight(pe, ve, qbe, po);
        sim.eul(k,:) = [e.roll e.pitch e.yaw];
        sim.pn (k,:) = pn';
        sim.f(k) = 0; % TODO 
    end
    [sim.Q, sim.M, sim.T, sim.Fn, sim.g] = compute(t, sim.h, sim.v, sim.a, sim.b, param);

end

%%
% Q: presión dinámica
% M: número de mach;
% f: fuerza específica
% T: empuje del motor
function [Q, M, T, Fn, g] = compute(t, h, v, alfa, beta, param)
    v2= v.^2;
    g = gravedad(h);
    [~, vs, po, rho] = atmosfera(h);
    M = v./vs;
    Q = 0.5*rho.*v2;
    
    mdot = interp1(param.t, param.mdot, t);
    T    = (param.ve*mdot + param.Ae*(param.pe - po)) .* (mdot > 0);
    cz   = interp3(param.alpha, param.mach, param.beta, param.cz, alfa, M, beta, 'spline');    
    Fn   = Q .* param.sref .* cz;
end

%% flight termination function
function [v, a, b] = height(~, x, h_max)
	[~ ,~ , h] = WGS84.ecef_to_lgv(x(1:3));    
    if h >= h_max || h <= 0
        v = 1;
        a = 1;
    else
        v = 0;
        a = 0;
    end
    b = 0;
end
function [v, a, b] = pitch_maneuver(t, x, ref, tol)
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
function [v, a, b] = grounded(~, x)
    [~, ~, h] = WGS84.ecef_to_lgv(x(1:3));
    if h <= 0
        v = 1;
        a = 1;
    else
        v = 0;
        a = 0;
    end
    b = 0;
end


