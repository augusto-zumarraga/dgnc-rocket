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
function [out, log, tf, xf] = tray_sim_2D(fp, tf, param, to, xo)

    w = what;
    k = strfind(w.path, 'mtlb');
    root_path = w.path(1:k+4);
    txt = strcat(root_path,'rocket/tray/2D;', root_path,'aero;', path);
    path(txt);

    %%
    if nargin < 4 || isempty(to)
        % = { x, y, u, v, θ, m }
        %    x = R+h , y = 0
        %    u = 0   , v = vt
        %    θ = 0   
        %    m = mi
        p = [WGS84.a+5 0];  
        v = [0 p(1)* param.we];
        x = [p v  0  param.mi];  
        t = 0;
    else
        t = to;
        x = xo;
        if iscolumn(x)
            x = x';
        end
    end    
    r2d = 180/pi;
    param.cx = squeeze(param.cx(:,:,1));
    param.cz = squeeze(param.cz(:,:,1));
    log = '';
    h = ECI.height(x(1:2));
    for k = 1:length(fp)  
        if isempty(fp(k).event) 
            odst = odeset('Event', @(t,y) check(t, y));
        else
            odst = odeset('Event', fp(k).event); 
        end
        if ~isempty(fp(k).exp)
             x(end,end) = x(end,end) - fp(k).exp;
            log = strcat(log, sprintf('\n%0.1fs : mass release %0.0f kg', t(end), fp(k).exp));
        end
        
        t1 = t(end);
        if isempty(fp(k).times) 
            t2 = tf;
        else
            if fp(k).times(1) == 0
                t2 = t1 + fp(k).times(2);
            else
                t2 = fp(k).times(1);
            end
        end
            
        [t_, x_] = ode23t(@(t,y) rocket_xdot(t, y, param, fp(k)) , [t1 t2], x(end,:)', odst); 
        t = [t ; t_(2:end)]; 
        x = [x ; x_(2:end,:)];
        switch fp(k).mode
            case 1   ; man = 'pitch maneuver';
            case 2   ; man = 'constant pitch';
            case 3   ; man = 'prescribed gravity turn'; 
            case 4   ; man = 'proportional steering';
            case 5   ; man = 'linear steering';
            case 6   ; man = 'insertion';
            case 7   ; man = 'coasting';
            case 8   ; man = 'constant pitch rate';
            otherwise; man = 'load relief';
        end
        vi  = x(end,3:4)';
        [h, ~, ~, p, y] = ECI.to_geo(x(end,1:2)', vi, x(end,5));
        vt  = sqrt(vi'*vi)*cos(y);
        log = strcat(log, sprintf('\n%0.1fs : %s complete (%0.0fm/s, %0.0fm, γ=%0.2f, θ=%0.2f⁰)', t(end), man, vt, h, y*r2d, p*r2d));       
        
        xf = x(end,:)';
        if ECI.in_orbit(xf(1:2),xf(3:4)) >= 0
            log = strcat(log, sprintf('\n*************\nIN ORBIT (%0.0fm, γ=%0.2f, vt=%0.2fm/s, m=%0.2fkg)\n*************\n' ...
                                     , h, y*r2d, vt, x(end,end)));
        elseif h <= 0 
            log = strcat(log, sprintf('\n*************\n%s\n*************\n', 'GROUNDED'));
            break;
        elseif t(end) >= tf
            break;
        end    
    end
    %%
    out.t = t;
    out.m = x(:,6);
    out.eci.pos = x(:,1:2);
    out.eci.vel = x(:,3:4);
    out.eci.att = x(:,5);
    [out.nav, out.aer, out.frc, out.loss] = compute(t, out.eci.pos, out.eci.vel, out.eci.att, out.m, param);
    tf = t(end);         
    
end

% pahse termination functions
% function [value,isterminal,direction] = height(~, x, ht)
%     h = sqrt(x(1:2)'*x(1:2)) - WGS84.a;
%     value      = [ h ht-h];  
%     isterminal = [ 1  1  ]; 
%     direction  = [-1 -1  ];  
% end
function [value,isterminal,direction] = check(t, x, tf)
    h = ECI.height(x(1:2));
    if nargin < 3
        value      = [ h ];  
        isterminal = [ 1 ]; 
        direction  = [-1 ];  
    else
        value      = [ h tf-t];  
        isterminal = [ 1  1  ]; 
        direction  = [-1 -1  ];  
    end
end
  


