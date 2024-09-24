% Simulador de trayectoria
%
% Argumentos:
%     
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
function [out, tmy, tf, xf] = sim_3Dd(tf, to, xo, param, data, cloop, fnav, fsns, fdisp)

    w = what;
    k = strfind(w.path, 'mtlb');
    root_path = w.path(1:k+4);
    txt = strcat(root_path,'rocket/sim/3D;', root_path,'aero;', path);
    path(txt);

    %%
    if isrow(xo)
        xo = xo';
    end
    ts = param.ts;
    Nk = floor(tf/ts);

    cmd = zeros(4,1);
    tmy = [];
    t_tx  = 1;
    t = zeros(Nk,1);
    x = zeros(Nk,length(xo)+length(cmd)); 
    
    t1     = t_tx;
    x(1,:) = [xo' cmd'];
    for k = 2:Nk 
        
        [sns,      tmy_s] = feval(fsns, to, xo, cmd, param, data);
        [fts, nav, tmy_n] = feval(fnav, sns, param);
        if fts
            disp('FTS')
            t(k+1:end)   = [];
            x(k+1:end,:) = [];
            break;
        else
            [cmd, tmy_c] = feval(cloop, nav, param);
            tmy_k = [tmy_s tmy_n tmy_c];
            if isempty(tmy)
                tmy = zeros(Nk,length(tmy_k));
            end
            tmy(k,:) = tmy_k;                 
            if to >= t1 
                t1 = t1 + t_tx;
                feval(fdisp, to, tmy_k);
                pause(0.01);
            end
        end
        [t_, x_] = ode15s(@(t,y) rocket_xdot(t, y, cmd, param, data), [to to+ts], xo); 

        xo     = x_(end,:)';
        to     = t_(end);
        t(k)   = to ;  
        x(k,:) = [xo' cmd'];
    end
    
    %%
    out.t = t;
    out.p = x(:,1:3);
    out.v = x(:,4:6);
    out.q = x(:,7:10);
    out.w = x(:,11:13);
    out.m = x(:,14);
    out.u = x(:,length(xo)+1:end);
    [out.nav, out.aer, out.frc, out.loss] ...
        = compute(t, out.p, out.v, out.q, out.w, out.m, out.u, param, data);
    tf = t(end);     
    xf = x(end,1:length(xo));
   
end





