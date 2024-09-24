% Simulador de trayectoria 3DOF XZ
%
% Argumentos:
%     
%    tf: tiempo máximo para la simulación
%    xo: estado inicial { r, h, vn[2], θ, ωy, m, tvc } 
% param: parámetros del vehículo. Sus campos necesarios son los requeridos
%        por la función rocket_xdot_2Df()
%
% Las salidas son arrays con valores en los cuales cada fila corresponde 
% a un instante del vuelo.
%
%  log: contiene un registro de texto de los resultados de cada fase
%
function [t, x, tmy, param] = sim_2Dd(tf, xo, param, cloop, fnav, fdisp)

    w = what;
    k = strfind(w.path, 'mtlb');
    root_path = w.path(1:k+4);
    txt = strcat(root_path,'rocket/sim/2D;', root_path,'aero;', path);
    path(txt);

    if isrow(xo)
        xo = xo';
    end
    param.cx = squeeze(param.aero.CX(:,:,1));
    param.cz = squeeze(param.aero.CZ(:,:,1));    
    param.cm = squeeze(param.aero.CM(:,:,1));
    
    param.czq = squeeze(param.aero.CZQ(:,:,1));    
    param.cmq = squeeze(param.aero.CMQ(:,:,1));    

    param.czad = squeeze(param.aero.CZAD(:,:,1));    
    param.cmad = squeeze(param.aero.CMAD(:,:,1));    
    
    ts = param.ts;
    Nk = floor(tf/ts);

    cmd   = 0;
    t_tx  = 1;
    t = zeros(Nk,1);
    x = zeros(Nk,length(xo)); 
    
    to = 0;
    t1 = t_tx;
    x(1,:) = xo';
    tmy = [];
    for k = 2:Nk 
        
        % commands saturation
        if abs(cmd) > 1
            cmd = sign(cmd);
        end
        % time step
        [t_, x_] = ode23t(@(t,y) rocket_xdot_f(t, y, cmd, param), [to to+ts], xo); 

        % sample instant
        xo     = x_(end,:)';
        to     = t_(end);
        t(k)   = to ;  
        x(k,:) = xo';
        [fts, y, tmy_n] = feval(fnav, to, xo, param);
        if fts
            disp('FTS')
            t(k+1:end)   = [];
            x(k+1:end,:) = [];
            break;
        else
            y.t   = to;
            [cmd, tmy_c] = feval(cloop, y, param);
            tmy_k = [tmy_n tmy_c];
            if isempty(tmy)
                tmy = zeros(Nk,length(tmy_k));
            end
            tmy(k,:) = tmy_k;          
            if to >= t1 
                t1 = t1 + t_tx;
                feval(fdisp, to, tmy_k);
            end
        end
    end

end


