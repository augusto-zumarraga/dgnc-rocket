% Simulador de trayectoria
%
%
function [out, log, tf, xf] = tray_sim_3D(fp, tf, param, to, xo)

    w = what;
    k = strfind(w.path, 'mtlb');
    root_path = w.path(1:k+4);
    txt = strcat(root_path,'rocket/tray/3D;', root_path,'aero;', path);
    path(txt);

    %%
	r2d = 180/pi;
    eci = ECI(param.launch.ut1);
    po  = eci.to_eci(param.launch.pos);
    if nargin < 4 || isempty(to)
        % = { p, v, qib, m }
        vo = eci.ground_speed(po);
        qo = eci.eci_att(param.launch.att); 
        x  = [po' vo' qo.q' param.mi];
        t  = 0;
    else
        t = to;
        x = xo;
        if iscolumn(x)
            x = x';
        end
    end
    log = '';
    %h   = alt(x(1:3)', param.launch.ut1 + t);    
    for k = 1:length(fp)  

        if isempty(fp(k).event) 
            odst = odeset('Event', @(t,y) check(t, y, param.launch.ut1));
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
        xf = x(end,:)';
        pc = xf(1:3);
        vc = xf(4:6);
        qc = quaternion(xf(7:10));
        
        [vg, qg] = sphere.to_ned(pc, vc, qc);
        vt  = sqrt(vg(1:2)'*vg(1:2));
        y   = atan(-vg(3)/vt);
        p   = qg.pitch();
        h   = alt(pc,t(end)); 
        log = strcat(log, sprintf('\n%0.1fs : %s complete (%0.0fm/s, %0.0fm, γ=%0.2f, θ=%0.2f⁰)', t(end), man, vt, h, y*r2d, p*r2d));       
        
        if ECI.in_orbit(pc, vc) > 0 
            log = strcat(log, sprintf('\n*************\nIN ORBIT (%0.0fm, γ=%0.2f, vt=%0.2fm/s)', h, y*r2d, vt));
        elseif h <= 0 
            log = strcat(log, sprintf('\n*************\n%s', 'GROUNDED'));
            break;
        elseif t(end) >= tf
            break;
        end    
    end
    %%
    out.t = t;
    out.m = x(:,11);
    out.eci.pos = x(:,1:3);
    out.eci.vel = x(:,4:6);
    out.eci.att = x(:,7:10);
    [out.nav, out.aer, out.frc, out.loss] = compute(t, out.eci.pos, out.eci.vel, out.eci.att, out.m, param);
    tf = t(end);          
    
end

function h = alt(pc, t)
%     eci = ECI(t);
%     pe  = eci.ecef_pos(pc);    
%     [~,~, h] = WGS84.ecef_to_lgv(pe);
    h = sqrt(pc'*pc) - WGS84.a;
end
function [value,isterminal,direction] = check(t, x, ut1)
    h = alt(x(1:3),ut1+t); 
    if nargin < 3
        value      =  h ;  
        isterminal =  1 ; 
        direction  = -1 ;  
    else
        value      =  h ;  
        isterminal =  1 ; 
        direction  = -1 ;  
    end
end


% % phase termination functions
% function [value,isterminal,direction] = height(t, x, ht, ut1)
%     h = alt(x(1:3),ut1+t); 
%     value      = [ h ht-h];  
%     isterminal = [ 1  1  ]; 
%     direction  = [-1 -1  ];  
% end
% 
% function [value,isterminal,direction] = pitch_maneuver(t, x, ref, tol,ut1)
%     [~, qnb] = sphere.to_ned(x(1:3), x(4:6), quaternion(x(7:10)));
%     h  = alt(x(1:3),ut1+t); 
%     rp = quaternion(ref);
%     ep = att_err(qnb, rp);
%     if abs(ep) < tol
%         ep = 0;
%     end
%     value      = [ h ep];  
%     isterminal = [ 1  1]; 
%     direction  = [-1  0];  
% end
%    
% function [value,isterminal,direction] = grounded(t, x, tf, ut1)
%     h = alt(x(1:3),ut1+t); 
%     value      = [ h tf-t];  
%     isterminal = [ 1  1  ]; 
%     direction  = [-1 -1  ];  
% end
% 
% function [value,isterminal,direction] = in_orbit(t, x, tf, ut1)
%     y = ECI.in_orbit(x(1:3), x(4:6));
%     h = alt(x(1:3),ut1+t); 
%     value      = [ h   y tf-t];  
%     isterminal = [ 1   1   1]; 
%     direction  = [-1   1  -1];  
% end        



        
 
