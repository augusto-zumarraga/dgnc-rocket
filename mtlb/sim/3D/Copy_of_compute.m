% nav
%   .h : altura
%   .r : rango
%   .R : radio geocéntrico
%   .p : ángulo de cabeceo en terna geográfica local
%   .y : desvío la trayectoria respecto de la tangente a una orbita circular 
%   .alpha: ángulo de ataque respecto de la trayectoria 
%
% aer 
%   .po : presión atmosférica
%   .rho: densidad atmosférica
%   .v  : velocidad respecto del aire
%   .alpha: ángulo de ataque respecto del aire
%   .Q  : presión dinámica
%   .M  : número de mach 
%   .Fa : fuerza aerodinámica
%
% frc
%   g : gravitación
%   f : fuerza específica neta
%   T : empuje del motor
%
function [nav, aer, frc, loss] = compute(t, pos, vel, att, mass, param)

    [nav.h, nav.r, nav.eul, nav.y, nav.alpha, nav.beta, frc.g, nav.R, vb] = ECI.sphere_map(pos, vel, att, param.launch.pos); 
    v2 = sum(vel.^2,2);
    v  = sqrt(v2);
    a  = [0 ; (v(2:end)-v(1:end-1))./(t(2:end)-t(1:end-1))]; 
    a(1) = a(2);
    frc.f = a + frc.g .* sin(nav.eul(:,2)); 
    nav.v = v;

    [aer.Fa, aer.po, aer.rho, aer.v, aer.alpha, aer.beta, aer.Q, aer.M] = aero_map(nav.h, vb, param);

    mdot = interp1(param.t, param.mdot, t, 'linear', 0);
    if length(param.ve) > 1 
        ve = interp1(param.pa, param.ve, aer.po, 'linear','extrap');
    else
        ve = param.ve;
    end
    frc.T = (mdot.*ve + param.Ae*(param.pe - aer.po)) .* (mdot > 0);
    
    sa = sin(nav.alpha);
    ca = cos(nav.alpha);
    
    f = frc.T ./ mass; % T/m
    loss(1).name = 'thrust';
    loss(1).data =  f .* ca;
    loss(2).name = 'gravitation';
    loss(2).data = -frc.g .* sin(nav.y);
    loss(3).name = 'drag';
    loss(3).data =  (aer.Fa(:,1) .* ca - aer.Fa(:,2) .* sa) ./ mass;
    loss(4).name = 'steering α';
    loss(4).data = -f .* sin(nav.alpha/2).^2;
    loss(5).name = 'steering β';
    loss(5).data = -f .* sin(nav.beta/2).^2;

end




% function out = compute(t, x, param, coef)    
%     %%
%     out.t   = t;
%     out.pe  = x(:,1:3);
%     out.ve  = x(:,4:6);
%     out.qeb = x(:,7:10);
%     out.wb  = x(:,11:13);
%     out.m   = x(:,14);
%     out.cmd = x(:,15:end);
%     
%     Z  = zeros(size(t));
%     out.r = Z;
%     out.h = Z;
%     out.p = Z;
%     out.y = Z;
%     
%     out.a = Z;
%     out.b = Z;
%     out.v = Z;
%     out.Q = Z;
%     out.M = Z;
%     out.T = Z;
% 
%     Z3 = zeros(length(t),3);
%     out.eul = Z3;
%     out.fb  = Z3;
%     out.mb  = Z3;
%     
%     for k=1:length(t)
%         pe  = out.pe(k,:)';
%         ve  = out.ve(k,:)';
%         qeb = quaternion(out.qeb(k,:));
% 
%         [out.h(k), out.r(k), e, out.y(k), out.a(k), out.b(k), out.v(k), pn] = ecef_to_flight(pe, ve, qeb, out.pe(1,:)');
%         out.eul(k,:) = [e.roll e.pitch e.yaw];
%         out.pn (k,:) = pn';
%         out.p  (k)   = e.pitch;
% 
%         [nav.lat, nav.lng, nav.hgt] = WGS84.ecef_to_lgv(out.pe(k,:)');
%         out.h (k) = nav.hgt;
%         nav.t = t(k);
%         
%         act.tvc = out.cmd(k,1:2)';
%         act.alr = out.cmd(k,3);
%         
%         [fb, mb, ~, T, air] = sfc(nav, ve, out.wb(k,:)', qeb', out.m(k), act, param, coef);
%         out.a (k) = air.a;
%         out.b (k) = air.b;
%         out.v (k) = sqrt(2*air.Q/air.ro);
%         out.M (k) = air.M;
%         out.Q (k) = air.Q;
%         out.T (k) = T;
%         
%         out.fb(k,:) = fb';
%         out.mb(k,:) = mb';
%         
%     end
% end






