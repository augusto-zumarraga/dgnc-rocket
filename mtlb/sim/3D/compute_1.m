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
function [nav, aer, frc, loss] = compute(t, pos, vel, att, wbi, mass, cmd, param, data)

    [nav.h, nav.r, nav.eul, nav.y, nav.alpha, nav.beta, frc.g, nav.R] = ECI.sphere_map(pos, vel, att, param.launch.pos); 

    Z1 = zeros(length(t),1);
    Z3 = zeros(length(t),3);

    nav.v     = sqrt(vel(:,1).^2+vel(:,2).^2+vel(:,3).^2);
    
    aer.Fa    = Z3;
    aer.po    = Z1;
    aer.rho   = Z1;
    aer.a     = Z1;
    aer.b     = Z1;
    aer.v     = Z1;
    aer.alpha = Z1;
    aer.beta  = Z1;
    aer.Q     = Z1;
    aer.M     = Z1;
    
    frc.T     = Z1;

    mk = mass(1);
    for k=1:length(t)
        
        eci = ECI(param.launch.ut1 + t(k));
        [pe, ve] = eci.to_ecef(pos(k,:)', vel(k,:)');
        [nv.lat, nv.lng, nv.hgt] = WGS84.ecef_to_lgv(pe);
        nv.t = t(k);
    
        qib   = quaternion(att(k,:));
        qeb   = eci.ecef_att(qib); 
        wb    = wbi(k,:)';
        act   = cmd(k,:);
        u.tvc = act(1:2)';   % posición del TVC normalizada
        u.alr = act(3);     % posición alerón normalizada   
        [fb, mb, ~, ~, T, air] = sfc(nv, ve, wb, qeb', mass(k), u, data, mk ~= mass(k));
        aer.a(k) = air.a;
        aer.b(k) = air.b;
        aer.v(k) = sqrt(2*air.Q/air.ro);
        aer.M(k) = air.M;
        aer.Q(k) = air.Q;
        frc.T(k) = T;
        
        frc.fb(k,:) = fb';
        frc.mb(k,:) = mb';
        
    end
    
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






