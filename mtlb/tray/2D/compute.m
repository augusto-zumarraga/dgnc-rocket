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

    [nav.h, nav.r, nav.p, nav.y, nav.alpha, ae, frc.g, nav.R, vb] = ECI.geo_map(pos, vel, att, param.we); 
    v2 = sum(vel.^2,2);
    v  = sqrt(v2);
    a  = [0 ; (v(2:end)-v(1:end-1))./(t(2:end)-t(1:end-1))]; 
    a(1) = a(2);
    frc.f = a + frc.g .* sin(nav.p); 
    nav.v = v;

    [aer.Fa, aer.po, aer.rho, aer.v, aer.alpha, aer.Q, aer.M] = aero_map(nav.h, vb, param);

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
    loss(4).name = 'steering';
    loss(4).data = -f .* sin(nav.alpha/2).^2;
    
    loss(5).name = 'steering*';
    loss(5).data = -f .* sin(ae/2).^2;

end



