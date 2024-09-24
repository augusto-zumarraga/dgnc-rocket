% Cómputos de fuerza y momento específico. Como subproductos
% se devuelven también parámetros de propulsión, aire y tensor de inercia 
%
%    nav: {lat lng hgt t}
%    ve : velocidad ECEF
%    wb : velocidad angular inercial en terna body 
%    qeb: cuaternion de actitud body -> ECEF
%    m  : masa
%    tvc: deflexiones {y,z} normalizadas del TVC
%    alr: deflexiones {y,z} normalizadas del TVC
%
function [fb, mb, mdot, xcg, T, air, ltv, J] = sfc(nav, ve, wb, qbe, m, act, data, T_on)

    % propiedades másicas
    [xcg, J] = mass(m, data.mass); 
    dca = data.aero.XCG - xcg; % + si CG está más a proa
    ltv = data.thrust.x_gimbal - xcg;  
    
    % aerodinámica  
    loc = local_geo(nav.lat, nav.lng);
    qen = loc.as_ned();
    [vw, ww] = wind(nav.hgt, ve, wb, qbe, qen, data);
    
%     if isfield(act, 'alr') 
%         param.fin(end).delta = act.alr * param.alr_max;
%     end
    [Fa, Ma, air] = aero(nav.hgt, vw, ww, dca, data.aero); %, param.fin);
    
    % propulsión
    if T_on && m > data.mass.m(1)
        [Fp, Mp, T, mdot] = prop(nav.t, air.po, act.tvc, ltv, data.thrust); 
    else
        mdot = 0; T = 0;
        Fp = [0;0;0];
        Mp = Fp;
    end

    fb = (Fa+Fp)/m; 
    mb = J\(Ma+Mp);
    
end


