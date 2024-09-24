% Cómputos de fuerza y momento específico. Como subproductos
% se devuelven también parámetros de propulsión y de aire
%
function [fb, mp, mdot, T, air] = sfc(nav, vn, dcm_bn, q, m, tvc, param)

    % desplazamiento del CG (+ si el CG está más a popa del punto de referencia)
    dca = interp1(param.mass, param.x_ref, m, 'pchip','extrap');
    % brazo de palanca del TVC
    ltv = interp1(param.mass, param.ltvc , m, 'pchip','extrap');
    % Momento de inercia
    Iy  = interp1(param.mass, param.Iy   , m, 'pchip','extrap');  

%    y   = atan2(-vn(2), vn(1));  % γ    


    %% viento
    vb = wind(nav.h, vn, dcm_bn, param);   
    
    %% aerodinámica
    [Fa, Ma, air] = aero(nav.h, vb, q, dca, param);

    %% propulsión
    [Fp, Mp, T, mdot] = prop(nav.t, air.po, tvc, ltv, param);


    %% efectos adicionales
    fb = (Fa + Fp)/m;
%     if vb(1) > 0.1
%         % da/dt
%         ab = fb + fg; 
%         ad = 1/v^2 * (vb(1)*ab(2) - vb(2)*ab(1));      
%         [Fad, Mad] = aero_ad(Mch, a, ad, param, v, QS);
%         fb = fb + Fad/m;
%         M  = M  + Mad;
%     end
    mp = (Ma + Mp)/Iy;

end
