% h : altura
% r : rango (si se especifica po)
% e : euler
% y : γ
% a : alfa
% b : beta
% V : velocidad
% pn: posición ENU (si se especifica po)
function [h, r, e, y, a, b, V, pn] = ecef_to_flight(pe, ve, qeb, po)

    [lat, lng, h] = WGS84.ecef_to_lgv(pe);
    loc = local_geo(lat, lng);  % NED → ECEF  
    qnb = loc.ecef_to_astro_ned(qeb); 
    qbe = qeb';
    
    e = qnb.as_euler;
    
    vb = qbe.trnsf(ve);
    V  = sqrt(vb'*vb);
    if V > 1
        a  = atan2(vb(3), vb(1)); % α
        b  = asin(vb(2)./V);        % β        
    else
        a = 0;
        b = 0;
    end
    vn = qnb.trnsf(vb);
    vh = sqrt(vn(1:2)'*vn(1:2));
    y  = atan2(-vn(3), vh);    
    
    if nargin > 3
        que = local_geo.enu_quat(lat, lng)';
        pn = que.trnsf(pe - po);
        r  = sqrt(pn(1:2)' * pn(1:2)); 
    else
        r  = 0;
    end
end

