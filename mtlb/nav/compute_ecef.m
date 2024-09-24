function [h, r, v, e, y, Q, M, f, T, g] = compute_ecef(t, pe, ve, qbe, param, po)

    h = zeros(length(t),1);
    r = h;
    p = h;
    y = h;
    for k=1:length(t)
       [h(k), r(k), e(k,:), y(k)] = ecef_to_flight(pe(k,:)', ve(k,:)', quaternion(qbe(k,:)), po);
    end  
    v = sqrt(sum(ve.^2,2));
    g = gravedad(h);
    a = [0 ; (v(2:end)-v(1:end-1))./(t(2:end)-t(1:end-1))]; 
    a(1) = a(2);
    f = a + g .* sin(p); 

    [~, a, po, rho] = atmosfera(h);
    M = v./a;
    Q = 0.5*rho.*v.^2;
    
    mdot = interp1(param.t, param.mdot, t);
    T = (param.ve*mdot + param.Ae*(param.pe - po)) .* (mdot > 0);
    
end

