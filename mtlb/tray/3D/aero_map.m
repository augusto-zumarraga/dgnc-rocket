function [Fa, po, rho, V, a, b, Q, M] = aero_map(h, vb, param)

    [~, s, po, rho] = atmosfera(h);
                    
    V2 = vb(:,1).^2+vb(:,2).^2+vb(:,3).^2;
    V  = sqrt(V2);
    Q  = 0.5*rho.*V2;  % presión dinámica
    M  = V./s;         % número de Mach  

    a  = atan2(vb(:,3), vb(:,1)) .* (V > 0.1); % α [rad] 
    b  = asin (vb(:,2)./V)       .* (V > 0.1); % β [rad] 
    
    k = abs(a) > pi/2;
    a = a - sign(a) .* k * pi/2;
    k = abs(b) > pi/2;
    b = b - sign(b) .* k * pi/2;
    
    aa = abs(a);
    bb = abs(b);
    cx = interp3(param.alpha, param.mach, param.beta, param.cx, aa, M, bb, 'spline');
    cy = interp3(param.alpha, param.mach, param.beta, param.cy, aa, M, bb, 'spline') .* sign(b);
    cz = interp3(param.alpha, param.mach, param.beta, param.cz, aa, M, bb, 'spline') .* sign(a);
    Fa = [Q.*cx Q.*cy Q.*cz]* param.sref;
end
        