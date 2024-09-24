function [Fa, po, rho, V, a, Q, M] = aero_map(h, vb, param)

    [~, s, po, rho] = atmosfera(h);
                    
    V2 = vb(:,1).^2+vb(:,2).^2;
    V  = sqrt(V2);
    Q  = 0.5*rho.*V2;  % presión dinámica
    M  = V./s;         % número de Mach  

    a  = atan2(vb(:,2), vb(:,1)) .* (V > 0.1); % α [rad] 
    aa = abs(a);
    cx = interp2(param.alpha, param.mach, param.cx, aa, M, 'spline');
    cz = interp2(param.alpha, param.mach, param.cz, aa, M, 'spline') .* sign(a);
    Fa = [Q.*cx Q.*cz]* param.sref;
end
        


