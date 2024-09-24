function [Fa, po, V, a, QS, M] = aero(h, vb, param)

    [~, s, po, rho] = atmosfera(h);
                    
    V2 = vb'*vb;
    V  = sqrt(V2);
    Q  = 0.5*rho*V2; % presión dinámica
    M  = V/s;        % número de Mach  

    if V > 0.1
        a = atan2(vb(2), vb(1)); % α [rad] 
        if abs(a) > pi/2
            a = a - sign(a) * pi/2;
        end
    else
        a = 0;
    end
    aa = abs(a);
    cx = interp2(param.alpha, param.mach, param.cx, aa, M, 'spline');
    cz = interp2(param.alpha, param.mach, param.cz, aa, M, 'spline') * sign(a);
    QS = Q * param.sref;
    Fa = QS * [cx ; cz];
end
        


