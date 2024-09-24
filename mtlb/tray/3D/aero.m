% h : altura
% vb: velocidad en terna b
%
% Fa: fuerza aerodinámica en terna b
% Ta: empuje por presión
% a : ángulo de ataque [rad]
% b : ángulo de deslizamiento [rad]
% Q : presión dinámica 
function [Fa, po, V, a, b, Q] = aero(h, vb, param)

    [~, vs, po, rho] = atmosfera(h);

    V2 = vb'*vb;
    V  = sqrt(V2);
    Q  = 0.5*rho*V2;  % presión dinámica
    M  = V/vs;        % número de Mach  

    if V > 0.1
        a = atan2(vb(3), vb(1)); % α
        b = asin(vb(2)/V);       % β
        if abs(a) > pi/2
            a = a - sign(a) * pi/2;
        end
        if abs(b) > pi/2
            b = b - sign(b) * pi/2;
        end
    else
        a = 0;
        b = 0;
    end
    
    aa = abs(a);
    bb = abs(b);
    if M <= param.mach(1)
        idx = 1;    
        frc = [];
    elseif M >= param.mach(end)
        idx = length(param.mach);    
        frc = [];
    else
        idx = floor(interp1(param.mach, 1:length(param.mach), M));
        M1  = param.mach(idx  );
        M2  = param.mach(idx+1);
        frc = (M-M1)/(M2-M1);
    end
    cx  = interp_M(param, param.cx, aa, bb, idx, frc);
    cy  = interp_M(param, param.cy, aa, bb, idx, frc) * sign(b);
    cz  = interp_M(param, param.cz, aa, bb, idx, frc) * sign(a);    
%     cx = interp3(param.alpha, param.mach, param.beta, param.cx, aa, M, bb, 'spline');
%     cy = interp3(param.alpha, param.mach, param.beta, param.cy, aa, M, bb, 'spline') * sign(b);
%     cz = interp3(param.alpha, param.mach, param.beta, param.cz, aa, M, bb, 'spline') * sign(a);
    QS = Q * param.sref;
    Fa = QS * [cx ; cy ; cz];
  
end

function c = interp_M(coef, CF, aa, bb, idx, frc)
    if isempty(frc)
        CF = squeeze(CF(idx,:,:));
    else
        CF = squeeze(CF(idx,:,:)*(1-frc) + CF(idx+1,:,:)*frc);
    end
    c  = interp2(coef.beta, coef.alpha, CF, bb, aa, 'spline');
end





