% Cómputo de EFECTOS AERODINAMICOS
%
% dca: desplazamiento del CG respecto de X_REF (+ si el CG está más a proa) 
function [Fa, Ma, air] = aero(h, vb, q, dca, param)

    vb(2) = vb(2) + q*dca;  % vb en x_ref         
        
    [~, vs, po, rho] = atmosfera(h);
 
    V2 = vb'*vb;
    V  = sqrt(V2);
    Q  = 0.5*rho*V2;  % presión dinámica
    M  = V/vs;        % número de Mach  

    if V > 0.1
        a = atan2(vb(2), vb(1)); % α
    else
        a = 0;
    end
    if V > 0
        q = q * param.lref / (2*V);
    end
    aa  = abs(a);
    cx  = interp2(param.alpha, param.mach, param.cx, aa, M, 'spline');
    cz  = interp2(param.alpha, param.mach, param.cz, aa, M, 'spline') * sign(a);
    cm  = interp2(param.alpha, param.mach, param.cm, aa, M, 'spline') * sign(a);
    
    czq = interp2(param.alpha, param.mach, param.czq, aa, M, 'spline') * q;
    cmq = interp2(param.alpha, param.mach, param.cmq, aa, M, 'spline') * q;

    QS = Q * param.sref;
    Fa = QS * [cx ; cz+czq];
    Ma = QS * param.lref * (cm + cmq);

    Ma = Ma + dca * Fa(2);
    
    air.ro = rho;
    air.po = po;
    air.a  = a;
    air.V  = V;
    air.Q  = Q;
    air.M  = M;
end
% function [Fa, Ma] = aero_ad(M, a, ad, param, V, QS)
%     ad = ad * param.lref / (2*V);
%     aa = abs(a);
%     czad = interp2(param.alpha, param.mach, param.czad, aa, M, 'pchip') * ad;
%     cmad = interp2(param.alpha, param.mach, param.cmad, aa, M, 'pchip') * ad;
%     Fa = QS * [0 ; czad];
%     Ma = QS * param.lref * cmad;
% end



