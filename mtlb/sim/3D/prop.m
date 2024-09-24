% PROPULSION
%
% t  : tiempo
% po : presión atmosférica
% tvc: deflexión alrededor de {y,z} del TVC
%    +tvc(1) → q↑ → Tz+ 
%    +tvc(2) → r↑ → Ty-
% dgm: brazo de palanca del TVC 
function [F, M, T, mdot] = prop(t, po, tvc, dgm, param)

    mdot = param.mdot;
    %dot = interp1(param.t, param.mdot, t, 'linear', 0);
    if length(param.ve) > 1 
        ve = interp1(param.pa, param.ve, po, 'linear','extrap');
    else
        ve = param.ve;
    end
    T   = mdot*ve + (param.pe - po)*param.Ae;        
    tvc = T * tvc * param.d_max;
    F   = [T ; -tvc(2) ; tvc(1)];
    M   = [0 ;  tvc] * dgm;

end
