% PROPULSION
%
% t  : tiempo
% po : presión atmosférica
% tvc: deflexión alrededor de {y,z} del TVC
%    +tvc(1) → q↑ → Tz+ 
%    +tvc(2) → r↑ → Ty-
% dgm: brazo de palanca del TVC 
function [F, M, T, mdot] = prop(t, po, tvc, dgm, param)

    %% propulsión
    mdot = interp1(param.t, param.mdot, t);
    if mdot == 0
        T =  0;
        F = [0 ; 0];
        M =  0;
    else
        T  = mdot*param.ve + (param.pe - po)*param.Ae;
        tvc = T * tvc * param.d_tvc;
        F  = [T ; tvc];
        M  = tvc * dgm;
    end

end

