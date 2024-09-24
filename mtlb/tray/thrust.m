function [T, mdot] = thrust(t, po, param)
    mdot = interp1(param.t, param.mdot, t, 'linear', 0);
    if mdot > 0
        if length(param.ve) > 1 
            ve = interp1(param.pa, param.ve, po, 'linear','extrap');
        else
            ve = param.ve;
        end
        T = mdot*ve + (param.pe - po)*param.Ae;
    else
        T = 0;
    end
end
        


