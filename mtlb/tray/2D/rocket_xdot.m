% x = { pi[2], vi[2], θi, m }
% pi: posición en referencia inercial (2D cartesioano estandard)
% vi: velocidad en el sistema de coordenadas inercial
% θi: ángulo de cabeceo respecto del eje xi (antihorario)
% m : masa
%
% param
%   .t    : tiempo de vuelo
%   .mdot : dm/dt para cada t 
%   .ve   : velocidad de salida de la tobera 
%   .pe   : presión de salida de la tobera
%   .Ae   : área de salida de la tobera
%
%   .sref : superficie de referencia
%   .mach : rango de números de Mach
%   .alpha: rango de ángulos de ataque
%
%   para cada número de mach y ángulo de ataque:
%
%   .cx   : coeficiente de fuerza en xb  
%   .cz   : coeficiente de fuerza en zb 
%
% maneouver: ver función flight_phase()
function xd = rocket_xdot(t, x, param, maneouver)

    vi = x(3:4);
    pc = x(1:2);
    [h, R, b, p, y, g] = ECI.to_geo(pc, vi, x(5));
    if h > 0
        %%
        
        [vb, Cib] = ECI.to_body(b, x(5), vi, [R*param.we;0]);
        
        [F, po, ~, a] = aero(h, vb, param);
        [T, mdot] = thrust(t, po, param);
                     
        %% ================================================================
        % q = ωb: velocidad angular en terna móvil 
        [qg, eng] = fcc(t, h, a, p, y, sqrt(vi'*vi), T, R, g, param, maneouver); 
%         qi = param.we*WGS84.a/R - qg;
        qi = - qg;
        if eng
            F(1) = F(1)+T;
        end
        fb = F/x(6);
        fg = [-sin(p) ; cos(p)] * g; % (N)ED
        
        %%
        fi = Cib * (fb + fg);  
        xd = [vi
              fi 
              qi  
             -mdot];
    else
        xd = zeros(size(x));
    end

end




