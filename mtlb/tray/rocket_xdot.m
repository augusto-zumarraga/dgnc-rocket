% x = { r, h, v, γ, m }
% α = 0 → γ = θ
%
% param
%   .t    : tiempo de vuelo
%   .mdot : dm/dt para cada t 
%   .ve   : velocidad de salida de la tobera 
%   .pe   : presión de salida de la tobera
%   .Ae   : área de salida de la tobera
%   .mach : rango de números de Mach
%   .cx   : coeficiente de fuerza en xb para cada Mach
%   .sref : superficie de referencia
%
function xd = rocket_xdot(t, x, param, maneouver)

    h  = x(2);
    if h > 0
                
        v  = x(3);
        p  = x(4);
        m  = x(5);
        
        %%
        [X, Ta] = aero(h, v, param);
        if t < param.t(end)
            mdot = interp1(param.t, param.mdot, t);
        else
            mdot = 0;
        end
        if mdot == 0
            T = 0;
        else
            T = mdot*param.ve + Ta;
        end
        
        [g, R] = gravedad(h);
        
        %%
        sy = sin(p);
        cy = cos(p);
        hd = v * sy;
        rd = v * cy;
        vd = (T + X)/m  - g*sy;
        
        %% 
        if T > 0 && (maneouver.mode == 1 || maneouver.mode == 2)
            % pitch over / constat pitch 
            q = maneouver.ref - p;
            if abs(q) > param.pitch_rate
                q = sign(q) * param.pitch_rate;
            end
        else
            if v < 0.01 || p == pi/2
                if T == 0   % apogeo 
                    q = -180/pi;
                else
                    q = 0;
                end
            elseif maneouver.mode == 4
                % steering
                pr = (maneouver.ref - h)/maneouver.ref;
                q  = pr - p;
                if abs(q) > param.pitch_rate
                    q = sign(q) * param.pitch_rate;
                end
            else
                % gravity turn
                q = (v/R - g/v) * cy;
                if T > 0 && abs(q) > param.pitch_rate
                    q = sign(q) * param.pitch_rate;
                end
            end
        end

        %%
        xd = [rd 
              hd
              vd 
               q
              -mdot];
    else
        xd = zeros(size(x));
    end

end

%%
function [X, Ta] = aero(h, v, param)

        [~, a, po, rho] = atmosfera(h);
        M = v/a; % mach number

        %%
        cx = interp1(param.mach, param.cx(:,1), M,'spline','extrap');
        X  = 0.5*rho*v^2 * param.sref * cx;

        Ta = (param.pe - po)*param.Ae;
end


