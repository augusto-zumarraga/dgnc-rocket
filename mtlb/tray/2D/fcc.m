%% -----------------------------------------------------   guiado
% h: altura
% a: ángulo de ataque
% p: ángulo de cabeceo
% y: ángulo de la trayectoria γ
% T: empuje
% R: radio 
% g: aceleración gravitatoria
%
% q = ωb: velocidad angular en terna móvil requerida
function [q, engine] = fcc(t, h, a, p, y, v, T, R, g, param, maneouver)
    if maneouver.mode == 7 % 'coasting'
        q = 0;
        engine = false;
    else    
        engine = T > 0;
        if engine 
            if v > 1
                switch maneouver.mode 
                    case 1 % pitch maneouver
                        ep = maneouver.ref(t) - p;
                        q = ep * param.kp;
                    case {2,6} % constant pitch 
                        ep = maneouver.ref - p;
                        q = ep * param.kp;
                    case 3 % gravity turn
                        q = (v/R - g/v) * cos(y); % 
                    case 4 % proportional steering
                        yr = (maneouver.ref - h)/maneouver.ref;
                        ey = yr - y;
                        fn = ey * param.kn;
                        rp = y + atan(fn/T);
                        if rp < param.p_min
                           rp = param.p_min;
                        end
                        ep = rp - p;
                        q  = ep * param.kp;
                    case 5 % linear tangent steering
                        rp = atan((maneouver.ref(1) - t) * maneouver.ref(2) + maneouver.ref(3));
                        ep = rp - p;
                        q  = ep * param.kp;
                    case 7
                        q = 0;
                        engine = false;
                    otherwise % α = 0
                        q = - a * param.ka;
                end

                % -----------------------------------------------------   control
                if abs(q) > param.pitch_rate
                    q = sign(q) * param.pitch_rate;
                end
            else
                q = 0;
            end
        % coasting
        elseif v < 0.01 || y == pi/2
            q = -1;
        else
            q = -a * param.ka;
        end
    end
end

