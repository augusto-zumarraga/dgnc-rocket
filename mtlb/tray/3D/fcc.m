% -----------------------------------------------------   guiado
% a   : ángulo de ataque [rad]
% b   : ángulo de deslizamiento [rad]
% att : cuaternión de actitud NED
% nav : cooerdanas de navegación
% vg  : velocidad en terna geográfica local
% prop: estado de la propulsión 
function [w,eng] = fcc(t, aer, nav, att, T, prm, maneouver)
  
    eng = T > 0;
    if eng 
        if nav.h > 10
            switch maneouver.mode 
                case 0 % α = 0, β = 0
                    q = - aer.a * prm.ka;
                    r = - aer.b * prm.ka;
                case 3 % prescribed gravity turn
                    q = (nav.V/nav.R - nav.g/nav.V) * cos(nav.y); % 
                    r = 0;
                case 7 % coasting
                    q = 0;
                    r = 0;
                    eng = false;
                otherwise
                    switch maneouver.mode 
                        case 1 % pitch maneouver
                            p = maneouver.ref(t);
                            qref = quaternion(0, p, prm.launch.hdg);
                            [ep, ey] = att_err(att, qref);
                        case {2,6} % constant pitch  
                            ref = maneouver.ref;
                            [ep, ey] = att_err(att, ref);
%                         case 4 % proportional steering
%                             yr = (maneouver.ref - nav.h)/maneouver.ref;
%                             ey = yr - nav.y;
%                             fn = ey * prm.kn;
%                             rp = y + atan(fn/T);
%                             if rp < prm.p_min
%                                rp = prm.p_min;
%                             end
%                             ep = nav.pitch - rp;
%                             ey = nav.yaw;
                        case 5 % linear tangent steering
                            rp = atan((maneouver.ref(1) - t) * maneouver.ref(2) + maneouver.ref(3));
                            ep = nav.qtg.pitch - rp;
                            ry = (nav.xte * prm.kx + prm.kv*nav.vtg(2))/nav.V;
                            ey = 0; %nav.qtg.yaw - ry;
                    end
                    q  = -ep * prm.kp;
                    r  = -ey * prm.kp;
            end
            % -----------------------------------------------------   control
            if abs(q) > prm.pitch_rate
                q = sign(q) * prm.pitch_rate;
            end
            if abs(r) > prm.pitch_rate
                r = sign(r) * prm.pitch_rate;
            end
        else
            q = 0;
            r = 0;
        end
    elseif nav.V < 0.01 % || y == pi/2
        q = -1;
        r =  0;
    else
        q = -prm.ka * aer.a; % sign(a)*sqrt(abs(a))
        r =  prm.ka * aer.b; % sign(b)*sqrt(abs(b)) 
    end
    w = [0;q;r];
end



