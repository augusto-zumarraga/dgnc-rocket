classdef ECI
    methods(Static)
        function [R, R2] = Rc(pc)
            R2  = pc(1).^2 + pc(2).^2;
            R   = sqrt(R2);
        end
        % h  : altura
        % R  : radio geocéntrico
        % Cgi: matriz de rotación a terna geográfica local (N)ED
        % p  : ángulo de cabeceo en terna geo loc
        % y  : desvío la trayectoria respecto de la tangente a una
        %      orbita circular 
        % g  : aceleración gravitacional
        function [h, R, b, p, y, g] = to_geo(pc, vc, qc)
            [R, R2] = ECI.Rc(pc);
            g = WGS84.ge * WGS84.a2/R2;
            h = R - WGS84.a;
            
            l = atan2(pc(2), pc(1)); % longitud  
            b = l + pi/2;            % inclinación del horizonte
            p = b - qc; 

            d = atan2(vc(2), vc(1));  % inclinación de la velocidad
            y = b - d;
            if  y > 2*pi
                y = y - 2*pi;
            end
        end 
        % h  : alturas
        % r  : rango
        % p  : ángulo de cabeceo en terna geo loc
        % y  : desvío la trayectoria respecto de la tangente a una
        %      orbita circular 
        % a  : ángulo entre empuje y velocidad
        % g  : aceleración gravitacional        
        % R  : radio geocéntrico
        % vb : ground speed in body coordinates
        function [h, r, p, y, a, ae, g, R, vb] = geo_map(pc, vc, qc, we)
            R2  = pc(:,1).^2 + pc(:,2).^2;
            R   = sqrt(R2);
            g   = WGS84.ge * WGS84.a2./R2;
            h   = R - WGS84.a;
            
            l   = atan2(pc(:,2), pc(:,1));  
            b   = l + pi/2; % horizonte
            p   = b - qc;   % cabeceo respecto del horizonte local

            r   = l * WGS84.a;  
            
            %% ángulo de la trayectoria respecto del horizonte local 
            d = atan2(vc(:,2), vc(:,1));  % inclinación de la velocidad
            a = d - qc;
            y = b - d;
            y = y - 2*pi*(y > 2*pi);
            
            de= atan2(vc(:,2)-R*we, vc(:,1));  % inclinación de la velocidad
            ae= qc - de;  
            
            %% velocidad relativa respecto de la tierra en coordenadas i
            s   = sin(b);
            c   = cos(b);
            
            vg  = vc - [c.*R  s.*R] * we;
            s   = sin(qc);
            c   = cos(qc);
            vb  = [vg(:,1).*c+vg(:,2).*s vg(:,2).*c-vg(:,1).*s];    
            
        end 
        function h = height(pc)
            h = ECI.Rc(pc) - WGS84.a;
        end        
        function [g, R, R2] = gravedad(pc)
            [R, R2] = ECI.Rc(pc);
            g  = WGS84.GM./R2;
        end     
        function [vf, g, R] = v_orbit(h)
            R  = WGS84.a + h;
            g  = WGS84.GM./(R.^2);
            vf = sqrt(g.*R);
        end    
        function [y, sp] = in_orbit(pc, vc)
            sp.rad = sqrt(pc(1)^2+pc(2)^2);  
            p  = atan2(pc(2), pc(1));
            cs = cos(p); sn = sin(p);
            sp.Cni = [-sn cs ; 
                       cs sn ];
            vl = sp.Cni * vc;
            vo = sqrt(WGS84.GM/sp.rad);
            y  = vl(1) - vo;
        end        
        function [vg, p] = path(pc, vc, qc)
            l = atan2(pc(2), pc(1));  
            b = l + pi/2;
            Cgi = ECI.dcm(b);
            vg = Cgi * vc;
            if nargin > 2
                p = b - qc; 
            else    
                p = b;
            end
        end 
        % b : inclinación del horizonte en terna i
        % p : actitud en terna i
        % vi: velocidad en terna i
        % vg: ground speed
        function [vb, Cib] = to_body(b, p, vi, vg) 
            Cig = ECI.dcm(-b); % iso 1151
            Cib = ECI.dcm(-p);
            ve  = Cig * vg;
            vb  = Cib' * (vi-ve);
        end
        function v = vt(pc, vc)
            l = atan2(pc(2), pc(1));  
            b = l + pi/2;
            s = sin(b);
            c = cos(b);
            v = [c  s] * vc;
        end 
        function C = dcm(p)
            s = sin(p);
            c = cos(p);
            C = [c  s 
                -s  c];
        end        
    end
end


