% x = { r, h, vn[2], θ, ωy, m, tvc } 
%
% param
%   .t    : tiempo de vuelo
%   .mdot : dm/dt para cada t 
%   .ve   : velocidad de salida de la tobera 
%   .pe   : presión de salida de la tobera
%   .Ae   : área de salida de la tobera
%   .sref : superficie de referencia
%   .lref : longitud de referencia
%   .mach : rango de números de Mach
%
%           para cada número de mach:
%
%   .cx   : coeficiente de fuerza axial 
%   .cz   : derivada del coeficiente de fuerza normal con el ángulo de ataque 
%   .czq  : derivadativa de fuerza normal con la velocidad angular
%   .cma  : derivada del coeficiente de momento de cabeceo con el ángulo de 
%           ataque 
%   .cmq  : derivadativa del momento de cabeceo con la velocidad angular
%   .cme  : derivadativa del momento de cabeceo con la deflexión del
%   elevador
%
%   .mass : valores de masa total 
%   .Iy   : momentos de inercia
%   .x_ref: corrimiento de REF respecto del CG (> 0 si REF está más adelante 
%           que el CG).
%
%   r_tvc : comando para el tvc
function xd = rocket_xdot_f(t, x, r_tvc, param)

    nav.h  = x(2); % altura 
    if nav.h > 0
        nav.t = t;
        
        vn  = x(3:4); % velocidad en coordenadas de navegación
        p   = x(5);   % θ
        q   = x(6);   % dθ/dt
        m   = x(7);   % masa 
        tvc = x(8);   % posición normalizada del TVC  
        
        dcm_bn = att(p);
          
        [fb, mp, mdot] = sfc(nav, vn, dcm_bn, q, m, tvc, param);
        d_tvc = (r_tvc - tvc) * param.w_tvc;
        
        xd = [xz_dot(nav.h, vn, dcm_bn, q, fb, mp)
             -mdot
              d_tvc];
    else
        xd = zeros(size(x));
    end

end

function xd = xz_dot(h, vn, dcm_bn, q, fb, mp)

    g  = gravedad(h);
    fg = dcm_bn(:,2) * g;  %dcm_bn * [0 ; gravedad(h)];
    ab = fb + fg;  
    an = dcm_bn' *  ab;

    xd = [vn(1)
         -vn(2)
          an 
          q
          mp];

end
% proyección n → b        
function dcm_bn = att(p)
    %% Actitud
%     p = mod(p, 2*pi);
%     if p > pi
%         p = 2*pi - p;
%     elseif p < -pi
%         p = 2*pi + p;
%     end
    sp  = sin(p);
    cp  = cos(p);  
    dcm_bn = [cp -sp 
              sp  cp];  
end


