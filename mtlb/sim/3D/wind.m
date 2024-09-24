% Cómputo de velocidad lineal y angular relativa respecto del aire
%
% dca: desplazamiento del CG respecto de REF (+ si el CG está más a proa) 
% vb : velocidad relativa respecto del aire en REF
function [vb, wb] = wind(h, ve, wb, qbe, qen, param)
     
    if isfield(param, 'wind')
        u  = interp1(param.wind.h, param.wind.u, h, 'pchip','extrap');  
        v  = interp1(param.wind.h, param.wind.v, h, 'pchip','extrap'); 
        w  = interp1(param.wind.h, param.wind.w, h, 'pchip','extrap'); 
        vw = qen.trnsf([u ; v ;-w]);
        vw = ve - vw;
    else
        vw = ve;
    end        
    % Para el viento la terna de referencia es ECEF
    % velocidad ECEF proyectada en terna b 
    vb = qbe.trnsf(vw);
    % wb es una velocidad angular inercial. 
    % Le quitamos la velocidad angular de la terna ECEF
    wb = wb - qbe.trnsf(WGS84.we); 
end

