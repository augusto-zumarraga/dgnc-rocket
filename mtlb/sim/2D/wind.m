% Cómputo de velocidad relativa respecto del aire
%
% dca: desplazamiento del CG respecto de REF (+ si el CG está más a proa) 
% vb : velocidad relativa respecto del aire en REF
function vb = wind(h, vn, dcm_bn, param)
     
    if isfield(param, 'wind')
        uw = interp1(param.wind.h, param.wind.u, h, 'pchip','extrap');  
        vn(1) = vn(1) - uw;
    end        
    vb = dcm_bn * vn;   
end

