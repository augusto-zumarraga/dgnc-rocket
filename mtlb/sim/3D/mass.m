% PROPIEDADES MASICAS
%
% dca: desplazamiento del CG respecto de X_REF (+ si el CG está más a popa) 
% dgm: brazo de palanca del TVC 
% J  : tensor de inercia
function [xcg, J] = mass(m, data)
    xcg = interp1(data.m, data.xcg, m, 'pchip','extrap');
    Ix  = interp1(data.m, data.Ix , m, 'pchip','extrap');  
    Iy  = interp1(data.m, data.Iy , m, 'pchip','extrap');  
    Iz  = interp1(data.m, data.Iz , m, 'pchip','extrap');  
    J   = diag([Ix Iy Iz]);
end

