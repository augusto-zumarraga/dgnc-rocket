%    pc : posición
%    vc : velocidad
%    qib: cuaternion de actitud
%
function nav = nvc(pc, vc, qbi, orb)

        nav.R = sqrt(pc'*pc);       % radio geocéntrico 
        nav.h = nav.R - WGS84.a;
        nav.g = WGS84.ge * (WGS84.a/nav.R)^2;
        nav.V = sqrt(vc'*vc);       % módulo de la velocidad 
        
        orb.location(pc);
        nav.qtg = orb.tng_att(qbi);
        nav.vtg = orb.tng_vel(vc);
        po      = orb.orb_pos(pc);       
        nav.xte = po(3);
end

