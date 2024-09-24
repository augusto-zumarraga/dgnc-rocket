% x = { pe[3], ve[3], qeb[4], wb(3), m, act(3)}
%
%    pe : posición ECEF
%    ve : velocidad ECEF
%    qeb: cuaternion de actitud body -> ECEF
%    wb : velocidad angular inercial en terna body 
%    m  : masa
%    tvc: deflexiones {y,z} normalizadas del TVC
%    alr: deflexiones {y,z} normalizadas del TVC
%
function xd = rocket_xdot(t, x, cmd, param, data)

    pc  = x(1:3);
    vc  = x(4:6);
    eci = ECI(param.launch.ut1 + t);
    [pe, ve] = eci.to_ecef(pc, vc);
    [nav.lat, nav.lng, nav.hgt] = WGS84.ecef_to_lgv(pe);
    if nav.hgt > 0 && nav.hgt < 1e6
        qib   = quaternion(x(7:10));
        qeb   = eci.ecef_att(qib); 
        wb    = x(11:13);
        m     = x(14); 
        act   = x(15:17);
        u.tvc = act(1:2);   % posición del TVC normalizada
        u.alr = act(3);     % posición alerón normalizada
       
        nav.t = t;
        [fb, mb, mdot] = sfc(nav, ve, wb, qeb', m, u, data, cmd(4));
        fc = qib.trnsf(fb);
        
        ge = WGS84.gravitation(pe);   
        gc = eci.to_eci(ge);
       %gc = eci.gravitation(pc);        
        
        qdot = qib.angular_velocity_transform() * wb;        
        d_act = (cmd(1:3) - act) .* data.w_act;
        xd = [vc 
              fc + gc 
              qdot(1:4)   
              mb
             -mdot
              d_act];        
    else
        xd = zeros(size(x));
    end

end


