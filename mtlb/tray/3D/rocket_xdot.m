% x = { pc[3], vc[3], qbi[4], m }
% 
% pc : posición ECI
% vc : velocidad ECI
% qbi: cuaternion de actitud {ECI}->{bdy}
% m  : masa
%
function xd = rocket_xdot(t, x, param, maneouver)

    pc  = x(1:3);
    vc  = x(4:6);
    eci = ECI(param.launch.ut1 + t);
    pe  = eci.ecef_pos(pc);
    [nav.lat, nav.lng, nav.hgt] = WGS84.ecef_to_lgv(pe);
    if nav.hgt > 0 && nav.hgt < 1e6
        qbi = quaternion(x(7:10));
        qib = qbi';
       %qeb = eci.ecef_att(qib); 
        [~, qnb] = sphere.to_ned(pc, vc, qib);     
        
        ve  = ECI.ground_speed(pc);
        vb  = qbi.trnsf(vc - ve); 
        %% fuerzas
        [F, aer.po, aer.V, aer.a, aer.b] = aero(nav.hgt, vb, param);
        [T, mdot] = thrust(t, aer.po, param);

        %% navegación
        nav = nvc(pc, vc, qbi, param.orb);      

        %% ================================================================
        %  ωb: velocidad angular en terna móvil
        [wb, eng] = fcc(t, aer, nav, qnb, T, param, maneouver);  
        if eng
            F(1) = F(1)+T;
        end
        m  = x(11);
        fb = F/m;
        fc = qib.trnsf(fb); 
        gc = ECI.gravitation(pc);

        %fc = qib.trnsf(fb);
        qdot = qib.angular_velocity_transform() * wb;
        
        %% x = { p_î, v_î, qbi }
        %  u = { fb, wib }
        %  f = dx/dt
        
        xd = [vc ; fc + gc ; qdot(1:4) ; -mdot];
    else
        xd = zeros(size(x));
    end
end



