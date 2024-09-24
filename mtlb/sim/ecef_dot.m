%% NAVEGACION
%
%  pe : posición ECEF
%  ve : veclocidad ECEF
%  qeb: cuaternión de actitud body → ECEF
%  fb : fuerza específica en terna body
%  wib: velocidad angular inercial en terna body
%
%  x = { pe, ve, qeb }
%  f = dx/dt
function f = ecef_dot(pe, ve, qeb, fb, wib)

    fe = qeb.trnsf(fb);
    
    ge = WGS84.gravity(pe); 
    ae = fe + ge;
    cr = WGS84.coriolis(ve);

    f = zeros(10,1);
    f(1:3) = ve;
    f(4:6) = ae + cr;
    
    qbe = qeb';
    wb  = wib - qbe.trnsf([0 0 WGS84.We]);
    
    qdot = qeb.angular_velocity_transform() * wb;
    f(7:10) = qdot(1:4)';
   
end


