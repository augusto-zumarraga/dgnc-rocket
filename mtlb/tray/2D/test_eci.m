clear
clc

tol = 1e-6;

%%
R_ = WGS84.a + 100;
pc = [R_; 0];
vi = [10; 0];
qc = 0;

    [h, R, b, p, y] = eci.to_geo(pc, vi, qc);
    assert(abs(h - 100 ) < tol);
    assert(abs(R - R_  ) < tol);
    assert(abs(b - pi/2) < tol);
    assert(abs(p - pi/2) < tol);
    assert(abs(y - pi/2) < tol);
    

    %    [vb, Cib] = eci.to_body(b, q, vi, [R*param.we;0]);

        
%%
pc = [1;1]*cosd(45)*R_;
vi = [0;10];
qc = pi/2;
   
    [h, R, b, p, y] = eci.to_geo(pc, vi, qc);
    assert(abs(h - 100) < tol);
    assert(abs(R - R_ ) < tol);
    assert(abs(b - pi*(1/2+1/4)) < tol);
    assert(abs(p - pi/4) < tol);
    assert(abs(y - pi/4) < tol);


%%
pc = [R_ 0
      pc'];
vc = [10 0
      vi'];
qc = [0
      qc];
we = 0;

    [h, r, p, y, a, g, R, vb] = eci.geo_map(pc, vc, qc, we);
    
    assert(abs(h(1) - 100) < tol);
    assert(abs(h(2) - 100) < tol);
    assert(abs(R(1) - R_ ) < tol);
    assert(abs(R(2) - R_ ) < tol);
    
    assert(abs(p(1) - pi/2) < tol);
    assert(abs(p(2) - pi/4) < tol);
    
    assert(abs(y(1) - pi/2) < tol);
    assert(abs(y(2) - pi/4) < tol);

        
        