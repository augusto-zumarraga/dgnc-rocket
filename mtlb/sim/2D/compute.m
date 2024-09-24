function out = compute(t, x, param)    
    
    Z1 = zeros(size(t));
    
    out.t   = t;
    out.r   = x(:,1); 
    out.h   = x(:,2);
    out.pn  = [Z1 out.r out.h];
    out.vn  = [x(:,3) Z1 x(:,4)];
    out.y   = atan2(-x(:,4), x(:,3));  
    out.p   = mod(x(:,5), 2*pi);
    out.eul = [Z1 out.p  Z1];
    out.wb  = [Z1 x(:,6) Z1];
    out.m   = x(:,7);
    out.cmd = [x(:,8) Z1 Z1];
    
    
    out.a = Z1;
    out.b = Z1;
    out.v = Z1;
    out.Q = Z1;
    out.M = Z1;
    out.T = Z1;

    Z3 = zeros(length(t),3);
    out.fb = Z3;
    out.mb = Z3;
    
    for k=1:length(t)
       
        nav.t = t(k);
        nav.h = out.h(k);
        dcm_bn = att(out.p(k));
        [fb, mb, ~, T, air] = sfc(nav, out.vn(k,[1 3])', dcm_bn, out.wb(k,2), out.m(k), out.cmd(k,1), param);
        out.a (k) = air.a;
        out.v (k) = sqrt(2*air.Q/air.ro);
        out.M (k) = air.M;
        out.Q (k) = air.Q;
        out.T (k) = T;
        
        out.fb(k,:) = [fb(1) 0  fb(2)];
        out.mb(k,:) = [0     mb 0];
        
    end
end

% proyección n → b        
function dcm_bn = att(p)
    sp  = sin(p);
    cp  = cos(p);
    
    dcm_bn = [cp -sp 
              sp  cp];  
end




