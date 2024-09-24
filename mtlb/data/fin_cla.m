% cómputo de la pendiente CL vs alfa para una aleta de bajo alargamiento
function a = fin_cla(M, b, S, a0)

    if nargin < 4
        a0 = 2*pi;
    end
    AR = b^2/S;

    %% subsónico
    M1 = M(M <= 1);    
    M2 = M(M >  1);
    
    PG = 1 - M1.^2;
    apr = a0/(pi*AR);
    a1 = a0 ./ (sqrt(PG + apr^2) + apr);
    
    % supersónico
    PG = sqrt(M2.^2 - 1);
    sw = 1./(2*AR*PG); % esto debería ir multiplicado por ep taper ratio l = ct/cr?
    a2 = 4./PG .* (1 - sw);
    
    %k = find(a2 > 0, 1); %= a1(end), 1);
    k = find(a2 >= a1(end), 1);
    a2(1:k) = interp1([1 M2(k)], [a1(end) a2(k)], M2(1:k));
    
%    sw = sw .* (sw > )
   
%     r2 = find(sw < 0.8);
%     M2 = M2(r2);
%     sw = sw(r2);
%     PG = PG(r2);
    
   
    a = [a1 a2];
    
end