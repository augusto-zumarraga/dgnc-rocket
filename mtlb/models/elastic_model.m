% flx
%   .X : secciones
%   .S : Área de la sección 
%   .mu: densidad lineal en cada sección 
%   .Y1: primer forma modal
%   .w1: frecuencia de la primera forma modal
%
% Iz 
% Iq: Iθ
% In: Iη
function [Iz, Iq, In, mw] = elastic_model(flx)

    Cn = deriv(flx.X, flx.S);
    f  = 4 * pi/180;   % aproximación para la contribución viscosa
    Cn = Cn + flx.D*f;

    Iz =-trapz(flx.X, Cn.*flx.Y1);        % Iz
    Iq = trapz(flx.X, Cn.*flx.Y1.*flx.X); % Iθ
    In =-trapz(flx.X, Cn.*(flx.Y1.^2));   % Iη
   
    % masa generalizada
    mw = trapz(flx.X, flx.mu.*(flx.Y1.^2));     
end

% ver derivada_parabolica.m en core/numeric
function D = deriv(X, Y)

    assert(length(X) == size(Y,2));
    D = zeros(size(Y));
   
    D(:,1) = diff2(X(1), X(2), X(3), Y(:,1), Y(:,2), Y(:,3), X(1));
    for c=2:length(X)-1
        D(:,c) = diff2(X(c-1), X(c), X(c+1), Y(:,c-1), Y(:,c), Y(:,c+1), X(c));
    end
    D(:,end) = diff2(X(end-2), X(end-1), X(end), Y(:,end-2), Y(:,end-1), Y(:,end), X(end));
   
end

function d = diff2(x1, x2, x3, y1, y2, y3, v)
    dn  = (x1-x2).*(x1-x3).*(x2-x3);
    d12 = y1-y2;
    d23 = y2-y3;
    d31 = y3-y1;
    b = (x1.^2.*d23 + x2.^2.*d31 + x3.^2.*d12)./dn;
    c =-(x1.*d23 + x2.*d31 + x3.*d12)./dn;
    d = b + 2*c .* v;
end
