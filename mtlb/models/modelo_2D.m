%% Argumentos
%
% cnd       : m, Iy, v (velocidad), p (pitch)
% drv       : .Zw, .Zq, .Zad, .Mw, .Mq, .Mad (dimensionales)
% df        : distancia del acelerómetro al CG
% alfa_state: si es true el primer estado en el modelo (mdl) se convierte a 
%             ángulo de ataque
%
% mdl : modelo de estados
%
% Matrices individuales del modelo para estados = {'w' 'q' 'θ'}
%     M : matriz de inercia
%     Aa: derivativas aerodinámicas
%     Ai: derivativas inerciales
%     Bu: derivativas de la perturbación atmosférica
%
function [mdl, M, Aa, Ai, Bu] = modelo_2D(cnd, drv, df, alfa_state)

    if nargin < 3 
        df = [];
    end
    if nargin < 4 || isempty(alfa_state)
        alfa_state = true;
    end
    Aa = [drv.Zw  drv.Zq  0
          drv.Mw  drv.Mq  0
          0        0      0]; 
    zq =  cnd.m*cnd.v;
    zp = -cnd.m*cnd.g*sin(cnd.p);
    Ai = [0  zq zp   
          0  0  0 
          0  1  0]; 
    A  = Ai + Aa; 
    Bu = [drv.Zu
          drv.Mu
          0];
    M  = [cnd.m-drv.Zwd 0       0
         -drv.Mwd       cnd.Iy  0
          0             0       1]; 
    %%
    A = M\A; 
    B = M\Bu; 
    C = [1 0 0 
         0 0 1 
         0 1 0];
    % inp: 1: { elev thr }, 2: { uw ww qw }, otro: { elev thr uw ww qw },
    D = [0; 0; 0];
    estados = {'w' 'q' 'θ'}; 
    salidas = {'w' 'θ' 'q'};
    if ~isempty(df)
        C = [C ; Aa(1,:)-df*A(2,:)];
        D = [D ;  B(1,:)-df*B(2,:)];
        salidas{4}  = 'f';
    end
    if cnd.v > 0 && alfa_state
        estados{1} = 'α'; 
        salidas{1} = 'α';
        T = diag([cnd.v 1 1]);
        A = T\A*T;
        B = T\B;
        C = C*T; C(1,1) = 1;
    end
    mdl = ss(A, B, C, D, 'InputName', 'ν', 'OutputName', salidas, 'StateName', estados);
end


        
