% modelo reducido para la dinamica del angulo de ataque
function mdl = modelo_alfa(mdl)
     A = mdl.A(1:2, 1:2);
     B = mdl.B(1:2);
     C = mdl.C(1:2,1:2);
     D = mdl.D(1:2,:);
     mdl = ss(A, B, C, D ...
             , 'InputName' ,  mdl.InputName ...
             , 'OutputName', {mdl.OutputName{1} mdl.OutputName{2}} ...
             , 'StateName' , {mdl.StateName{1} mdl.StateName{2}});
end

        
