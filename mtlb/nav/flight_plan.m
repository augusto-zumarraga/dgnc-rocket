% mod[]: 0: α = 0 
%        1: pitch over (θ → ref) 
%        2: constant pitch (θ = ref)
%        3: prescribed gravity turn  
%        4: steering  
%        5: insertion  
% hgt[]: altura de inicio (0 implica "hasta volver al piso")
% ref[]: valor de referencia. El "pitch over" se termina al alcanzar este
% valor, mientras que para "constat pitch" es el ángulo a mantener.
function fp = flight_plan(mod, hgt, ref)  

    field_1 = 'mode';
    field_2 = 'start';
    field_3 = 'ref';

    fp = struct( field_1, mod ...
               , field_2, hgt ...
               , field_3, ref);

end

