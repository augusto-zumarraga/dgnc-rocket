% mod[]: 0: α = 0 
%        1: pitch maneuver (θ → ref) 
%        2: constant pitch (θ = ref)
%        3: prescribed gravity turn  
%        4: height proportional steering  
%        5: linear tangent steering. Ref = [tf a b]
%        6: constant pitch insertion  
% tms[2]: {tf, dt} tiempo final, duración (si tf == 0) 
% ref   : valor o generador de referencia.  
% exp   : masa a expulsar
% evn   : función de terminación (para odeset('Event'...)
function fp = flight_phase(mod, tms, ref, exp, evn)  

    field_1 = 'mode';
    field_2 = 'times';
    field_3 = 'ref';
    field_4 = 'exp';
    field_5 = 'event';
    if nargin < 2
        tms = [];
    end
    if nargin < 3
        ref = [];
    end
    if nargin < 4
        exp = [];
    end
    if nargin < 5
        evn = [];
    end
    if ~isnumeric(mod)
        switch mod
            case 'load relief'
                mod = 0;
            case 'pitch maneuver'
                mod = 1;
            case 'constant pitch'
                mod = 2;
            case 'gravity turn'
                mod = 3;
            case 'height proportional steering'
                mod = 4;
            case 'linear tangent steering'
                mod = 5;
            case 'constant pitch insertion'
                mod = 6;
            case 'coasting'
                mod = 7;
            case 'constant pitch rate'
                mod = 8;
            otherwise
                error 'unkown mode'
        end
    end

    fp = struct( field_1, mod ...
               , field_2, tms ...
               , field_3, ref ...
               , field_4, exp ...
               , field_5, evn);
end

