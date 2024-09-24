function [idx, frc] = find_index(coef, V)
    if V <= coef(1)
        idx = 1;    
        frc = [];
    elseif V >= coef(end)
        idx = length(coef);    
        frc = [];
    else
        idx = floor(interp1(coef, 1:length(coef), V));
        V1  = coef(idx  );
        V2  = coef(idx+1);
        frc = (V-V1)/(V2-V1);
    end
end
