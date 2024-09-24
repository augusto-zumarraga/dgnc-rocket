function c = interp_M(coef, CF, aa, bb, idx, frc)
    if isempty(frc)
        CF = squeeze(CF(idx,:,:));
    else
        CF = squeeze(CF(idx,:,:)*(1-frc) + CF(idx+1,:,:)*frc);
    end
    c = interp2(coef.beta, coef.alpha, CF, bb, aa, 'spline');
end


