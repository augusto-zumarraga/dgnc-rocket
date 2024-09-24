% flx 
%   (ver elastic_model)
%
% aero
%   .rho
%   .speed 
%
% rigid (x = {'w' 'q' 'θ'})
%   .M
%   .Aa
%   .Ai  
%
function [Ar, Ae, Are, Aer, Ae_, Me, De, Ze, mw] = aero_elastic(flx, aero, rigid)

    [Iz, Iq, Ie, mw] = elastic_model(flx, aero);

    h = aero.rho*aero.speed;
    Ze = h*Iz;
    Me = h*Iq;
    De = h*Ie;
    lk = flx.w1^2;

    Ar  = rigid.M\(rigid.Aa+rigid.Ai); 

    Are = zeros(3,2); 
    Are(1,2) = Ze; 
    Are(2,2) = Me; 
    Are = rigid.M\Are;  

    Aer = zeros(2,3); 
    Aer(2,2) = Ze/mw;

    Ae_ = [ 0    1
           -lk  -2*0.02*sqrt(lk)];
    Ae  = [ 0    1 
           -lk   Ae_(2,2)-De/mw];
   
end

