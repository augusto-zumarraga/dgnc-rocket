% 17/09/2023
function wind = wind_model_3(qen, nfig)
    if nargin < 1
        qen = [];
    end
    if nargin < 2
        nfig = 3;
    end
    HW = [  0    0   0
           10   13  95
          100   17  95
          600   24  80
          900   21  60
         1500    8  45
         3000   23  330
         5500   65  290
         7000   79  320
        10000   97  300
        13500   86  280];
    HW(:,2) = HW(:,2) * 1.852 / 3.6;
    
    wind = build_wind([], HW, [], qen, nfig);

end

