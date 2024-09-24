% 16/09/2023
function wind = wind_model_2(qen, nfig)
    if nargin < 1
        qen = [];
    end
    if nargin < 2
        nfig = 2;
    end
    HW = [ 10   9   290
          100   12  280
          600   16  270
          900   23  250
         1500   21  260
         3000   37  280
         5500   74  270
         7000   95  260
        10000   117 260
        13500   88  260];
    HW(:,2) = HW(:,2) * 1.852 / 3.6;
    
    wind = build_wind([], HW, [], qen, nfig);

end

