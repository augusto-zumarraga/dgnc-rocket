function wind = wind_model_1(qen, nfig)
    if nargin < 1
        qen = [];
    end
    if nargin < 2
        nfig = 1;
    end
    HW = [  10  18 325
           100  27 320
          1500  46 290
          3500  64 290
          5000 110 290
         10500 168 295
         17500  57 275
         26500 116 290];
    HW(:,2) = HW(:,2) / 3.6;
    
    wind = build_wind([], HW, [], qen, nfig);

end

