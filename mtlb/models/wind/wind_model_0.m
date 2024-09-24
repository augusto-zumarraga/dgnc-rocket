function wind = wind_model_0(qen, nfig)
    if nargin < 1
        qen = [];
    end
    if nargin < 2
        nfig = 1;
    end
    CL = [10 10 270];
    HW = [1000 20 180
          3000 20 170
          5000 0 160];
    GS = [1200 1400 10];
    
    wind = build_wind(CL, HW, GS, qen, nfig);

end

