function export_wind(wnd, csv)
    wind = [wnd.h wnd.u wnd.v wnd.w];
    save(csv, 'wind', '-ascii');
end

