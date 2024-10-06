function f = show_flight(fname, nfig, do_import)
    
    if nargin < 2
        nfig = [1 2 3 4];
    else
        if length(nfig) == 1
            nfig = [0 1 2 3] + nfig;
        end
    end    
    fpath = '../../../eclipse/fsim/rec/';
    fmat  = strcat(fpath, fname, '_sim.mat'); 
    if nargin < 3 || isempty(do_import) 
        do_import = false;
    if ~do_import 
        l_mat = dir(fmat);
        if(isempty(l_mat))
            do_import = true;
        else
            fexp  = strcat(fpath, fname, '_sim.csv'); 
            l_exp = dir(fexp);
            do_import = l_exp.datenum > l_mat.datenum;
        end
    end
    if do_import
        f = import_flight(fname, fpath, fmat); 
    else
        f = load(fmat); 
    end
    rng = find(f.frc.T > 0, 1, 'first'):find(f.frc.T > 0, 1, 'last');    
    if nfig(1)
        plot_tray(f, [], [], [], nfig(1)); 
    end
    if nfig(2)
        plot_gnc (f, [], [], nfig(2), rng); 
    end
    if nfig(3)
        plot_ctrl_s1(f, nfig(3)); 
    end 
    if nfig(4)
        plot_ctrl_s2(f, nfig(4)); 
    end
    if nargout == 0
        f = [];
    end
end

