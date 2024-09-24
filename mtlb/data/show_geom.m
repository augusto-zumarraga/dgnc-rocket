% dibuja en la figura activa
function h = show_geom(fpath, fname, last, xtick, bdy, fin)
    if nargin < 6 
        fin = [];
    end
    if nargin < 5 
        bdy = [];
    end
    if nargin < 4 
        xtick = true;
    end
    if nargin < 2 
        fname = [];
    end
    GEOM = geom_read(fpath, fname);
    hold on;
    if nargin > 2 && last
        ko = size(GEOM,1);
    else
        ko = 1;
    end
    for k = ko:size(GEOM,1)
        
        L = squeeze(GEOM(k).X)';
        R = squeeze(GEOM(k).R)';
        
        h = max(L);
        
        R =    [-R(end:-1:1) R -R(end)]; 
        L = h -[ L(end:-1:1) L  L(end)];
        
        plot(R, L, 'k'); 
    end
    xlim([-1 1] * h / 4);
    ylim([-0.1 1.1] * h);
    if ~xtick
        set(gca,'xtick',[])
    else
        set(gca,'xtick',[-1 0 1])
    end
    grid on;
    daspect([1 1 1])
    
    if ~ isempty(bdy)    
        hold on;
        plot([-0.5 0.5]*bdy.DNOSE , [1 1] * (h - bdy.LNOSE), 'k'); 
        plot([-0.5 0.5]*bdy.DCENTR, [1 1] * (h - bdy.LNOSE-bdy.LCENTR), 'k'); 
    end    
    if ~ isempty(fin)
        hold on;
        for k=1:length(fin)
            plot_fin(fin(k), bdy, h);
        end
    end
end
function plot_fin(fin, bdy, h)
    if fin.SSPAN(1) == 0 && ~isempty(bdy)    
        fin.SSPAN = fin.SSPAN + 0.5*bdy.DCENTR;
    end
    b = fin.SSPAN(2) - fin.SSPAN(1);
    s = b * sind(fin.SWEEP);
    if isfield(fin, 'STA') && fin.STA == 1   % trailing edge sweep  
        s = s - fin.CHORD(2) + fin.CHORD(1);
    end
    R = [fin.SSPAN(1) fin.SSPAN(2) fin.SSPAN(2)   fin.SSPAN(1) fin.SSPAN(1)]; 
    L = [0            s            s+fin.CHORD(2) fin.CHORD(1) 0] + fin.XLE;
    L = h - L;
    plot( R, L, 'k'); 
    plot(-R, L, 'k'); 
end

