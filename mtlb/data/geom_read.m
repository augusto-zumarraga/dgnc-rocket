function geom = geom_read(fpath, fname)

    % Abrir Archivo "for009.dat"
    if nargin < 2 || isempty(fname)
        fname = "for009";
    end
    inp = text_file(fname, fpath, '.dat');
    if inp.is_bad()
        error('No se pudo abrir el archivo');
    end
    
    index = build_index(inp);
    f2m = 12 * 0.0254;
    for k=1:length(index)-1
        G = parse_geom(inp, index(k), index(k+1)-1)*f2m;
        geom(k).X = G(:,1);
        geom(k).R = G(:,2);
    end

end


function index = build_index(inp) 
    k = 1;
    while ~inp.eof()
        txt = inp.pop_line();
        h = strfind(txt, 'ZONE T="BODY GEOMETRY IN FEET"'); 
        if h ~= 0
            index(k) = inp.line;
            k = k + 1;
        end
    end
end
function geom = parse_geom(inp, a, b) 
    geom = zeros(b-a,2);
    inp.goto_line(a);
    for k=1:length(geom)
        txt = inp.pop_line();
        c = sscanf(txt,'%f'); 
        geom(k,:) = c';
    end
end

