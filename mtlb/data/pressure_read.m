function p = pressure_read(fpath, fname)

    if nargin < 2 || isempty(fname)
        fname = "for010";
    end
    inp = text_file(fname, fpath, '.dat');
    if inp.is_bad()
        error('No se pudo abrir el archivo');
    end
    
    index = build_index(inp);
    for k=1:length(index)-1
        data = parse_pressures(inp, index(k).line, index(k+1).line-1);
        p(k).MACH  = index(k).MACH ;  
        p(k).ALPHA = index(k).ALPHA;  
        p(k).X_D   = data(:,1);
        p(k).CP    = data(:,2:end);
    end
end


function index = build_index(inp) 
    k = 1;
    while ~inp.eof()
        txt = inp.pop_line();
        % ZONE T="BODY CP AT MACH=  1.30 ALPHA=  0.00"
        h = strfind(txt, 'ZONE T="BODY CP AT MACH='); 
        if h ~= 0
            index(k).line  = inp.line;
            index(k).MACH  = parse_val(txt, "MACH");  
            index(k).ALPHA = parse_val(txt, "ALPHA");  
            k = k + 1;
        end
    end
end
function txt = find_val(txt, tag)  
    h = strfind(txt, tag); 
    if h ~= 0
        h = h + strlength(tag);
        txt = txt(h:end);
        h = strfind(txt, '='); 
        assert(~isempty(h));
        h = h + 1; %strlength(tag) + 1;
        txt = txt(h:end);
    else
        txt = [];
    end
end
function [val,b] = parse_val(txt, name) 
    txt = find_val(txt,name);
    assert(~isempty(txt));
    [val,~,~,b] = sscanf(txt,'%f');
    b = b + 1;
end
% VARIABLES=X/D,CP(0),CP(30),CP(60),CP(90),CP(120),CP(150),CP(180)
function press = parse_pressures(inp, a, b) 
    press = zeros(b-a,8);
    inp.goto_line(a);
    for k=1:length(press)
        txt = inp.pop_line();
        c = sscanf(txt,'%f'); 
        press(k,:) = c';
    end
end

