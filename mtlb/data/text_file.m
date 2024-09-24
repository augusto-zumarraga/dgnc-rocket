classdef text_file < handle

    properties (Access = private)     
        m_desc;
        m_line;
    	m_name;
        m_size;
        m_lines;
        m_index;
    end    
    methods
        function this = text_file(fname, fpath, fext)
            this.m_lines = 0;
            this.m_size  = 0;
            if nargin < 3
                fext  = [];
            end
            if nargin < 2
                fpath = [];
            end
            if nargin < 1
                this.m_name = [];
                this.m_desc = 0;
                this.m_line = 0;
            else
                this.open(fname, fpath, fext);
            end
        end
        function err = open(this, fname, fpath, fext)
            if nargin > 2
                fname = strcat(fpath, '/', fname);
            end
            if nargin > 3
                fname = strcat(fname, fext);
            end            
            this.m_name = fname;
            this.m_desc = fopen(fname,'r');
            if this.m_desc == -1
                this.m_line = 0;
                err = true;
            else
                this.m_line = 1;
                err = false;
                fseek(this.m_desc, 0, 'eof');
                this.m_size = ftell(this.m_desc);
                frewind(this.m_desc);
                % Read the whole file.
                data = fread(this.m_desc, this.m_size, 'uint8');
                % Count number of line-feeds and increase by one.
                data = (data == 10);
                this.m_index = [1 ; find(data)+1];
                this.m_lines = sum(data) + 1;
                frewind(this.m_desc);
            end
        end
        function close(this)
            fclose(this.m_desc);
            this.m_desc = 0;
            this.m_line = 0;
        end
        function rewind(this)
            frewind(this.m_desc);
            this.m_line = 1;
        end
        function goto_line(this, n)
           if   n < 1
                n = 1;
            elseif n >= this.m_lines
                n = this.m_lines;
            end
            this.m_line = n;
            fseek(this.m_desc, this.m_index(this.m_line), 'bof');
        end        
        % qty: cantidad de líneas a saltear (puede ser negativo)
        function skip_line(this, qty)
            if nargin < 2
                qty = 1;
            end
            goto_line(this, this.m_line + qty);
        end
        % n_disc: caracteres a descartar del inicio 
        function txt = pop_line(this, n_disc)  
            txt = [];
            c_cmnt = [];
            if nargin < 2
                n_disc = [];
            elseif ischar(n_disc)
                c_cmnt = n_disc;
                n_disc = [];
            end
            while ~feof(this.m_desc) && isempty(txt)
                txt = fgetl(this.m_desc); 
                this.m_line = this.m_line + 1;
                if ~isempty(n_disc)
                    if n_disc > length(txt)
                        n_disc = length(txt);
                    end
                    txt(1:n_disc) = [];
                end
                if ~isempty(c_cmnt)
                    h = strfind(txt, c_cmnt); 
                    if h ~= 0
                        txt(h:end) = [];
                    end
                end
            end
        end
        %------------------------------------------------------------------
        function r = is_open(this)
            r = this.m_desc > 0;
        end
        function r = is_good(this)
            r = (this.m_desc > 0) && ~feof(this.m_desc);
        end
        function r = is_bad(this)
            r = (this.m_desc == -1);
        end
        function r = eof(this)
            r = feof(this.m_desc);
        end
        function r = size(this)
            r = this.m_size;
        end
        function r = name(this)
            r = this.m_name;
        end
        function r = line(this)
            r = this.m_line;
        end
        function r = lines(this)
            r = this.m_lines;
        end    
    end
%     methods (Access = private)        
%         
% 
%     end
end

