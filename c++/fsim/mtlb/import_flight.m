function [f, param] = import_flight(froot, fpath, fmat)

    param  = [];
    
    if nargin < 2
        fpath = '../../../eclipse/fsim/rec/';
    end
    fname = strcat(fpath, froot);
    
	S = load(strcat(fname, '_sim.csv'), '-ascii');
	N = load(strcat(fname, '_nav.csv'), '-ascii');
	D = load(strcat(fname, '_dyn.csv'), '-ascii');
	G = load(strcat(fname, '_gnc.csv'), '-ascii');

    disp(sprintf('importing %s...', fname))

    d2r = pi/180;
    
	% nav_state_t
    k = 1;
    f.tk  = S(:,k    ); k = k + 1; 
    f.t   = S(:,k    ); k = k + 1; 
    f.pe  = S(:,k:k+2); k = k + 3; 
    f.ve  = S(:,k:k+2); k = k + 3; 
    f.qe  = S(:,k:k+3); k = k + 4;   
	% rot_state_t    
    f.wbi = S(:,k:k+2); k = k + 3; 
    % frc_state_t
    f.m   = S(:,k    ); k = k + 1; 
    f.act = S(:,k:k+5); k = k + 5;% tvc[2] / ail / rcs / flw
    f.flx.y = S(:,k  ); k = k + 1; 
    f.flx.q = S(:,k  ); k = k + 1; 
    f.flx.z = S(:,k  ); k = k + 1; 
    f.flx.r = S(:,k  );  

    k = 1;
    f.nav.t     = N(:,k);     k = k + 1; 
    f.nav.h     = N(:,k)*1e3; k = k + 1; 
    f.nav.r     = N(:,k)*1e3; k = k + 1; 
    f.nav.v     = N(:,k);     k = k + 1; 
    f.nav.gamma = N(:,k)*d2r; k = k + 1; 
    f.nav.beta  = N(:,k)*d2r; k = k + 1; 
    f.nav.lat   = N(:,k);     k = k + 1; 
    f.nav.lng   = N(:,k);     k = k + 1; 
    f.nav.pos   = N(:,k:k+2);
        

    k = 1;
    f.dyn.t    = D(:,k);         k = k + 1; 
   	f.nav.eul  = D(:,k:k+2)*d2r; k = k + 3; 
    f.dyn.dy   = D(:,k);         k = k + 1; 
    f.dyn.dz   = D(:,k);         k = k + 1; 
    f.dyn.da   = D(:,k);         k = k + 1;
    f.dyn.rc   = D(:,k);         k = k + 1;
    
    f.aer.Q     = D(:,k);     k = k + 1; 
    f.aer.M     = D(:,k);     k = k + 1; 
    f.aer.alpha = D(:,k)*d2r; k = k + 1; 
    f.aer.beta  = D(:,k)*d2r; k = k + 1; 
    
    f.frc.fb   = D(:,k:k+2); k = k + 3; 
    f.frc.T    = D(:,k);     %k = k + 1; 
    %f.dyn.mass = D(:,k); k = k + 1; 

    k = 1;
    f.gnc.t      = G(:,k    ); k = k + 1; 
   	f.gnc.r_dir  = G(:,k:k+2); k = k + 3; % referencia de dirección ECI
   	f.gnc.x_dir  = G(:,k:k+2); k = k + 3; % dirección del eje x ECI
   	f.gnc.e_dir  = G(:,k:k+2); k = k + 3; % desviación (body)
    f.gnc.e_nrm  = G(:,k)    ; k = k + 1; % norma de la desviación r_dir * x_dir [⁰]
   	f.gnc.q_err  = G(:,k:k+3); k = k + 4; % quternión de error de actitud (primera etapa) 
   	f.gnc.w_err  = G(:,k:k+2); k = k + 3; % error de velocidad angular
    f.gnc.g_st   = G(:,k    ); k = k + 1; % estado del PGUID 
    f.gnc.ttgo   = G(:,k    ); k = k + 1; % time to go
    f.gnc.rtgo   = G(:,k    ); k = k + 1; % range to go
    f.gnc.rbis   = G(:,k    );            % range bias

    if nargin > 2 && ~isempty(fmat)
        tk  = f.tk; 
        t   = f.t ; 
        p2  = f.pe; 
        ve  = f.ve; 
        qe  = f.qe;   
        m   = f.m ; 
        act = f.act;
        gnc = f.gnc;
        dyn = f.dyn;
        nav = f.nav;
        aer = f.aer;
        frc = f.frc;
        flx = f.flx;
	    save(fmat, 'tk', 't', 'p2', 've', 'qe', 'm' ...,
                 , 'act', 'gnc', 'dyn', 'nav', 'aer', 'frc', 'flx');
        disp(sprintf('saving to %s...', fmat))
    end
    disp('...done')

end