% nav: [lat [⁰] , long [⁰], h [m]
% qnb: quaternión de actitud en terna BODY → NED
function [pe, qeb, loc] = launch_ecef(nav, qnb)

    d2r = pi/180;
    lat = nav(1)*d2r;
    lng = nav(2)*d2r;
    h   = nav(3);
    
    pe  = WGS84.lgv_to_ecef(lat, lng, h);

    loc = local_geo(lat, lng);
    qeb = loc.astro_ned_to_ecef(qnb); % ECEF → NED   
    
end

