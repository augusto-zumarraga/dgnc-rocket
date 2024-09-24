function [ep, ey] = att_err(qeb, nav, ref)
    loc  = local_geo(nav.lat, nav.lng);
    qnow = loc.ecef_to_ned(qeb);
    qref = quaternion(ref);
    qerr = qref' * qnow;
    ep = qerr.pitch;
    ey = qerr.yaw;
end

