function [ep, ey] = att_err(qnow, qref)
    qerr = qref' * qnow;
    ep = qerr.pitch;
    ey = qerr.yaw;
end

