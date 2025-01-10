function [q_pos,q_vel,q_acc,q_jer] = p2pTrjTrap1DOF(t,T,Pos,c)

    n = length(T)-1;  % n = number of trajectories
    
    for j = n+1:-1:2
        if t <= T(j)
            i = j-1;
        end
    end
    
    qi = Pos(i);
    qf = Pos(i+1);
    ti = t-T(i);
    tf = T(i+1)-T(i);
    
    D = qf - qi;
    signD = sign(D);

    ta = tf/(2+c);
    kv = abs(D)/((c+1)*ta);
    ka = abs(D)/((c+1)*ta^2);
%     if k == 1
%         fprintf('qi: %1.2f, qf: %1.2f, ti: %1.2f, ta: %1.2f, tf: %1.2f, kv: %1.2f, ka: %1.2f, \n',rad2deg(qi),rad2deg(qf),ti,ta,tf,kv,ka);
%     end
    
    if (ti <= ta)
        q_pos = qi + 0.5*ti^2*ka*signD;
        q_vel = ti*ka*signD;
        q_acc = ka*signD;
        q_jer = 0.0;
    elseif (ti <= tf - ta)
        q_pos = qi + (ti-0.5*ta)*kv*signD;
        q_vel = kv*signD;
        q_acc = 0.0;
        q_jer = 0.0;
    else
        q_pos = qf - 0.5*(tf-ti)^2*ka*signD;
        q_vel = (tf-ti)*ka*signD;
        q_acc = -ka*signD;
        q_jer = 0.0;
    end

end