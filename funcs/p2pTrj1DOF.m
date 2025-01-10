function [q_pos,q_vel,q_acc,q_jer] = p2pTrj1DOF(t,T,Pos)

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

    s_pos = ti/tf;
    s_vel = 1/tf;
    s_acc = 0;
    s_jer = 0;
    
    q_pos = qi + s_pos*D;
    q_vel = s_vel*D;
    q_acc = s_acc*D;
    q_jer = s_jer*D;
    
%     switch trj_profile
%         case 1
%             s_pos = t/tf;
%             s_vel = 1/tf;
%             s_acc = 0;
%         case 2
%             s_pos = 3*(t/tf)^2 - 2*(t/tf)^3;
%             s_vel = 6*(t/tf^2) - 6*(t^2/tf^3);
%             s_acc = 6/tf^2 - 12*(t/tf^3);
%         case 3
%             s_pos = 10*(t/tf)^3 - 15*(t/tf)^4 + 6*(t/tf)^5;
%             s_vel = 30*(t^2/tf^3) - 60*(t^3/tf^4) + 30*(t^4/tf^5);
%             s_acc = 60*(t/tf^3) - 180*(t^2/tf^4) + 120*(t^3/tf^5);
%         case 4
%             if t <= tf/2
%                 s_pos = 2*(t/tf)^2;
%                 s_vel = 4*(t/tf^2);
%                 s_acc = 4/tf^2;
%             else
%                 s_pos = -1 + 4*(t/tf) - 2*(t/tf)^2;
%                 s_vel = 4/tf - 4*(t/tf^2);
%                 s_acc = -4/tf^2;
%             end
%         otherwise
%             s_pos = 0;
%             s_vel = 0;
%             s_acc = 0;
%     end

end