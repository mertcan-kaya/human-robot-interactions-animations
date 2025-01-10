function Cj = minimumJerkCoefficient1DOF(T,Pos,Vel,Acc)

    n = length(T)-1;  % n = number of trajectories 

    % Waypoints' Matrix
    R = zeros(6*n,6*n); % Start with zero matrix for R, joint
    
    % Positions of all waypoints
    for i = 1:n
        R(1+2*(i-1),1+6*(i-1):6+6*(i-1)) = [T(i)^5 T(i)^4 T(i)^3 T(i)^2 T(i)^1 1];
        R(2+2*(i-1),1+6*(i-1):6+6*(i-1)) = [T(i+1)^5 T(i+1)^4 T(i+1)^3 T(i+1)^2 T(i+1)^1 1];
    end
    % Velocity boundary conditions (inital and final waypoints)        
    R(((1+2*n):(1+2*n)),(1:6))               = [  5*T(1)^4   4*T(1)^3   3*T(1)^2   2*T(1) 1 0];
    R(((2+2*n):(2+2*n)),((1+6*(n-1)):(6*n))) = [5*T(n+1)^4 4*T(n+1)^3 3*T(n+1)^2 2*T(n+1) 1 0];
    % Equal Accelaration boundary conditions (initial and final waypoints)
    R(((3+2*n):(3+2*n)),(1:6))               = [  20*T(1)^3   12*T(1)^2   6*T(1)^1 2 0 0];
    R(((4+2*n):(4+2*n)),((1+6*(n-1)):(6*n))) = [20*T(n+1)^3 12*T(n+1)^2 6*T(n+1)^1 2 0 0];
    %Equal velocity, accelaration , jerk, and snap at intermideate waypoints
    for i = 1:n-1
        R(((1+(i-1)+(0*(n-1))+4+(2*n)):(1+(i-1)+(0*(n-1))+4+(2*n))),(1+(6*(i-1)):6*(i+1))) = [  5*T(i+1)^4  4*T(i+1)^3 3*T(i+1)^2 2*T(i+1)^1 1 0   -5*T(i+1)^4  -4*T(i+1)^3 -3*T(i+1)^2 -2*T(i+1)^1 -1 0]; % Equal velocity at intermediate waypoints
        R(((1+(i-1)+(1*(n-1))+4+(2*n)):(1+(i-1)+(1*(n-1))+4+(2*n))),(1+(6*(i-1)):6*(i+1))) = [ 20*T(i+1)^3 12*T(i+1)^2 6*T(i+1)^1          2 0 0  -20*T(i+1)^3 -12*T(i+1)^2 -6*T(i+1)^1          -2  0 0]; % Equal acceleration at intermediate waypoints
        R(((1+(i-1)+(2*(n-1))+4+(2*n)):(1+(i-1)+(2*(n-1))+4+(2*n))),(1+(6*(i-1)):6*(i+1))) = [ 60*T(i+1)^2 24*T(i+1)^1          6          0 0 0  -60*T(i+1)^2 -24*T(i+1)^1          -6           0  0 0]; % Equal jerk at intermediate waypoints
        R(((1+(i-1)+(3*(n-1))+4+(2*n)):(1+(i-1)+(3*(n-1))+4+(2*n))),(1+(6*(i-1)):6*(i+1))) = [120*T(i+1)^1          24          0          0 0 0 -120*T(i+1)^1          -24           0           0  0 0]; % Equal snap at intermediate waypoints
    end
    
    % Boundary Conditions Matrix
    BC = zeros(6*n,1);
    BC(1,1) = Pos(1);     % Position of the first waypoint
    if n > 1
        PosInter = zeros(2*(n-1),1);
    end
    for i = 2:n
        PosInter(2*(i-2)+1:2*(i-2)+2) = [Pos(i);Pos(i)];
    end
    for i = 1:2*(n-1)
        BC(2+(i-1)) = PosInter(i);
    end
    BC(2+2*(n-1)) = Pos(n+1);   % Position of the final waypoint
    BC(3+2*(n-1)) = Vel(1);     % initial velocity
    BC(4+2*(n-1)) = Vel(2);     % final velocity
    BC(5+2*(n-1)) = Acc(1);     % initial acceleration
    BC(6+2*(n-1)) = Acc(2);     % final acceleration

    % Coefficient  Vector
    Cj = R\BC;
    
end