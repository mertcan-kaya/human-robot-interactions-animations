function Cj = minimumJerkConstrCoefficient1DOF(T,Pos,Vel,Acc)

    n = length(T)-1;  % n= number of trajectories 

    % Waypoints' Matrix
    R = zeros(6*n,6*n); % Start with zero matrix for R, joint
    
    % Positions of all waypoints
    for i = 1:n
        % Position
        R(2*i-1:2*i-1        ,6*i-5:6*i) = [T(i)^5      T(i)^4    T(i)^3    T(i)^2    T(i)   1];
        R(2*i-0:2*i+0        ,6*i-5:6*i) = [T(i+1)^5    T(i+1)^4  T(i+1)^3  T(i+1)^2  T(i+1) 1];
        % Velocity
        R(2*n+2*i-1:2*n+2*i-1,6*i-5:6*i) = [5*T(i)^4	4*T(i)^3	3*T(i)^2      2*T(i)	1 0];
        R(2*n+2*i-0:2*n+2*i-0,6*i-5:6*i) = [5*T(i+1)^4	4*T(i+1)^3	3*T(i+1)^2    2*T(i+1)  1 0];
        % Accelaration
        R(4*n+2*i-1:4*n+2*i-1,6*i-5:6*i) = [20*T(i)^3 	12*T(i)^2 	6*T(i)      2	0 0];
        R(4*n+2*i-0:4*n+2*i-0,6*i-5:6*i) = [20*T(i+1)^3	12*T(i+1)^2	6*T(i+1)	2	0 0];
    end
    
    % Boundary Conditions Matrix
    BC = zeros(6*n,1);
    BC(1,1) = Pos(:,1);      	% Position of the first waypoint
    BC(1+2*n:1+2*n) = Vel(:,1);	% Velocity of the first waypoint
    BC(1+4*n:1+4*n) = Acc(:,1);	% Acceleration of the first waypoint
    if n > 1
        PosInter = zeros(2*(n-1),1);
        VelInter = zeros(2*(n-1),1);
        AccInter = zeros(2*(n-1),1);
    end
    for i = 2:n
        PosInter(2*i-3:2*i-2,:) = [Pos(:,i)';Pos(:,i)'];
        VelInter(2*i-3:2*i-2,:) = [Vel(:,i)';Vel(:,i)'];
        AccInter(2*i-3:2*i-2,:) = [Acc(:,i)';Acc(:,i)'];
    end
    for i = 1:2*(n-1)
        % Position
        BC(1+i:i+1) = PosInter(i,:);
        % Velocity
        BC(1+2*n+i:2*n+i+1) = VelInter(i,:);
        % Acceelration
        BC(1+4*n+i:4*n+i+1) = AccInter(i,:);
    end
    BC(1+2*n-1:2*n+0) = Pos(:,n+1); % Position of the final waypoint
    BC(1+2*n+1:2*n+2) = Vel(:,n+1); % Velocity of the final waypoint
    BC(1+2*n+3:2*n+4) = Acc(:,n+1); % Acceleration of the final waypoint

    % Coefficient  Vector
    Cj = R\BC;
    
end