function Cj = minimumJerkConstrCoefficient(T,Pos,Vel,Acc)

    [d,p] = size(Pos);  % d = # of dimensions, p = # of waypoints
    I = eye(d);
    Z = zeros(d);
    n = p-1;  % n = number of trajectories 

    % Waypoints' Matrix
    R = zeros(6*n*d,6*n*d); % Start with zero matrix for R, joint
    
    % Positions of all waypoints
    for i = 1:n
        % Position
        R(1+d*(2*i-2):d*(2*i-1)        ,1+6*d*(i-1):6*d*i) = [I*T(i)^5      I*T(i)^4    I*T(i)^3    I*T(i)^2    I*T(i)   I];
        R(1+d*(2*i-1):d*(2*i+0)        ,1+6*d*(i-1):6*d*i) = [I*T(i+1)^5    I*T(i+1)^4  I*T(i+1)^3  I*T(i+1)^2  I*T(i+1) I];
        % Velocity
        R(1+d*(2*n+2*i-2):d*(2*n+2*i-1),1+6*d*(i-1):6*d*i) = [5*I*T(i)^4 	4*I*T(i)^3      3*I*T(i)^2      2*I*T(i)	I Z];
        R(1+d*(2*n+2*i-1):d*(2*n+2*i-0),1+6*d*(i-1):6*d*i) = [5*I*T(i+1)^4	4*I*T(i+1)^3	3*I*T(i+1)^2    2*I*T(i+1)  I Z];
        % Accelaration
        R(1+d*(4*n+2*i-2):d*(4*n+2*i-1),1+6*d*(i-1):6*d*i) = [20*I*T(i)^3	12*I*T(i)^2     6*I*T(i)	2*I	Z Z];
        R(1+d*(4*n+2*i-1):d*(4*n+2*i-0),1+6*d*(i-1):6*d*i) = [20*I*T(i+1)^3	12*I*T(i+1)^2	6*I*T(i+1)	2*I	Z Z];
    end
    
    % Boundary Conditions Matrix
    BC = zeros(6*d*n,1);
    BC(1:d) = Pos(:,1);                 % Position of the first waypoint
    BC(1+2*d*n:d*(1+2*n)) = Vel(:,1);	% Velocity of the first waypoint
    BC(1+4*d*n:d*(1+4*n)) = Acc(:,1);	% Acceleration of the first waypoint
    if n > 1
        PosInter = zeros(2*(n-1),d);
        VelInter = zeros(2*(n-1),d);
        AccInter = zeros(2*(n-1),d);
    end
    for i = 2:n
        PosInter(2*(i-2)+1:2*(i-2)+2,:) = [Pos(:,i)';Pos(:,i)'];
        VelInter(2*(i-2)+1:2*(i-2)+2,:) = [Vel(:,i)';Vel(:,i)'];
        AccInter(2*(i-2)+1:2*(i-2)+2,:) = [Acc(:,i)';Acc(:,i)'];
    end
    for i = 1:2*(n-1)
        % Position
        BC(1+d*i:d*(i+1)) = PosInter(i,:);
        % Velocity
        BC(1+d*(2*n+i):d*(2*n+i+1)) = VelInter(i,:);
        % Acceelration
        BC(1+d*(4*n+i):d*(4*n+i+1)) = AccInter(i,:);
    end
    BC(1+d*(2*n-1):d*(2*n+0)) = Pos(:,n+1); % Position of the final waypoint
    BC(1+d*(2*n+1):d*(2*n+2)) = Vel(:,n+1); % Velocity of the final waypoint
    BC(1+d*(2*n+3):d*(2*n+4)) = Acc(:,n+1); % Acceleration of the final waypoint

    % Coefficient  Vector
    Cj = R\BC;
    
end