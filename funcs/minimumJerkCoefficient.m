function Cj = minimumJerkCoefficient(T,Pos,Vel,Acc)

    [d,p] = size(Pos);  % d = # of dimensions, p = # of waypoints
    I = eye(d);
    Z = zeros(d);
    n = p-1;  % n = number of trajectories 
    
    % Waypoints' Matrix
    R = zeros(6*n*d,6*n*d); % Start with zero matrix for R, joint
    % Positions of all waypoints
    for i = 1:n
        R(1+d*(2*i-2):d*(2*i-1),1+6*d*(i-1):6*d*i)  = [I*T(i)^5      I*T(i)^4    I*T(i)^3    I*T(i)^2    I*T(i)   I];
        R(1+d*(2*i-1):d*(2*i+0),1+6*d*(i-1):6*d*i)	= [I*T(i+1)^5    I*T(i+1)^4  I*T(i+1)^3  I*T(i+1)^2  I*T(i+1) I];
    end
    % Velocity boundary conditions (inital and final waypoints)
    R(1+d*(0+2*n):d*(1+2*n),1:6*d            ) = [5*I*T(1)^4 	4*I*T(1)^3      3*I*T(1)^2      2*I*T(1)	I Z];
    R(1+d*(1+2*n):d*(2+2*n),1+6*d*(n-1):6*d*n) = [5*I*T(n+1)^4	4*I*T(n+1)^3	3*I*T(n+1)^2    2*I*T(n+1)  I Z];
    % Equal Accelaration boundary conditions (initial and final waypoints)
    R(1+d*(2+2*n):d*(3+2*n),1:6*d            ) = [20*I*T(1)^3	12*I*T(1)^2     6*I*T(1)	2*I	Z Z];
    R(1+d*(3+2*n):d*(4+2*n),1+6*d*(n-1):6*d*n) = [20*I*T(n+1)^3	12*I*T(n+1)^2	6*I*T(n+1)	2*I	Z Z];
    %Equal velocity, accelaration , jerk, and snap at intermideate waypoints
    for i = 1:n-1
        R(1+d*(4+(i-1)+0*(n-1)+2*n):d*(5+(i-1)+0*(n-1)+2*n),1+6*d*(i-1):6*(i+1)*d) = [5*I*T(i+1)^4	4*I*T(i+1)^3	3*I*T(i+1)^2	2*I*T(i+1)	I Z	-5*I*T(i+1)^4	-4*I*T(i+1)^3	-3*I*T(i+1)^2	-2*I*T(i+1)	-I  Z]; % Equal velocity at intermediate waypoints
        R(1+d*(4+(i-1)+1*(n-1)+2*n):d*(5+(i-1)+1*(n-1)+2*n),1+6*d*(i-1):6*(i+1)*d) = [20*I*T(i+1)^3	12*I*T(i+1)^2	6*I*T(i+1)      2*I        	Z Z	-20*I*T(i+1)^3	-12*I*T(i+1)^2	-6*I*T(i+1)     -2*I      	Z   Z]; % Equal acceleration at intermediate waypoints
        R(1+d*(4+(i-1)+2*(n-1)+2*n):d*(5+(i-1)+2*(n-1)+2*n),1+6*d*(i-1):6*(i+1)*d) = [60*I*T(i+1)^2	24*I*T(i+1)     6*I           	Z           Z Z	-60*I*T(i+1)^2	-24*I*T(i+1)	-6*I            Z           Z   Z]; % Equal jerk at intermediate waypoints
        R(1+d*(4+(i-1)+3*(n-1)+2*n):d*(5+(i-1)+3*(n-1)+2*n),1+6*d*(i-1):6*(i+1)*d) = [120*I*T(i+1)	24*I        	Z               Z           Z Z -120*I*T(i+1)	-24*I         	Z               Z           Z   Z]; % Equal snap at intermediate waypoints
    end

    % Boundary Conditions Matrix
    if n > 1
        PosInter = zeros(2*(n-1),d);
    end
    for i = 2:n
        PosInter(2*(i-2)+1:2*(i-2)+2,:) = [Pos(:,i)';Pos(:,i)'];
    end
    BC = zeros(6*d*n,1);
    BC(1:d) = Pos(:,1);             % Position of the first waypoint
    for i = 1:2*(n-1)
        BC(1+d*i:d*(i+1)) = PosInter(i,:);
    end
    BC(1+d*(1+2*(n-1)):d*(2+2*(n-1))) = Pos(:,n+1); % Position of the final waypoint
    BC(1+d*(2+2*(n-1)):d*(3+2*(n-1))) = Vel(:,1);   % initial velocity
    BC(1+d*(3+2*(n-1)):d*(4+2*(n-1))) = Vel(:,2);   % final velocity
    BC(1+d*(4+2*(n-1)):d*(5+2*(n-1))) = Acc(:,1);   % initial acceleration
    BC(1+d*(5+2*(n-1)):d*(6+2*(n-1))) = Acc(:,2);   % final acceleration
    
    % Coefficient  Vector
    Cj = R\BC;
    
end