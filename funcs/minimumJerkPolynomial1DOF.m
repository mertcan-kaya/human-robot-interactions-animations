function [Pj,Vj,Aj,Jj] = minimumJerkPolynomial1DOF(t,T,Cj)

    n = length(T)-1;  % n = number of trajectories
    
    % Trajectory Polynomial
    for j = n+1:-1:2
        if t <= T(j)
            i = j-1;
        end
    end

    Pj = Cj(1+6*(i-1))*t^5 + Cj(2+6*(i-1))*t^4 + Cj(3+6*(i-1))*t^3 + Cj(4+6*(i-1))*t^2 + Cj(5+6*(i-1))*t + Cj(6+6*(i-1));
    Vj = 5*Cj(1+6*(i-1))*t^4 + 4*Cj(2+6*(i-1))*t^3 + 3*Cj(3+6*(i-1))*t^2 + 2*Cj(4+6*(i-1))*t + Cj(5+6*(i-1));
    Aj = 5*4*Cj(1+6*(i-1))*t^3 + 4*3*Cj(2+6*(i-1))*t^2 + 3*2*Cj(3+6*(i-1))*t + 2*Cj(4+6*(i-1));
    Jj = 5*4*3*Cj(1+6*(i-1))*t^2 + 4*3*2*Cj(2+6*(i-1))*t + 3*2*Cj(3+6*(i-1));

end