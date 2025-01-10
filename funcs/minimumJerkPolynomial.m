function [Pt,Vt,At,Jt] = minimumJerkPolynomial(t,T,Cj)

    n = length(T)-1;  % n = number of trajectories
    d = length(Cj)/(6*n);
    
    % Trajectory Polynomial
    for j = n+1:-1:2
        if t <= T(j)
            i = j-1;
        end
    end
    
    Pt = Cj(1+d*(6*i-6):d*(6*i-5))*t^5 + Cj(1+d*(6*i-5):d*(6*i-4))*t^4 + Cj(1+d*(6*i-4):d*(6*i-3))*t^3 + Cj(1+d*(6*i-3):d*(6*i-2))*t^2 + Cj(1+d*(6*i-2):d*(6*i-1))*t + Cj(1+d*(6*i-1):d*(6*i));
    Vt = 5*Cj(1+d*(6*i-6):d*(6*i-5))*t^4 + 4*Cj(1+d*(6*i-5):d*(6*i-4))*t^3 + 3*Cj(1+d*(6*i-4):d*(6*i-3))*t^2 + 2*Cj(1+d*(6*i-3):d*(6*i-2))*t + Cj(1+d*(6*i-2):d*(6*i-1));
    At = 5*4*Cj(1+d*(6*i-6):d*(6*i-5))*t^3 + 4*3*Cj(1+d*(6*i-5):d*(6*i-4))*t^2 + 3*2*Cj(1+d*(6*i-4):d*(6*i-3))*t + 2*Cj(1+d*(6*i-3):d*(6*i-2));
    Jt = 5*4*3*Cj(1+d*(6*i-6):d*(6*i-5))*t^2 + 4*3*2*Cj(1+d*(6*i-5):d*(6*i-4))*t + 3*2*Cj(1+d*(6*i-4):d*(6*i-3));

end